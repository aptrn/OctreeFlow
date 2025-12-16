using OctreeFlow.Data;
using System.Collections.Concurrent;

namespace OctreeFlow.Api;

/// <summary>
/// Automatic GPU buffer manager that handles loading/unloading of nodes.
/// Just throw a list of nodes at it each frame and it manages what to upload.
/// Uses LRU eviction when buffer is full, prioritizes by node level.
/// </summary>
public class GpuLoader : IDisposable
{
    private readonly CacheManager _cache;
    private readonly int _maxSectors;
    private readonly int _sectorSizePoints;
    private readonly int _bytesPerPoint;

    // Sector state
    private readonly GpuLoaderSector[] _sectors;
    private readonly Dictionary<string, int> _nodeToSector = new();
    private readonly LinkedList<string> _lruList = new();
    private readonly object _lock = new();

    // Change tracking for this frame
    private readonly List<SectorUpload> _pendingUploads = new();
    private readonly List<int> _releasedSectors = new();

    private int _version;
    private int _frameVersion;

    /// <summary>
    /// Current version (increments on any change).
    /// </summary>
    public int Version => _version;

    /// <summary>
    /// Number of sectors.
    /// </summary>
    public int SectorCount => _maxSectors;

    /// <summary>
    /// Points per sector.
    /// </summary>
    public int SectorSizePoints => _sectorSizePoints;

    /// <summary>
    /// Bytes per point.
    /// </summary>
    public int BytesPerPoint => _bytesPerPoint;

    /// <summary>
    /// Number of active sectors.
    /// </summary>
    public int ActiveSectorCount
    {
        get
        {
            lock (_lock)
            {
                return _sectors.Count(s => s.IsActive);
            }
        }
    }

    /// <summary>
    /// Total points currently on GPU.
    /// </summary>
    public int TotalPointsOnGpu
    {
        get
        {
            lock (_lock)
            {
                return _sectors.Where(s => s.IsActive).Sum(s => s.PointCount);
            }
        }
    }

    /// <summary>
    /// Creates a GPU loader.
    /// </summary>
    /// <param name="cache">RAM cache to read point data from.</param>
    /// <param name="maxBufferSizeMB">Maximum GPU buffer size in MB.</param>
    /// <param name="maxPointsPerNode">Maximum points per node (sector size).</param>
    /// <param name="bytesPerPoint">Bytes per point (default 44 for pos+color+normal+intensity).</param>
    public GpuLoader(CacheManager cache, int maxBufferSizeMB, int maxPointsPerNode, int bytesPerPoint = 44)
    {
        _cache = cache;
        _sectorSizePoints = maxPointsPerNode;
        _bytesPerPoint = bytesPerPoint;

        int sectorSizeBytes = _sectorSizePoints * _bytesPerPoint;
        _maxSectors = (maxBufferSizeMB * 1024 * 1024) / sectorSizeBytes;
        if (_maxSectors < 1) _maxSectors = 1;

        _sectors = new GpuLoaderSector[_maxSectors];
        for (int i = 0; i < _maxSectors; i++)
        {
            _sectors[i] = new GpuLoaderSector
            {
                Index = i,
                ByteOffset = i * sectorSizeBytes,
                IsActive = false
            };
        }
    }

    /// <summary>
    /// Updates the GPU state to match the desired node list.
    /// Call this each frame with the nodes you want visible.
    /// Returns what needs to be uploaded/released.
    /// </summary>
    /// <param name="desiredNodes">Nodes that should be on GPU (in priority order - first = highest).</param>
    /// <returns>Update result with uploads needed and sectors released.</returns>
    public GpuUpdateResult Update(IEnumerable<NodeInfo> desiredNodes)
    {
        var sw = System.Diagnostics.Stopwatch.StartNew();
        var result = new GpuUpdateResult();

        lock (_lock)
        {
            _pendingUploads.Clear();
            _releasedSectors.Clear();
            _frameVersion++;

            var desiredSet = new HashSet<string>();
            var orderedNodes = desiredNodes.ToList();

            // Mark desired nodes
            foreach (var node in orderedNodes)
            {
                desiredSet.Add(node.Id);
            }

            // Release nodes that are no longer desired
            var toRelease = _nodeToSector.Keys
                .Where(id => !desiredSet.Contains(id))
                .ToList();

            foreach (var nodeId in toRelease)
            {
                ReleaseSectorInternal(nodeId);
                result.NodesReleased++;
            }

            // Load new nodes (in priority order)
            foreach (var node in orderedNodes)
            {
                if (_nodeToSector.ContainsKey(node.Id))
                {
                    // Already loaded - just touch LRU
                    TouchLru(node.Id);
                    result.NodesAlreadyLoaded++;
                }
                else
                {
                    // Try to load
                    var upload = TryLoadNode(node);
                    if (upload != null)
                    {
                        _pendingUploads.Add(upload);
                        result.NodesLoaded++;
                    }
                    else
                    {
                        result.NodesSkipped++;
                    }
                }
            }

            result.Uploads = _pendingUploads.ToArray();
            result.ReleasedSectors = _releasedSectors.ToArray();
            result.ActiveSectors = GetActiveSectorsInternal();
            result.Version = _version;
        }

        sw.Stop();
        result.UpdateTimeMs = sw.ElapsedMilliseconds;
        result.TotalPointsOnGpu = TotalPointsOnGpu;

        return result;
    }

    /// <summary>
    /// Tries to load a node. Evicts LRU if needed.
    /// </summary>
    private SectorUpload? TryLoadNode(NodeInfo node)
    {
        // Check if in cache
        var pointData = _cache.GetPointData(node.Id);
        if (pointData == null)
        {
            // Not in cache - can't load to GPU
            return null;
        }

        // Find or make a sector
        int sectorIndex = FindEmptySector();
        if (sectorIndex < 0)
        {
            // Try to evict LRU
            sectorIndex = EvictLruSector();
        }

        if (sectorIndex < 0)
        {
            // No space and nothing to evict (shouldn't happen normally)
            return null;
        }

        // Allocate sector
        var sector = _sectors[sectorIndex];
        sector.IsActive = true;
        sector.NodeId = node.Id;
        sector.PointCount = pointData.Length;
        sector.Level = node.Level;
        sector.FrameLoaded = _frameVersion;

        _nodeToSector[node.Id] = sectorIndex;
        AddToLru(node.Id);
        _version++;

        // Create upload data
        return new SectorUpload
        {
            SectorIndex = sectorIndex,
            ByteOffset = sector.ByteOffset,
            NodeId = node.Id,
            PointCount = pointData.Length,
            PointData = pointData,
            GpuData = GpuPointData.FromPointDataArray(pointData),
            RawBytes = GpuPointData.ToByteArray(pointData)
        };
    }

    private int FindEmptySector()
    {
        for (int i = 0; i < _sectors.Length; i++)
        {
            if (!_sectors[i].IsActive)
                return i;
        }
        return -1;
    }

    private int EvictLruSector()
    {
        if (_lruList.Count == 0)
            return -1;

        // Get least recently used
        var lruNodeId = _lruList.First?.Value;
        if (lruNodeId == null)
            return -1;

        // Release it
        if (_nodeToSector.TryGetValue(lruNodeId, out int sectorIndex))
        {
            ReleaseSectorInternal(lruNodeId);
            return sectorIndex;
        }

        return -1;
    }

    private void ReleaseSectorInternal(string nodeId)
    {
        if (_nodeToSector.TryGetValue(nodeId, out int sectorIndex))
        {
            _sectors[sectorIndex].IsActive = false;
            _sectors[sectorIndex].NodeId = null;
            _sectors[sectorIndex].PointCount = 0;
            _nodeToSector.Remove(nodeId);
            RemoveFromLru(nodeId);
            _releasedSectors.Add(sectorIndex);
            _version++;
        }
    }

    private void TouchLru(string nodeId)
    {
        var node = _lruList.Find(nodeId);
        if (node != null)
        {
            _lruList.Remove(node);
            _lruList.AddLast(nodeId);
        }
    }

    private void AddToLru(string nodeId)
    {
        _lruList.AddLast(nodeId);
    }

    private void RemoveFromLru(string nodeId)
    {
        _lruList.Remove(nodeId);
    }

    private SectorState[] GetActiveSectorsInternal()
    {
        return _sectors
            .Where(s => s.IsActive)
            .Select(s => new SectorState
            {
                SectorIndex = s.Index,
                ByteOffset = s.ByteOffset,
                NodeId = s.NodeId!,
                PointCount = s.PointCount,
                Level = s.Level
            })
            .ToArray();
    }

    /// <summary>
    /// Checks if a node is currently on GPU.
    /// </summary>
    public bool Contains(string nodeId)
    {
        lock (_lock)
        {
            return _nodeToSector.ContainsKey(nodeId);
        }
    }

    /// <summary>
    /// Gets all currently loaded node IDs.
    /// </summary>
    public string[] GetLoadedNodeIds()
    {
        lock (_lock)
        {
            return _nodeToSector.Keys.ToArray();
        }
    }

    /// <summary>
    /// Gets the sector index for a node, or -1 if not loaded.
    /// </summary>
    public int GetSectorFor(string nodeId)
    {
        lock (_lock)
        {
            return _nodeToSector.TryGetValue(nodeId, out int idx) ? idx : -1;
        }
    }

    /// <summary>
    /// Clears all sectors.
    /// </summary>
    public void Clear()
    {
        lock (_lock)
        {
            foreach (var sector in _sectors)
            {
                sector.IsActive = false;
                sector.NodeId = null;
                sector.PointCount = 0;
            }
            _nodeToSector.Clear();
            _lruList.Clear();
            _version++;
        }
    }

    public void Dispose()
    {
        Clear();
    }

    private class GpuLoaderSector
    {
        public int Index;
        public int ByteOffset;
        public bool IsActive;
        public string? NodeId;
        public int PointCount;
        public int Level;
        public int FrameLoaded;
    }
}

/// <summary>
/// Result of a GPU update operation.
/// </summary>
public class GpuUpdateResult
{
    /// <summary>
    /// Version number.
    /// </summary>
    public int Version { get; set; }

    /// <summary>
    /// Time taken in milliseconds.
    /// </summary>
    public long UpdateTimeMs { get; set; }

    /// <summary>
    /// Sectors that need data uploaded.
    /// </summary>
    public SectorUpload[] Uploads { get; set; } = Array.Empty<SectorUpload>();

    /// <summary>
    /// Sector indices that were released (can be cleared if needed).
    /// </summary>
    public int[] ReleasedSectors { get; set; } = Array.Empty<int>();

    /// <summary>
    /// Current active sectors after update.
    /// </summary>
    public SectorState[] ActiveSectors { get; set; } = Array.Empty<SectorState>();

    /// <summary>
    /// Number of nodes newly loaded.
    /// </summary>
    public int NodesLoaded { get; set; }

    /// <summary>
    /// Number of nodes already on GPU.
    /// </summary>
    public int NodesAlreadyLoaded { get; set; }

    /// <summary>
    /// Number of nodes released.
    /// </summary>
    public int NodesReleased { get; set; }

    /// <summary>
    /// Number of nodes skipped (not in cache or no space).
    /// </summary>
    public int NodesSkipped { get; set; }

    /// <summary>
    /// Total points currently on GPU.
    /// </summary>
    public int TotalPointsOnGpu { get; set; }

    /// <summary>
    /// True if there are uploads pending.
    /// </summary>
    public bool HasUploads => Uploads.Length > 0;
}

/// <summary>
/// Data for uploading to a GPU sector.
/// </summary>
public class SectorUpload
{
    /// <summary>
    /// Sector index.
    /// </summary>
    public int SectorIndex { get; set; }

    /// <summary>
    /// Byte offset in GPU buffer.
    /// </summary>
    public int ByteOffset { get; set; }

    /// <summary>
    /// Node ID being uploaded.
    /// </summary>
    public string NodeId { get; set; } = "";

    /// <summary>
    /// Number of points.
    /// </summary>
    public int PointCount { get; set; }

    /// <summary>
    /// Original point data.
    /// </summary>
    public PointData[] PointData { get; set; } = Array.Empty<PointData>();

    /// <summary>
    /// GPU-formatted point data.
    /// </summary>
    public GpuPointData[] GpuData { get; set; } = Array.Empty<GpuPointData>();

    /// <summary>
    /// Raw bytes ready for GPU upload.
    /// </summary>
    public byte[] RawBytes { get; set; } = Array.Empty<byte>();
}

/// <summary>
/// State of an active sector.
/// </summary>
public struct SectorState
{
    /// <summary>
    /// Sector index.
    /// </summary>
    public int SectorIndex { get; set; }

    /// <summary>
    /// Byte offset in buffer.
    /// </summary>
    public int ByteOffset { get; set; }

    /// <summary>
    /// Node ID in this sector.
    /// </summary>
    public string NodeId { get; set; }

    /// <summary>
    /// Number of points.
    /// </summary>
    public int PointCount { get; set; }

    /// <summary>
    /// Node level in octree.
    /// </summary>
    public int Level { get; set; }
}

