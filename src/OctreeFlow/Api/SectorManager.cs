using OctreeFlow.Data;
using System.Collections.Concurrent;

namespace OctreeFlow.Api;

/// <summary>
/// Manages buffer sectors for point cloud data.
/// Handles sector allocation with LRU eviction.
/// Outputs data ready for vvvv gamma's DynamicBufferAdvanced.
/// 
/// Usage:
/// 1. Create with configuration
/// 2. Call Update() each frame with desired nodes
/// 3. Use the result to upload new sectors to your buffers
/// 4. Use ActiveSectors for rendering dispatch
/// </summary>
public class SectorManager : IDisposable
{
    private readonly CacheManager _cache;
    private readonly BufferConfiguration _config;

    // Sector state
    private readonly Sector[] _sectors;
    private readonly Dictionary<string, int> _nodeToSector = new();
    private readonly LinkedList<string> _lruList = new();
    private readonly object _lock = new();
    
    // Mutable output list for NewSectors
    private readonly List<SectorData> _newSectorsList = new();

    // Change tracking
    private readonly List<SectorData> _pendingUploads = new();
    private readonly List<int> _releasedSectors = new();

    // Available features (determined from PLY file)
    private readonly HashSet<string> _availableVector4Features = new();
    private readonly HashSet<string> _availableFloat32Features = new();

    private int _version;
    private int _frameVersion;

    /// <summary>
    /// Current version (increments on any change).
    /// </summary>
    public int Version => _version;

    /// <summary>
    /// Buffer configuration.
    /// </summary>
    public BufferConfiguration Configuration => _config;

    /// <summary>
    /// Number of sectors.
    /// </summary>
    public int SectorCount => _config.SectorCount;

    /// <summary>
    /// Points per sector.
    /// </summary>
    public int MaxPointsPerSector => _config.MaxPointsPerSector;

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
    /// Total points currently in buffer.
    /// </summary>
    public int TotalPointsInBuffer
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
    /// Creates a sector manager.
    /// </summary>
    /// <param name="cache">RAM cache to read point data from.</param>
    /// <param name="config">Buffer configuration.</param>
    public SectorManager(CacheManager cache, BufferConfiguration config)
    {
        _cache = cache;
        _config = config;

        _sectors = new Sector[config.SectorCount];
        for (int i = 0; i < config.SectorCount; i++)
        {
            _sectors[i] = new Sector
            {
                Index = i,
                ByteOffsetVector4 = i * config.MaxPointsPerSector * BufferConfiguration.BytesPerVector4,
                ByteOffsetFloat = i * config.MaxPointsPerSector * BufferConfiguration.BytesPerFloat,
                IsActive = false
            };
        }
    }

    /// <summary>
    /// Creates a sector manager with default configuration.
    /// </summary>
    /// <param name="cache">RAM cache to read point data from.</param>
    /// <param name="bufferSizeBytes">Buffer size in bytes.</param>
    /// <param name="maxPointsPerSector">Maximum points per sector.</param>
    public SectorManager(CacheManager cache, long bufferSizeBytes, int maxPointsPerSector = 65536)
        : this(cache, BufferConfiguration.FromBufferSize(bufferSizeBytes, maxPointsPerSector))
    {
    }

    /// <summary>
    /// Sets the available features based on what's in the PLY file.
    /// Call this after initialization with the feature names from OctreeFlowReader.
    /// </summary>
    /// <param name="vector4Features">Names of available Vector4 features (e.g., "Position", "Colors", "Normals").</param>
    /// <param name="float32Features">Names of available Float32 features (e.g., "Intensity", scalar names).</param>
    public void SetAvailableFeatures(IEnumerable<string> vector4Features, IEnumerable<string> float32Features)
    {
        _availableVector4Features.Clear();
        _availableFloat32Features.Clear();
        
        foreach (var f in vector4Features)
            _availableVector4Features.Add(f);
        
        foreach (var f in float32Features)
            _availableFloat32Features.Add(f);
    }

    /// <summary>
    /// Updates the buffer state to match the desired node list.
    /// Call this each frame with the nodes you want in the buffer.
    /// Returns data to upload and current sector states.
    /// </summary>
    /// <param name="desiredNodes">Nodes that should be in buffer (in priority order - first = highest priority).</param>
    /// <returns>Update result with new sectors to upload and active sector info.</returns>
    public BufferUpdateResult Update(IEnumerable<NodeInfo> desiredNodes)
    {
        var sw = System.Diagnostics.Stopwatch.StartNew();
        var result = new BufferUpdateResult();

        lock (_lock)
        {
            _pendingUploads.Clear();
            _releasedSectors.Clear();
            _newSectorsList.Clear();
            _frameVersion++;

            var desiredSet = new HashSet<string>();
            var orderedNodes = desiredNodes.ToList();

            // Build desired set
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
                    var sectorData = TryLoadNode(node);
                    if (sectorData != null)
                    {
                        _pendingUploads.Add(sectorData);
                        _newSectorsList.Add(sectorData);
                        result.NodesLoaded++;
                    }
                    else
                    {
                        result.NodesSkipped++;
                    }
                }
            }

            result.NewSectors = _newSectorsList;
            result.ReleasedSectors = _releasedSectors.ToArray();
            result.ActiveSectors = GetActiveSectorsInternal();
            result.Version = _version;
            result.TotalPointsInBuffer = _sectors.Where(s => s.IsActive).Sum(s => s.PointCount);
        }

        sw.Stop();
        result.UpdateTimeMs = sw.ElapsedMilliseconds;

        return result;
    }

    /// <summary>
    /// Tries to load a node. Evicts LRU if needed.
    /// </summary>
    private SectorData? TryLoadNode(NodeInfo node)
    {
        // Check if in cache
        var pointData = _cache.GetPointData(node.Id);
        if (pointData == null)
        {
            // Not in cache - can't load
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
            // No space and nothing to evict
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

        // Create sector data for upload
        return SectorData.FromPointData(
            sectorIndex,
            sector.ByteOffsetVector4,
            sector.ByteOffsetFloat,
            node.Id,
            node.Level,
            pointData,
            _availableVector4Features,
            _availableFloat32Features);
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

    private SectorInfo[] GetActiveSectorsInternal()
    {
        return _sectors
            .Where(s => s.IsActive)
            .Select(s => new SectorInfo
            {
                SectorIndex = s.Index,
                ByteOffsetVector4 = s.ByteOffsetVector4,
                ByteOffsetFloat = s.ByteOffsetFloat,
                StartIndex = s.Index * _config.MaxPointsPerSector,
                PointCount = s.PointCount,
                NodeId = s.NodeId!,
                Level = s.Level
            })
            .ToArray();
    }

    /// <summary>
    /// Checks if a node is currently in buffer.
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
    /// Gets info about a specific sector.
    /// </summary>
    public SectorInfo? GetSectorInfo(int sectorIndex)
    {
        lock (_lock)
        {
            if (sectorIndex < 0 || sectorIndex >= _sectors.Length)
                return null;

            var s = _sectors[sectorIndex];
            if (!s.IsActive)
                return null;

            return new SectorInfo
            {
                SectorIndex = s.Index,
                ByteOffsetVector4 = s.ByteOffsetVector4,
                ByteOffsetFloat = s.ByteOffsetFloat,
                StartIndex = s.Index * _config.MaxPointsPerSector,
                PointCount = s.PointCount,
                NodeId = s.NodeId!,
                Level = s.Level
            };
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

    private class Sector
    {
        public int Index;
        public int ByteOffsetVector4;
        public int ByteOffsetFloat;
        public bool IsActive;
        public string? NodeId;
        public int PointCount;
        public int Level;
        public int FrameLoaded;
    }
}
