using OctreeFlow.Data;
using System.Collections.Concurrent;

namespace OctreeFlow.Api;

/// <summary>
/// Manages GPU buffer sectors. Each sector can hold one node's worth of point data.
/// Sectors can be individually updated without clearing the entire buffer.
/// </summary>
public class GpuSectorManager : IDisposable
{
    private readonly int _maxBufferSizeBytes;
    private readonly int _sectorSizePoints;
    private readonly int _bytesPerPoint;
    private readonly GpuSector[] _sectors;
    private readonly ConcurrentDictionary<string, int> _nodeToSector = new();
    private readonly object _allocationLock = new();
    private int _version;

    /// <summary>
    /// Current version (increments on each change).
    /// </summary>
    public int Version => _version;

    /// <summary>
    /// Number of sectors in the buffer.
    /// </summary>
    public int SectorCount => _sectors.Length;

    /// <summary>
    /// Points per sector.
    /// </summary>
    public int SectorSizePoints => _sectorSizePoints;

    /// <summary>
    /// Total buffer size in bytes.
    /// </summary>
    public int TotalBufferSizeBytes => _sectors.Length * _sectorSizePoints * _bytesPerPoint;

    /// <summary>
    /// Number of active (occupied) sectors.
    /// </summary>
    public int ActiveSectorCount => _sectors.Count(s => s.IsActive);

    /// <summary>
    /// Creates a GPU sector manager.
    /// </summary>
    /// <param name="maxBufferSizeMB">Maximum GPU buffer size in megabytes.</param>
    /// <param name="maxPointsPerNode">Maximum points per node (determines sector size).</param>
    /// <param name="bytesPerPoint">Bytes per point in GPU format (default: 48 for pos+color+normal+intensity).</param>
    public GpuSectorManager(int maxBufferSizeMB, int maxPointsPerNode, int bytesPerPoint = 48)
    {
        _maxBufferSizeBytes = maxBufferSizeMB * 1024 * 1024;
        _sectorSizePoints = maxPointsPerNode;
        _bytesPerPoint = bytesPerPoint;

        // Calculate sector size in bytes
        int sectorSizeBytes = _sectorSizePoints * _bytesPerPoint;

        // Calculate number of sectors
        int sectorCount = _maxBufferSizeBytes / sectorSizeBytes;
        if (sectorCount < 1) sectorCount = 1;

        _sectors = new GpuSector[sectorCount];
        for (int i = 0; i < sectorCount; i++)
        {
            _sectors[i] = new GpuSector
            {
                Index = i,
                StartOffset = i * sectorSizeBytes,
                MaxPoints = _sectorSizePoints,
                IsActive = false,
                NodeId = null,
                PointCount = 0
            };
        }
    }

    /// <summary>
    /// Gets the sector index for a node, or -1 if not loaded.
    /// </summary>
    public int GetSectorForNode(string nodeId)
    {
        return _nodeToSector.TryGetValue(nodeId, out int index) ? index : -1;
    }

    /// <summary>
    /// Checks if a node is loaded on GPU.
    /// </summary>
    public bool Contains(string nodeId) => _nodeToSector.ContainsKey(nodeId);

    /// <summary>
    /// Allocates a sector for a node. Returns the sector index, or -1 if no space.
    /// If the node is already loaded, returns its existing sector.
    /// </summary>
    public int AllocateSector(string nodeId, int pointCount)
    {
        // Check if already allocated
        if (_nodeToSector.TryGetValue(nodeId, out int existingIndex))
        {
            return existingIndex;
        }

        lock (_allocationLock)
        {
            // Double-check after lock
            if (_nodeToSector.TryGetValue(nodeId, out existingIndex))
            {
                return existingIndex;
            }

            // Find an empty sector
            for (int i = 0; i < _sectors.Length; i++)
            {
                if (!_sectors[i].IsActive)
                {
                    _sectors[i].IsActive = true;
                    _sectors[i].NodeId = nodeId;
                    _sectors[i].PointCount = pointCount;
                    _nodeToSector[nodeId] = i;
                    Interlocked.Increment(ref _version);
                    return i;
                }
            }

            // No empty sector found
            return -1;
        }
    }

    /// <summary>
    /// Releases a sector, making it available for reuse.
    /// </summary>
    public bool ReleaseSector(string nodeId)
    {
        if (_nodeToSector.TryRemove(nodeId, out int index))
        {
            lock (_allocationLock)
            {
                _sectors[index].IsActive = false;
                _sectors[index].NodeId = null;
                _sectors[index].PointCount = 0;
            }
            Interlocked.Increment(ref _version);
            return true;
        }
        return false;
    }

    /// <summary>
    /// Gets information about a specific sector.
    /// </summary>
    public GpuSector GetSector(int index)
    {
        if (index < 0 || index >= _sectors.Length)
            throw new ArgumentOutOfRangeException(nameof(index));
        return _sectors[index];
    }

    /// <summary>
    /// Gets all active sectors.
    /// </summary>
    public IEnumerable<GpuSector> GetActiveSectors()
    {
        return _sectors.Where(s => s.IsActive);
    }

    /// <summary>
    /// Gets all sector activation states (for output).
    /// </summary>
    public SectorActivation[] GetSectorActivations()
    {
        return _sectors.Select(s => new SectorActivation
        {
            SectorIndex = s.Index,
            IsActive = s.IsActive,
            NodeId = s.NodeId,
            PointCount = s.PointCount,
            StartOffset = s.StartOffset
        }).ToArray();
    }

    /// <summary>
    /// Clears all sectors.
    /// </summary>
    public void Clear()
    {
        lock (_allocationLock)
        {
            _nodeToSector.Clear();
            for (int i = 0; i < _sectors.Length; i++)
            {
                _sectors[i].IsActive = false;
                _sectors[i].NodeId = null;
                _sectors[i].PointCount = 0;
            }
        }
        Interlocked.Increment(ref _version);
    }

    /// <summary>
    /// Gets all node IDs currently on GPU.
    /// </summary>
    public IEnumerable<string> GetLoadedNodeIds() => _nodeToSector.Keys;

    public void Dispose()
    {
        Clear();
    }
}

/// <summary>
/// Represents a single GPU buffer sector.
/// </summary>
public class GpuSector
{
    /// <summary>
    /// Index of this sector.
    /// </summary>
    public int Index { get; set; }

    /// <summary>
    /// Byte offset in the GPU buffer where this sector starts.
    /// </summary>
    public int StartOffset { get; set; }

    /// <summary>
    /// Maximum number of points this sector can hold.
    /// </summary>
    public int MaxPoints { get; set; }

    /// <summary>
    /// Whether this sector is currently in use.
    /// </summary>
    public bool IsActive { get; set; }

    /// <summary>
    /// Node ID currently occupying this sector, or null if empty.
    /// </summary>
    public string? NodeId { get; set; }

    /// <summary>
    /// Number of points currently stored in this sector.
    /// </summary>
    public int PointCount { get; set; }
}

/// <summary>
/// Activation state of a GPU sector (for output).
/// </summary>
public struct SectorActivation
{
    /// <summary>
    /// Index of the sector.
    /// </summary>
    public int SectorIndex { get; set; }

    /// <summary>
    /// Whether the sector is active.
    /// </summary>
    public bool IsActive { get; set; }

    /// <summary>
    /// Node ID in this sector, or null if empty.
    /// </summary>
    public string? NodeId { get; set; }

    /// <summary>
    /// Number of points in this sector.
    /// </summary>
    public int PointCount { get; set; }

    /// <summary>
    /// Byte offset of this sector in the GPU buffer.
    /// </summary>
    public int StartOffset { get; set; }
}

/// <summary>
/// Result of a GPU loading operation.
/// </summary>
public class GpuLoadResult
{
    /// <summary>
    /// Incremental version number of the GPU load.
    /// </summary>
    public int Version { get; set; }

    /// <summary>
    /// Total loading time in milliseconds.
    /// </summary>
    public long LoadTimeMs { get; set; }

    /// <summary>
    /// Whether the loading process is complete.
    /// </summary>
    public bool IsComplete { get; set; }

    /// <summary>
    /// Sector activation states.
    /// </summary>
    public SectorActivation[] SectorActivations { get; set; } = Array.Empty<SectorActivation>();

    /// <summary>
    /// Number of nodes successfully loaded.
    /// </summary>
    public int NodesLoaded { get; set; }

    /// <summary>
    /// Total points loaded to GPU.
    /// </summary>
    public int TotalPointsLoaded { get; set; }

    /// <summary>
    /// Error message if loading failed.
    /// </summary>
    public string? Error { get; set; }
}

