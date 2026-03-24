using OctreeFlow.Data;
using System.Collections.Concurrent;

namespace OctreeFlow.Api;

/// <summary>
/// Manages buffer sectors for point cloud data with VARIABLE-SIZE sectors.
/// Each sector holds exactly the points its node has - no wasted space.
/// Outputs data ready for vvvv gamma's DynamicBufferAdvanced.
/// 
/// Usage:
/// 1. Create with max buffer capacity
/// 2. Call Update() each frame with desired nodes
/// 3. Use the result to upload data to your buffers
/// 4. Use ActiveSectors for rendering dispatch
/// </summary>
public class SectorManager : IDisposable
{
    private readonly CacheManager _cache;
    private readonly long _maxBufferCapacityPoints; // Max points that fit in buffer

    // Variable-size sector state - each node uses only the space it needs
    private readonly Dictionary<string, VariableSector> _activeNodes = new();
    private readonly LinkedList<string> _lruList = new();
    private readonly object _lock = new();
    
    private int _currentOffset = 0; // Current write position in buffer
    private int _totalPointsInBuffer = 0;
    
    // Mutable output list for NewSectors
    private readonly List<SectorData> _newSectorsList = new();

    // Available features (determined from PLY file)
    private readonly HashSet<string> _availableVector4Features = new();
    private readonly HashSet<string> _availableFloat32Features = new();
    private readonly HashSet<string> _availableInt32Features = new();

    // Diagnostic tracking
    private string? _lastSkipReason;

    // Current frame's desired set (for eviction protection)
    private HashSet<string>? _currentDesiredSet;

    private int _version;
    private int _frameVersion;
    
    // Flag to track if buffer needs compaction
    private bool _needsCompaction = false;

    /// <summary>
    /// Current version (increments on any change).
    /// </summary>
    public int Version => _version;

    /// <summary>
    /// Maximum buffer capacity in points.
    /// </summary>
    public long MaxBufferCapacityPoints => _maxBufferCapacityPoints;

    /// <summary>
    /// Number of active nodes in buffer.
    /// </summary>
    public int ActiveNodeCount
    {
        get
        {
            lock (_lock)
            {
                return _activeNodes.Count;
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
                return _totalPointsInBuffer;
            }
        }
    }

    /// <summary>
    /// Creates a sector manager with variable-size sectors.
    /// </summary>
    /// <param name="cache">RAM cache to read point data from.</param>
    /// <param name="maxBufferCapacityPoints">Maximum points that can fit in buffer.</param>
    public SectorManager(CacheManager cache, long maxBufferCapacityPoints)
    {
        _cache = cache;
        _maxBufferCapacityPoints = maxBufferCapacityPoints;
    }

    /// <summary>
    /// Creates a sector manager from buffer configuration (for backwards compatibility).
    /// </summary>
    /// <param name="cache">RAM cache to read point data from.</param>
    /// <param name="config">Buffer configuration - uses TotalCapacity as max points.</param>
    public SectorManager(CacheManager cache, BufferConfiguration config)
        : this(cache, config.TotalCapacity)
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
        SetAvailableFeatures(vector4Features, float32Features, Enumerable.Empty<string>());
    }

    /// <summary>
    /// Sets the available features based on what's in the PLY file.
    /// Call this after initialization with the feature names from OctreeFlowReader.
    /// </summary>
    /// <param name="vector4Features">Names of available Vector4 features (e.g., "Position", "Colors", "Normals").</param>
    /// <param name="float32Features">Names of available Float32 features (e.g., "Intensity", scalar names).</param>
    /// <param name="int32Features">Names of available Int32 features (e.g., "Id").</param>
    public void SetAvailableFeatures(IEnumerable<string> vector4Features, IEnumerable<string> float32Features, IEnumerable<string> int32Features)
    {
        _availableVector4Features.Clear();
        _availableFloat32Features.Clear();
        _availableInt32Features.Clear();
        
        foreach (var f in vector4Features)
            _availableVector4Features.Add(f);
        
        foreach (var f in float32Features)
            _availableFloat32Features.Add(f);
        
        foreach (var f in int32Features)
            _availableInt32Features.Add(f);
    }

    /// <summary>
    /// Updates the buffer state to match the desired node list.
    /// Uses VARIABLE-SIZE sectors - each node uses only the space it needs.
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
            _newSectorsList.Clear();
            _lastSkipReason = null;
            _frameVersion++;

            var desiredSet = new HashSet<string>();
            var orderedNodes = desiredNodes.ToList();

            // Build desired set and calculate total points needed
            int totalPointsNeeded = 0;
            foreach (var node in orderedNodes)
            {
                desiredSet.Add(node.Id);
                
                // Get point count from cache if not already loaded
                if (!_activeNodes.ContainsKey(node.Id))
                {
                    var pointData = _cache.GetPointData(node.Id);
                    if (pointData != null)
                    {
                        totalPointsNeeded += pointData.Length;
                    }
                }
                else
                {
                    totalPointsNeeded += _activeNodes[node.Id].PointCount;
                }
            }

            // Release nodes that are no longer desired
            var toRelease = _activeNodes.Keys
                .Where(id => !desiredSet.Contains(id))
                .ToList();

            foreach (var nodeId in toRelease)
            {
                ReleaseNodeInternal(nodeId);
                result.NodesReleased++;
            }

            // Store desired set for reference
            _currentDesiredSet = desiredSet;

            // Check if we need to rebuild the buffer (nodes changed)
            bool needsRebuild = toRelease.Count > 0 || 
                orderedNodes.Any(n => !_activeNodes.ContainsKey(n.Id));

            if (needsRebuild)
            {
                // Rebuild buffer from scratch with all desired nodes
                RebuildBuffer(orderedNodes, result);
            }
            else
            {
                // No changes - just update LRU
                foreach (var node in orderedNodes)
                {
                    if (_activeNodes.ContainsKey(node.Id))
                    {
                        TouchLru(node.Id);
                        result.NodesAlreadyLoaded++;
                    }
                }
            }

            _currentDesiredSet = null;

            // Build result
            result.NewSectors = _newSectorsList;
            result.AllActiveSectors = _activeNodes.Values
                .Select(v => v.SectorData)
                .Where(s => s != null)
                .Cast<SectorData>()
                .ToList();
            result.ActiveSectors = GetActiveSectorsInternal();
            result.Version = _version;
            result.TotalPointsInBuffer = _totalPointsInBuffer;
            
            // Diagnostic info
            result.TruncatedNodes = 0; // No truncation with variable sectors!
            result.TruncatedPoints = 0;
            result.LastSkipReason = _lastSkipReason;
            result.MaxPointsPerSector = 0; // Not applicable
            result.AvailableSectors = (int)_maxBufferCapacityPoints; // Total capacity
            result.UsedSectors = _activeNodes.Count;
        }

        sw.Stop();
        result.UpdateTimeMs = sw.ElapsedMilliseconds;

        return result;
    }

    /// <summary>
    /// Rebuilds the buffer from scratch with the given nodes.
    /// Packs nodes contiguously from offset 0.
    /// </summary>
    private void RebuildBuffer(List<NodeInfo> orderedNodes, BufferUpdateResult result)
    {
        // Clear current state
        _activeNodes.Clear();
        _lruList.Clear();
        _currentOffset = 0;
        _totalPointsInBuffer = 0;
        _version++;

        // Load nodes in order until buffer is full
        foreach (var node in orderedNodes)
        {
            var pointData = _cache.GetPointData(node.Id);
            if (pointData == null)
            {
                _lastSkipReason = $"Node {node.Id} not in cache";
                result.NodesSkipped++;
                continue;
            }

            int pointCount = pointData.Length;

            // Check if this node fits in remaining buffer space
            if (_totalPointsInBuffer + pointCount > _maxBufferCapacityPoints)
            {
                _lastSkipReason = $"Buffer full: {_totalPointsInBuffer}/{_maxBufferCapacityPoints} points, " +
                    $"cannot fit node {node.Id} with {pointCount} points";
                result.NodesSkipped++;
                continue;
            }

            // Calculate byte offsets
            int byteOffsetVector4 = _currentOffset * 16; // 16 bytes per Vector4
            int byteOffsetFloat = _currentOffset * 4;    // 4 bytes per float
            int byteOffsetInt32 = _currentOffset * 4;    // 4 bytes per int

            // Create sector data — node.NodeId is the integer DFS index used by Point_NodeID
            // so shaders can cross-reference BF buffers: BF_Density[Point_NodeID[pointId]].
            var sectorData = SectorData.FromPointData(
                _activeNodes.Count, // Use node index as "sector index"
                byteOffsetVector4,
                byteOffsetFloat,
                byteOffsetInt32,
                node.Id,
                node.NodeId,
                node.Level,
                pointData,
                _availableVector4Features,
                _availableFloat32Features,
                _availableInt32Features);

            // Create variable sector
            var sector = new VariableSector
            {
                NodeId = node.Id,
                StartOffset = _currentOffset,
                PointCount = pointCount,
                Level = node.Level,
                SectorData = sectorData
            };

            _activeNodes[node.Id] = sector;
            _lruList.AddLast(node.Id);
            _newSectorsList.Add(sectorData);

            _currentOffset += pointCount;
            _totalPointsInBuffer += pointCount;
            result.NodesLoaded++;
        }
    }

    private void ReleaseNodeInternal(string nodeId)
    {
        if (_activeNodes.TryGetValue(nodeId, out var sector))
        {
            _activeNodes.Remove(nodeId);
            RemoveFromLru(nodeId);
            _needsCompaction = true;
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

    private void RemoveFromLru(string nodeId)
    {
        _lruList.Remove(nodeId);
    }

    private SectorInfo[] GetActiveSectorsInternal()
    {
        int index = 0;
        return _activeNodes.Values
            .Select(s => new SectorInfo
            {
                SectorIndex = index++,
                ByteOffsetVector4 = s.StartOffset * 16,
                ByteOffsetFloat = s.StartOffset * 4,
                ByteOffsetInt32 = s.StartOffset * 4,
                StartIndex = s.StartOffset,
                PointCount = s.PointCount,
                NodeId = s.NodeId,
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
            return _activeNodes.ContainsKey(nodeId);
        }
    }

    /// <summary>
    /// Gets all currently loaded node IDs.
    /// </summary>
    public string[] GetLoadedNodeIds()
    {
        lock (_lock)
        {
            return _activeNodes.Keys.ToArray();
        }
    }

    /// <summary>
    /// Gets the sector index for a node, or -1 if not loaded.
    /// With variable sectors, returns the node's position in the active list.
    /// </summary>
    public int GetSectorFor(string nodeId)
    {
        lock (_lock)
        {
            if (!_activeNodes.ContainsKey(nodeId))
                return -1;
            
            int index = 0;
            foreach (var key in _activeNodes.Keys)
            {
                if (key == nodeId)
                    return index;
                index++;
            }
            return -1;
        }
    }

    /// <summary>
    /// Clears all nodes from buffer.
    /// </summary>
    public void Clear()
    {
        lock (_lock)
        {
            _activeNodes.Clear();
            _lruList.Clear();
            _currentOffset = 0;
            _totalPointsInBuffer = 0;
            _version++;
        }
    }

    /// <summary>
    /// Gets all active sector data as a list.
    /// Use this for GetCombinedAllActiveData() operations.
    /// </summary>
    internal List<SectorData> GetAllActiveSectorData()
    {
        lock (_lock)
        {
            return _activeNodes.Values
                .Select(v => v.SectorData)
                .Where(s => s != null)
                .Cast<SectorData>()
                .ToList();
        }
    }

    public void Dispose()
    {
        Clear();
    }

    /// <summary>
    /// Variable-size sector that holds exactly the points a node has.
    /// </summary>
    private class VariableSector
    {
        public required string NodeId;
        public int StartOffset; // Element offset in buffer
        public int PointCount;
        public int Level;
        public SectorData? SectorData;
    }
}
