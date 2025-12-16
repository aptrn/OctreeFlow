using Stride.Core.Mathematics;
using Stride.Graphics;
using OctreeFlow.Data;
using System.Runtime.InteropServices;
using GraphicsBuffer = Stride.Graphics.Buffer;

namespace OctreeFlow.Api;

/// <summary>
/// Manages multiple GPU buffers for point cloud rendering.
/// Creates separate buffers for Position, Color, Normal, and extra scalar dimensions.
/// Uses VVVV Gamma compatible buffer patterns with IGraphicsDataProvider.
/// </summary>
public class PointCloudBuffers : IDisposable
{
    private readonly GraphicsDevice _graphicsDevice;
    private readonly int _sectorSizePoints;
    private readonly int _sectorCount;
    private readonly int _totalCapacity;

    // Individual attribute buffers
    private GraphicsBuffer? _positionBuffer;      // Vector3 per point
    private GraphicsBuffer? _colorBuffer;         // Vector3 per point (RGB)
    private GraphicsBuffer? _normalBuffer;        // Vector3 per point (optional)
    private readonly Dictionary<string, GraphicsBuffer> _scalarBuffers = new(); // float per point

    // Sector management
    private readonly BufferSector[] _sectors;
    private readonly Dictionary<string, int> _nodeToSector = new();
    private readonly LinkedList<string> _lruList = new();
    private readonly object _lock = new();

    // Available scalar properties from PLY
    private readonly List<string> _scalarProperties = new();
    private bool _hasNormals;

    private int _version;
    private bool _isDisposed;
    private bool _buffersCreated;

    #region Public Properties - Buffer References

    /// <summary>
    /// Position buffer (Vector3 per point). Bind to shader.
    /// </summary>
    public GraphicsBuffer? PositionBuffer => _positionBuffer;

    /// <summary>
    /// Color buffer (Vector3 RGB per point). Bind to shader.
    /// </summary>
    public GraphicsBuffer? ColorBuffer => _colorBuffer;

    /// <summary>
    /// Normal buffer (Vector3 per point). May be null if no normals.
    /// </summary>
    public GraphicsBuffer? NormalBuffer => _normalBuffer;

    /// <summary>
    /// Gets a scalar buffer by property name. Returns null if not available.
    /// </summary>
    public GraphicsBuffer? GetScalarBuffer(string propertyName)
    {
        return _scalarBuffers.TryGetValue(propertyName, out var buffer) ? buffer : null;
    }

    /// <summary>
    /// Names of available scalar properties.
    /// </summary>
    public IReadOnlyList<string> ScalarProperties => _scalarProperties;

    /// <summary>
    /// Whether normal buffer is available.
    /// </summary>
    public bool HasNormals => _hasNormals;

    /// <summary>
    /// All scalar buffers as a dictionary.
    /// </summary>
    public IReadOnlyDictionary<string, GraphicsBuffer> ScalarBuffers => _scalarBuffers;

    #endregion

    #region Public Properties - Buffer Info

    /// <summary>
    /// Total capacity in points.
    /// </summary>
    public int TotalCapacity => _totalCapacity;

    /// <summary>
    /// Number of sectors.
    /// </summary>
    public int SectorCount => _sectorCount;

    /// <summary>
    /// Points per sector.
    /// </summary>
    public int SectorSizePoints => _sectorSizePoints;

    /// <summary>
    /// Current version (increments on changes).
    /// </summary>
    public int Version => _version;

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
    public int TotalPointCount
    {
        get
        {
            lock (_lock)
            {
                return _sectors.Where(s => s.IsActive).Sum(s => s.PointCount);
            }
        }
    }

    #endregion

    /// <summary>
    /// Creates point cloud buffers.
    /// </summary>
    /// <param name="graphicsDevice">Stride GraphicsDevice.</param>
    /// <param name="maxBufferSizeMB">Maximum total buffer size in MB (shared across all buffers).</param>
    /// <param name="maxPointsPerNode">Maximum points per octree node (sector size).</param>
    public PointCloudBuffers(GraphicsDevice graphicsDevice, int maxBufferSizeMB, int maxPointsPerNode)
    {
        _graphicsDevice = graphicsDevice;
        _sectorSizePoints = maxPointsPerNode;

        // Calculate sector count - assume worst case of 4 buffers (pos + color + normal + 1 scalar)
        // Each buffer uses 12 bytes per point (Vector3) or 4 bytes (float)
        // Average ~10 bytes per point per buffer, assume 4 buffers = 40 bytes per point
        int bytesPerPointEstimate = 40;
        int sectorSizeBytes = _sectorSizePoints * bytesPerPointEstimate;
        _sectorCount = Math.Max(1, (maxBufferSizeMB * 1024 * 1024) / sectorSizeBytes);
        _totalCapacity = _sectorCount * _sectorSizePoints;

        // Initialize sectors
        _sectors = new BufferSector[_sectorCount];
        for (int i = 0; i < _sectorCount; i++)
        {
            _sectors[i] = new BufferSector
            {
                Index = i,
                StartIndex = i * _sectorSizePoints,
                IsActive = false
            };
        }
    }

    /// <summary>
    /// Initializes buffers based on available properties.
    /// Call this after you know what properties are available.
    /// </summary>
    /// <param name="hasNormals">Whether normal data is available.</param>
    /// <param name="scalarProperties">Names of extra scalar properties (e.g., "intensity").</param>
    public void CreateBuffers(bool hasNormals, IEnumerable<string>? scalarProperties = null)
    {
        if (_buffersCreated) return;

        _hasNormals = hasNormals;
        _scalarProperties.Clear();
        if (scalarProperties != null)
        {
            _scalarProperties.AddRange(scalarProperties);
        }

        // Create position buffer (always required)
        _positionBuffer = CreateVector3Buffer();

        // Create color buffer (always required)
        _colorBuffer = CreateVector3Buffer();

        // Create normal buffer (optional)
        if (_hasNormals)
        {
            _normalBuffer = CreateVector3Buffer();
        }

        // Create scalar buffers
        foreach (var propName in _scalarProperties)
        {
            _scalarBuffers[propName] = CreateFloatBuffer();
        }

        _buffersCreated = true;
    }

    private GraphicsBuffer CreateVector3Buffer()
    {
        var desc = new BufferDescription
        {
            SizeInBytes = _totalCapacity * 12, // 12 bytes per Vector3
            StructureByteStride = 12,
            BufferFlags = BufferFlags.ShaderResource | BufferFlags.StructuredBuffer,
            Usage = GraphicsResourceUsage.Default  // Required for SetData with offset
        };

        return GraphicsBuffer.New(_graphicsDevice, desc);
    }

    private GraphicsBuffer CreateFloatBuffer()
    {
        var desc = new BufferDescription
        {
            SizeInBytes = _totalCapacity * 4, // 4 bytes per float
            StructureByteStride = 4,
            BufferFlags = BufferFlags.ShaderResource | BufferFlags.StructuredBuffer,
            Usage = GraphicsResourceUsage.Default  // Required for SetData with offset
        };

        return GraphicsBuffer.New(_graphicsDevice, desc);
    }

    /// <summary>
    /// Uploads point data to the GPU buffers.
    /// Called by PointCloudLoader after loading from cache.
    /// </summary>
    public BufferUploadResult Upload(
        CommandList commandList,
        IEnumerable<NodeInfo> nodes,
        Func<string, PointData[]?> getPointData)
    {
        if (!_buffersCreated)
            return new BufferUploadResult { Error = "Buffers not created. Call CreateBuffers first." };

        var sw = System.Diagnostics.Stopwatch.StartNew();
        var result = new BufferUploadResult();

        lock (_lock)
        {
            var desiredSet = new HashSet<string>();
            var orderedNodes = nodes.ToList();

            foreach (var node in orderedNodes)
            {
                desiredSet.Add(node.Id);
            }

            // Release nodes no longer needed
            var toRelease = _nodeToSector.Keys.Where(id => !desiredSet.Contains(id)).ToList();
            foreach (var nodeId in toRelease)
            {
                ReleaseSectorInternal(nodeId);
                result.SectorsReleased++;
            }

            // Load new nodes
            foreach (var node in orderedNodes)
            {
                if (_nodeToSector.ContainsKey(node.Id))
                {
                    TouchLru(node.Id);
                    result.NodesAlreadyLoaded++;
                }
                else
                {
                    var pointData = getPointData(node.Id);
                    if (pointData == null || pointData.Length == 0)
                    {
                        result.NodesSkipped++;
                        continue;
                    }

                    int sectorIndex = FindOrEvictSector();
                    if (sectorIndex < 0)
                    {
                        result.NodesSkipped++;
                        continue;
                    }

                    // Upload to all buffers
                    UploadToSector(commandList, sectorIndex, pointData);

                    // Update tracking
                    var sector = _sectors[sectorIndex];
                    sector.IsActive = true;
                    sector.NodeId = node.Id;
                    sector.PointCount = pointData.Length;
                    sector.Level = node.Level;
                    _nodeToSector[node.Id] = sectorIndex;
                    AddToLru(node.Id);

                    result.SectorsUploaded++;
                    result.PointsUploaded += pointData.Length;
                    _version++;
                }
            }

            // Build result
            result.ActiveSectors = GetActiveSectorsInternal();
            result.TotalPointsInBuffer = _sectors.Where(s => s.IsActive).Sum(s => s.PointCount);
        }

        sw.Stop();
        result.UpdateTimeMs = sw.ElapsedMilliseconds;
        result.Version = _version;

        return result;
    }

    private void UploadToSector(CommandList commandList, int sectorIndex, PointData[] points)
    {
        var sector = _sectors[sectorIndex];
        int byteOffsetV3 = sector.StartIndex * 12;  // Vector3 = 12 bytes
        int byteOffsetF = sector.StartIndex * 4;    // float = 4 bytes

        // Extract and upload positions
        var positions = new Vector3[points.Length];
        for (int i = 0; i < points.Length; i++)
            positions[i] = points[i].Position;
        _positionBuffer?.SetData(commandList, positions, byteOffsetV3);

        // Extract and upload colors (as Vector3 RGB)
        var colors = new Vector3[points.Length];
        for (int i = 0; i < points.Length; i++)
            colors[i] = new Vector3(points[i].Color.R, points[i].Color.G, points[i].Color.B);
        _colorBuffer?.SetData(commandList, colors, byteOffsetV3);

        // Extract and upload normals
        if (_normalBuffer != null)
        {
            var normals = new Vector3[points.Length];
            for (int i = 0; i < points.Length; i++)
                normals[i] = points[i].Normal;
            _normalBuffer.SetData(commandList, normals, byteOffsetV3);
        }

        // Upload intensity to a scalar buffer if available
        if (_scalarBuffers.TryGetValue("intensity", out var intensityBuffer))
        {
            var intensity = new float[points.Length];
            for (int i = 0; i < points.Length; i++)
                intensity[i] = points[i].Intensity;
            intensityBuffer.SetData(commandList, intensity, byteOffsetF);
        }

        // Upload other scalars
        foreach (var propName in _scalarProperties)
        {
            if (propName == "intensity") continue; // Already handled
            if (!_scalarBuffers.TryGetValue(propName, out var buffer)) continue;

            var values = new float[points.Length];
            for (int i = 0; i < points.Length; i++)
                values[i] = points[i].GetScalar(propName);
            buffer.SetData(commandList, values, byteOffsetF);
        }
    }

    private int FindOrEvictSector()
    {
        for (int i = 0; i < _sectors.Length; i++)
        {
            if (!_sectors[i].IsActive)
                return i;
        }

        if (_lruList.Count > 0)
        {
            var oldestId = _lruList.First?.Value;
            if (oldestId != null && _nodeToSector.TryGetValue(oldestId, out int sectorIndex))
            {
                ReleaseSectorInternal(oldestId);
                return sectorIndex;
            }
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

    private void AddToLru(string nodeId) => _lruList.AddLast(nodeId);
    private void RemoveFromLru(string nodeId) => _lruList.Remove(nodeId);

    private PointBufferSector[] GetActiveSectorsInternal()
    {
        return _sectors
            .Where(s => s.IsActive)
            .Select(s => new PointBufferSector
            {
                SectorIndex = s.Index,
                StartIndex = s.StartIndex,
                PointCount = s.PointCount,
                NodeId = s.NodeId!,
                Level = s.Level
            })
            .ToArray();
    }

    /// <summary>
    /// Checks if a node is loaded.
    /// </summary>
    public bool Contains(string nodeId)
    {
        lock (_lock) { return _nodeToSector.ContainsKey(nodeId); }
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
        if (_isDisposed) return;
        _isDisposed = true;

        _positionBuffer?.Dispose();
        _colorBuffer?.Dispose();
        _normalBuffer?.Dispose();
        foreach (var buffer in _scalarBuffers.Values)
            buffer.Dispose();
        _scalarBuffers.Clear();
        Clear();
    }

    private class BufferSector
    {
        public int Index;
        public int StartIndex;
        public bool IsActive;
        public string? NodeId;
        public int PointCount;
        public int Level;
    }
}

/// <summary>
/// Information about an active buffer sector.
/// </summary>
public struct PointBufferSector
{
    /// <summary>Sector index.</summary>
    public int SectorIndex { get; set; }

    /// <summary>Start index in all buffers (first point).</summary>
    public int StartIndex { get; set; }

    /// <summary>Number of points in this sector.</summary>
    public int PointCount { get; set; }

    /// <summary>Node ID in this sector.</summary>
    public string NodeId { get; set; }

    /// <summary>Node level in octree.</summary>
    public int Level { get; set; }
}

/// <summary>
/// Result of buffer upload operation.
/// </summary>
public class BufferUploadResult
{
    public int Version { get; set; }
    public long UpdateTimeMs { get; set; }
    public int SectorsUploaded { get; set; }
    public int SectorsReleased { get; set; }
    public int NodesAlreadyLoaded { get; set; }
    public int NodesSkipped { get; set; }
    public int PointsUploaded { get; set; }
    public int TotalPointsInBuffer { get; set; }
    public PointBufferSector[] ActiveSectors { get; set; } = Array.Empty<PointBufferSector>();
    public string? Error { get; set; }
}

