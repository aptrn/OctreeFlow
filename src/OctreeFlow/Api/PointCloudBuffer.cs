using Stride.Core.Mathematics;
using Stride.Graphics;
using OctreeFlow.Data;
using System.Runtime.InteropServices;
using GraphicsBuffer = Stride.Graphics.Buffer;

namespace OctreeFlow.Api;

/// <summary>
/// GPU buffer for point cloud rendering.
/// Manages a structured buffer divided into sectors for efficient partial updates.
/// Each sector holds one octree node's worth of points.
/// </summary>
public class PointCloudBuffer : IDisposable
{
    private readonly GraphicsDevice _graphicsDevice;
    private GraphicsBuffer? _buffer;
    private readonly int _sectorSizePoints;
    private readonly int _sectorCount;
    private readonly int _bytesPerPoint;
    private readonly PointCloudSector[] _sectors;
    private readonly Dictionary<string, int> _nodeToSector = new();
    private readonly LinkedList<string> _lruList = new();
    private readonly object _lock = new();

    private int _version;
    private bool _isDisposed;

    /// <summary>
    /// The underlying GPU buffer. Use this for rendering.
    /// </summary>
    public GraphicsBuffer? Buffer => _buffer;

    /// <summary>
    /// Total capacity in points.
    /// </summary>
    public int TotalCapacity => _sectorSizePoints * _sectorCount;

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

    /// <summary>
    /// Creates a new point cloud buffer.
    /// </summary>
    /// <param name="graphicsDevice">The Stride graphics device.</param>
    /// <param name="maxBufferSizeMB">Maximum buffer size in megabytes.</param>
    /// <param name="maxPointsPerNode">Maximum points per octree node (sector size).</param>
    public PointCloudBuffer(GraphicsDevice graphicsDevice, int maxBufferSizeMB, int maxPointsPerNode)
    {
        _graphicsDevice = graphicsDevice;
        _sectorSizePoints = maxPointsPerNode;
        _bytesPerPoint = Marshal.SizeOf<GpuPoint>();

        // Calculate sector count based on buffer size
        int sectorSizeBytes = _sectorSizePoints * _bytesPerPoint;
        _sectorCount = (maxBufferSizeMB * 1024 * 1024) / sectorSizeBytes;
        if (_sectorCount < 1) _sectorCount = 1;

        // Initialize sectors
        _sectors = new PointCloudSector[_sectorCount];
        for (int i = 0; i < _sectorCount; i++)
        {
            _sectors[i] = new PointCloudSector
            {
                Index = i,
                StartIndex = i * _sectorSizePoints,
                IsActive = false
            };
        }

        // Create the GPU buffer
        CreateBuffer();
    }

    private void CreateBuffer()
    {
        int totalPoints = _sectorSizePoints * _sectorCount;
        
        // Create a structured buffer for GpuPoint data
        _buffer = GraphicsBuffer.New(
            _graphicsDevice,
            totalPoints * _bytesPerPoint,
            BufferFlags.ShaderResource | BufferFlags.StructuredBuffer,
            PixelFormat.None);
    }

    /// <summary>
    /// Updates the buffer to display the requested nodes.
    /// Automatically handles loading, eviction, and GPU uploads.
    /// </summary>
    /// <param name="commandList">Command list for GPU operations.</param>
    /// <param name="nodes">Nodes to display (in priority order).</param>
    /// <param name="cache">Cache manager to get point data from.</param>
    /// <returns>Update result with sector states.</returns>
    public BufferUpdateResult Update(CommandList commandList, IEnumerable<NodeInfo> nodes, CacheManager cache)
    {
        if (_isDisposed || _buffer == null)
            return new BufferUpdateResult { Error = "Buffer disposed" };

        var sw = System.Diagnostics.Stopwatch.StartNew();
        var result = new BufferUpdateResult();

        lock (_lock)
        {
            var desiredSet = new HashSet<string>();
            var orderedNodes = nodes.ToList();

            // Build desired set
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

            // Load new nodes (in priority order)
            foreach (var node in orderedNodes)
            {
                if (_nodeToSector.ContainsKey(node.Id))
                {
                    // Already loaded - touch LRU
                    TouchLru(node.Id);
                    result.NodesAlreadyLoaded++;
                }
                else
                {
                    // Get point data from cache
                    var pointData = cache.GetPointData(node.Id);
                    if (pointData == null || pointData.Length == 0)
                    {
                        result.NodesSkipped++;
                        continue;
                    }

                    // Find or evict a sector
                    int sectorIndex = FindOrEvictSector();
                    if (sectorIndex < 0)
                    {
                        result.NodesSkipped++;
                        continue;
                    }

                    // Upload to GPU
                    var sector = _sectors[sectorIndex];
                    var gpuPoints = ConvertToGpuPoints(pointData);
                    
                    UploadToSector(commandList, sectorIndex, gpuPoints);

                    // Update tracking
                    sector.IsActive = true;
                    sector.NodeId = node.Id;
                    sector.PointCount = gpuPoints.Length;
                    sector.Level = node.Level;
                    _nodeToSector[node.Id] = sectorIndex;
                    AddToLru(node.Id);
                    
                    result.SectorsUploaded++;
                    result.PointsUploaded += gpuPoints.Length;
                    _version++;
                }
            }

            // Build active sectors list
            result.ActiveSectors = _sectors
                .Where(s => s.IsActive)
                .Select(s => new ActiveSector
                {
                    SectorIndex = s.Index,
                    StartIndex = s.StartIndex,
                    PointCount = s.PointCount,
                    NodeId = s.NodeId!,
                    Level = s.Level
                })
                .ToArray();

            result.TotalPointsInBuffer = _sectors.Where(s => s.IsActive).Sum(s => s.PointCount);
        }

        sw.Stop();
        result.UpdateTimeMs = sw.ElapsedMilliseconds;
        result.Version = _version;

        return result;
    }

    private void UploadToSector(CommandList commandList, int sectorIndex, GpuPoint[] points)
    {
        if (_buffer == null) return;

        var sector = _sectors[sectorIndex];
        
        // Calculate byte offset
        int byteOffset = sector.StartIndex * _bytesPerPoint;
        
        // Upload to the specific sector region
        _buffer.SetData(commandList, points, byteOffset);
    }

    private int FindOrEvictSector()
    {
        // Find empty sector
        for (int i = 0; i < _sectors.Length; i++)
        {
            if (!_sectors[i].IsActive)
                return i;
        }

        // Evict LRU
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

    private void AddToLru(string nodeId)
    {
        _lruList.AddLast(nodeId);
    }

    private void RemoveFromLru(string nodeId)
    {
        _lruList.Remove(nodeId);
    }

    private static GpuPoint[] ConvertToGpuPoints(PointData[] points)
    {
        var result = new GpuPoint[points.Length];
        for (int i = 0; i < points.Length; i++)
        {
            result[i] = new GpuPoint
            {
                Position = points[i].Position,
                Color = points[i].Color,
                Normal = points[i].Normal,
                Intensity = points[i].Intensity
            };
        }
        return result;
    }

    /// <summary>
    /// Checks if a node is loaded.
    /// </summary>
    public bool Contains(string nodeId)
    {
        lock (_lock)
        {
            return _nodeToSector.ContainsKey(nodeId);
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
        if (_isDisposed) return;
        _isDisposed = true;
        
        _buffer?.Dispose();
        _buffer = null;
        Clear();
    }

    private class PointCloudSector
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
/// GPU-friendly point structure. Must match shader.
/// 48 bytes total, aligned to 16-byte boundary.
/// </summary>
[StructLayout(LayoutKind.Sequential, Pack = 4)]
public struct GpuPoint
{
    /// <summary>Position (12 bytes)</summary>
    public Vector3 Position;
    
    /// <summary>Padding for alignment (4 bytes)</summary>
    public float _pad0;
    
    /// <summary>Color RGBA (16 bytes)</summary>
    public Color4 Color;
    
    /// <summary>Normal (12 bytes)</summary>
    public Vector3 Normal;
    
    /// <summary>Intensity (4 bytes)</summary>
    public float Intensity;

    // Total: 48 bytes (aligned to 16)
}

/// <summary>
/// Result of a buffer update operation.
/// </summary>
public class BufferUpdateResult
{
    public int Version { get; set; }
    public long UpdateTimeMs { get; set; }
    public int SectorsUploaded { get; set; }
    public int SectorsReleased { get; set; }
    public int NodesAlreadyLoaded { get; set; }
    public int NodesSkipped { get; set; }
    public int PointsUploaded { get; set; }
    public int TotalPointsInBuffer { get; set; }
    public ActiveSector[] ActiveSectors { get; set; } = Array.Empty<ActiveSector>();
    public string? Error { get; set; }
}

/// <summary>
/// Information about an active sector for rendering.
/// </summary>
public struct ActiveSector
{
    /// <summary>Sector index.</summary>
    public int SectorIndex { get; set; }
    
    /// <summary>Start index in the buffer (first point).</summary>
    public int StartIndex { get; set; }
    
    /// <summary>Number of points in this sector.</summary>
    public int PointCount { get; set; }
    
    /// <summary>Node ID in this sector.</summary>
    public string NodeId { get; set; }
    
    /// <summary>Node level in octree.</summary>
    public int Level { get; set; }
}

