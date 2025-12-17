using Stride.Graphics;
using OctreeFlow.Core;
using OctreeFlow.Data;
using OctreeFlow.IO;
using System.Diagnostics;
using GraphicsBuffer = Stride.Graphics.Buffer;

namespace OctreeFlow.Api;

/// <summary>
/// Main point cloud loader for VVVV Gamma.
/// 
/// Workflow:
/// 1. Create loader with paths and settings
/// 2. Call Initialize() once
/// 3. Each frame: Call Traverse() with your traversal delegate
/// 4. Call CacheAndUploadAsync() with the traversal result
/// 5. Use the buffer references for rendering
/// </summary>
public class PointCloudLoader : IDisposable
{
    private readonly string _octreePath;
    private readonly string _plyPath;
    private readonly int _cacheSizeMB;
    private readonly int _gpuBufferSizeMB;
    private readonly int _maxPointsPerNode;

    private GraphicsDevice? _graphicsDevice;
    private CacheManager? _cache;
    private PointCloudBuffers? _gpuBuffers;
    private OctreeNode? _root;
    private OctreeFileInfo? _fileInfo;
    private PlyIndex? _plyIndex;
    private Dictionary<string, NodeInfo> _nodeInfoCache = new();

    private int _traversalVersion;
    private bool _isInitialized;
    private bool _isDisposed;

    #region Public Properties

    /// <summary>
    /// Whether the loader is initialized.
    /// </summary>
    public bool IsInitialized => _isInitialized;

    /// <summary>
    /// Position buffer (Vector3 per point).
    /// </summary>
    public GraphicsBuffer? PositionBuffer => _gpuBuffers?.PositionBuffer;

    /// <summary>
    /// Color buffer (Vector3 RGB per point).
    /// </summary>
    public GraphicsBuffer? ColorBuffer => _gpuBuffers?.ColorBuffer;

    /// <summary>
    /// Normal buffer (Vector3 per point). May be null.
    /// </summary>
    public GraphicsBuffer? NormalBuffer => _gpuBuffers?.NormalBuffer;

    /// <summary>
    /// Whether normals are available.
    /// </summary>
    public bool HasNormals => _gpuBuffers?.HasNormals ?? false;

    /// <summary>
    /// Available scalar property names.
    /// </summary>
    public IReadOnlyList<string> ScalarProperties => _gpuBuffers?.ScalarProperties ?? Array.Empty<string>();

    /// <summary>
    /// Gets a scalar buffer by property name.
    /// </summary>
    public GraphicsBuffer? GetScalarBuffer(string name) => _gpuBuffers?.GetScalarBuffer(name);

    /// <summary>
    /// All scalar buffers.
    /// </summary>
    public IReadOnlyDictionary<string, GraphicsBuffer>? ScalarBuffers => _gpuBuffers?.ScalarBuffers;

    /// <summary>
    /// The GPU buffers manager (for advanced access).
    /// </summary>
    public PointCloudBuffers? GpuBuffers => _gpuBuffers;

    /// <summary>
    /// The RAM cache manager.
    /// </summary>
    public CacheManager? Cache => _cache;

    /// <summary>
    /// Total points in the point cloud.
    /// </summary>
    public int TotalPoints => _fileInfo?.TotalPoints ?? 0;

    /// <summary>
    /// Total nodes in the octree.
    /// </summary>
    public int TotalNodes => _fileInfo?.NodeCount ?? 0;

    /// <summary>
    /// Bounding box of the point cloud.
    /// </summary>
    public Stride.Core.Mathematics.BoundingBox Bounds => _fileInfo?.Bounds ?? new();

    /// <summary>
    /// Sector size (points per sector).
    /// </summary>
    public int SectorSizePoints => _gpuBuffers?.SectorSizePoints ?? _maxPointsPerNode;

    /// <summary>
    /// Number of GPU sectors.
    /// </summary>
    public int SectorCount => _gpuBuffers?.SectorCount ?? 0;

    #endregion

    /// <summary>
    /// Creates a point cloud loader.
    /// </summary>
    /// <param name="octreePath">Path to the .octree file.</param>
    /// <param name="plyPath">Path to the .ply file.</param>
    /// <param name="cacheSizeMB">RAM cache size in MB.</param>
    /// <param name="gpuBufferSizeMB">GPU buffer size in MB.</param>
    /// <param name="maxPointsPerNode">Maximum points per node (sector size).</param>
    public PointCloudLoader(
        string octreePath,
        string plyPath,
        int cacheSizeMB = 512,
        int gpuBufferSizeMB = 256,
        int maxPointsPerNode = 65536)
    {
        _octreePath = octreePath;
        _plyPath = plyPath;
        _cacheSizeMB = cacheSizeMB;
        _gpuBufferSizeMB = gpuBufferSizeMB;
        _maxPointsPerNode = maxPointsPerNode;
    }

    /// <summary>
    /// Initializes the loader. Must be called before Traverse/CacheAndUpload.
    /// </summary>
    /// <param name="graphicsDevice">Stride GraphicsDevice from VVVV.</param>
    public void Initialize(GraphicsDevice graphicsDevice)
    {
        if (_isInitialized) return;

        _graphicsDevice = graphicsDevice;

        // Load octree structure
        var serializer = new StreamingOctreeSerializer();
        var (root, info) = serializer.LoadOctreeFile(_octreePath);

        _root = root ?? throw new InvalidOperationException("Failed to load octree");
        _fileInfo = info;

        // Build PLY index (header only - no bounds scan needed)
        _plyIndex = new PlyIndex(_plyPath);
        _plyIndex.BuildIndexHeaderOnly();

        // Build node info cache
        BuildNodeInfoCache(_root);

        // Create RAM cache
        _cache = new CacheManager(_cacheSizeMB);

        // Create GPU buffers
        _gpuBuffers = new PointCloudBuffers(graphicsDevice, _gpuBufferSizeMB, _maxPointsPerNode);

        // Determine available properties and create buffers
        bool hasNormals = _plyIndex.Properties.Any(p =>
            p.Name.Equals("nx", StringComparison.OrdinalIgnoreCase) ||
            p.Name.Equals("ny", StringComparison.OrdinalIgnoreCase) ||
            p.Name.Equals("nz", StringComparison.OrdinalIgnoreCase));

        var knownProps = new HashSet<string>(StringComparer.OrdinalIgnoreCase)
        {
            "x", "y", "z", "red", "green", "blue", "r", "g", "b", "alpha", "a",
            "nx", "ny", "nz"
        };

        var scalarProps = _plyIndex.Properties
            .Where(p => !knownProps.Contains(p.Name))
            .Select(p => p.Name)
            .ToList();

        // Always include intensity if available
        if (_plyIndex.Properties.Any(p =>
            p.Name.Equals("intensity", StringComparison.OrdinalIgnoreCase) ||
            p.Name.Equals("scalar_intensity", StringComparison.OrdinalIgnoreCase)))
        {
            if (!scalarProps.Contains("intensity"))
                scalarProps.Insert(0, "intensity");
        }

        _gpuBuffers.CreateBuffers(hasNormals, scalarProps);

        _isInitialized = true;
    }

    /// <summary>
    /// Initializes the loader asynchronously.
    /// </summary>
    public async Task InitializeAsync(GraphicsDevice graphicsDevice, Action<string, int, int>? onProgress = null)
    {
        if (_isInitialized) return;

        await Task.Run(() =>
        {
            onProgress?.Invoke("Loading octree...", 0, 100);
            Initialize(graphicsDevice);
            onProgress?.Invoke("Ready", 100, 100);
        });
    }

    private void BuildNodeInfoCache(OctreeNode node)
    {
        var info = new NodeInfo(node);
        _nodeInfoCache[node.Id] = info;

        foreach (var child in node.Children)
        {
            BuildNodeInfoCache(child);
        }
    }

    /// <summary>
    /// Traverses the octree with the given delegate.
    /// Does NOT load any data - just determines what should be loaded.
    /// </summary>
    /// <param name="traversalDelegate">Your traversal logic.</param>
    /// <returns>Traversal result with lists of nodes to cache/display.</returns>
    public TraversalResult Traverse(TraversalDelegate traversalDelegate)
    {
        EnsureInitialized();

        var sw = Stopwatch.StartNew();
        var result = new TraversalResult
        {
            Version = Interlocked.Increment(ref _traversalVersion)
        };

        if (_root == null)
        {
            result.IsComplete = true;
            result.TraversalTimeMs = sw.ElapsedMilliseconds;
            return result;
        }

        TraverseNode(_root, traversalDelegate, result);

        sw.Stop();
        result.TraversalTimeMs = sw.ElapsedMilliseconds;
        result.IsComplete = true;

        return result;
    }

    private void TraverseNode(OctreeNode node, TraversalDelegate del, TraversalResult result)
    {
        result.NodesVisited++;

        if (!_nodeInfoCache.TryGetValue(node.Id, out var nodeInfo))
        {
            nodeInfo = new NodeInfo(node);
            _nodeInfoCache[node.Id] = nodeInfo;
        }

        // Update status
        nodeInfo.UpdateStatus(
            _cache?.Contains(node.Id) ?? false,
            _gpuBuffers?.Contains(node.Id) ?? false,
            -1 // Sector index not tracked here
        );

        var decision = del(nodeInfo);

        if (decision.IsAccepted)
        {
            result.NodesAccepted++;
            result.CachingNodes.Add(nodeInfo);

            if (decision.IsForDisplay)
            {
                result.ViewingNodes.Add(nodeInfo);
            }
        }

        if (decision.ContinueToChildren)
        {
            foreach (var child in node.Children)
            {
                TraverseNode(child, del, result);
            }
        }
    }

    /// <summary>
    /// Loads nodes to RAM cache and uploads to GPU.
    /// Call this after Traverse() with the viewing nodes.
    /// </summary>
    /// <param name="commandList">Stride CommandList for GPU uploads.</param>
    /// <param name="traversalResult">Result from Traverse().</param>
    /// <returns>Combined cache and upload result.</returns>
    public CacheAndUploadResult CacheAndUpload(CommandList commandList, TraversalResult traversalResult)
    {
        EnsureInitialized();

        var sw = Stopwatch.StartNew();
        var result = new CacheAndUploadResult();

        // 1. Load viewing nodes to cache (if not already cached)
        var notInCache = traversalResult.ViewingNodes
            .Where(n => !_cache!.Contains(n.Id))
            .ToList();

        if (notInCache.Count > 0)
        {
            foreach (var nodeInfo in notInCache)
            {
                var indices = nodeInfo.Node.PointIndices?.ToArray();
                if (indices != null && indices.Length > 0)
                {
                    var pointData = ReadPointData(indices);
                    _cache!.Add(nodeInfo.Id, indices, pointData);
                    result.NodesCached++;
                    result.PointsCached += indices.Length;
                }
            }
        }

        result.CacheVersion = _cache!.Version;
        result.TotalNodesCached = _cache!.EntryCount;
        result.TotalPointsCached = _cache!.TotalPointsCached;

        // 2. Upload to GPU
        result.BufferResult = _gpuBuffers!.Upload(
            commandList,
            traversalResult.ViewingNodes,
            nodeId => _cache.GetPointData(nodeId)
        );

        sw.Stop();
        result.TotalTimeMs = sw.ElapsedMilliseconds;

        return result;
    }

    /// <summary>
    /// Loads nodes to RAM cache and uploads to GPU asynchronously.
    /// The cache loading happens async, GPU upload happens on the calling thread.
    /// </summary>
    public async Task<CacheAndUploadResult> CacheAndUploadAsync(
        CommandList commandList,
        TraversalResult traversalResult,
        CancellationToken cancellationToken = default)
    {
        EnsureInitialized();

        var sw = Stopwatch.StartNew();
        var result = new CacheAndUploadResult();

        // 1. Load viewing nodes to cache asynchronously
        var notInCache = traversalResult.ViewingNodes
            .Where(n => !_cache!.Contains(n.Id))
            .ToList();

        if (notInCache.Count > 0)
        {
            await Task.Run(() =>
            {
                foreach (var nodeInfo in notInCache)
                {
                    if (cancellationToken.IsCancellationRequested) break;

                    var indices = nodeInfo.Node.PointIndices?.ToArray();
                    if (indices != null && indices.Length > 0)
                    {
                        var pointData = ReadPointData(indices);
                        _cache!.Add(nodeInfo.Id, indices, pointData);
                        result.NodesCached++;
                        result.PointsCached += indices.Length;
                    }
                }
            }, cancellationToken);
        }

        result.CacheVersion = _cache!.Version;
        result.TotalNodesCached = _cache!.EntryCount;
        result.TotalPointsCached = _cache!.TotalPointsCached;

        // 2. Upload to GPU (must be on main thread)
        result.BufferResult = _gpuBuffers!.Upload(
            commandList,
            traversalResult.ViewingNodes,
            nodeId => _cache.GetPointData(nodeId)
        );

        sw.Stop();
        result.TotalTimeMs = sw.ElapsedMilliseconds;

        return result;
    }

    private PointData[] ReadPointData(int[] indices)
    {
        if (_plyIndex == null) return Array.Empty<PointData>();

        var result = new PointData[indices.Length];

        // For binary PLY, use random access
        if (_plyIndex.Format != PlyFormat.Ascii)
        {
            for (int i = 0; i < indices.Length; i++)
            {
                var values = _plyIndex.ReadVertex(indices[i]);
                result[i] = ConvertToPointData(values);
            }
        }
        else
        {
            // ASCII - stream through (less efficient)
            var sortedIndices = indices.Select((idx, i) => (idx, i)).OrderBy(x => x.idx).ToArray();
            int currentIndex = 0;

            _plyIndex.StreamVertices((vertexIndex, pos, values) =>
            {
                while (currentIndex < sortedIndices.Length && sortedIndices[currentIndex].idx == vertexIndex)
                {
                    result[sortedIndices[currentIndex].i] = ConvertToPointData(values);
                    currentIndex++;
                }
            });
        }

        return result;
    }

    private PointData ConvertToPointData(float[] values)
    {
        if (_plyIndex == null) return new PointData();

        var point = new PointData();
        var props = _plyIndex.Properties;

        for (int i = 0; i < props.Count && i < values.Length; i++)
        {
            var name = props[i].Name.ToLower();
            var val = values[i];

            switch (name)
            {
                case "x": point.Position.X = val; break;
                case "y": point.Position.Y = val; break;
                case "z": point.Position.Z = val; break;
                case "red" or "r":
                    point.Color = new Stride.Core.Mathematics.Color4(
                        val > 1 ? val / 255f : val, point.Color.G, point.Color.B, point.Color.A);
                    break;
                case "green" or "g":
                    point.Color = new Stride.Core.Mathematics.Color4(
                        point.Color.R, val > 1 ? val / 255f : val, point.Color.B, point.Color.A);
                    break;
                case "blue" or "b":
                    point.Color = new Stride.Core.Mathematics.Color4(
                        point.Color.R, point.Color.G, val > 1 ? val / 255f : val, point.Color.A);
                    break;
                case "alpha" or "a":
                    point.Color = new Stride.Core.Mathematics.Color4(
                        point.Color.R, point.Color.G, point.Color.B, val > 1 ? val / 255f : val);
                    break;
                case "nx": point.Normal.X = val; break;
                case "ny": point.Normal.Y = val; break;
                case "nz": point.Normal.Z = val; break;
                case "intensity" or "scalar_intensity":
                    point.Intensity = val > 1 ? val / 65535f : val;
                    break;
                default:
                    // Store as scalar
                    point.SetScalar(name, val);
                    break;
            }
        }

        return point;
    }

    private void EnsureInitialized()
    {
        if (!_isInitialized)
            throw new InvalidOperationException("Loader not initialized. Call Initialize() first.");
    }

    public void Dispose()
    {
        if (_isDisposed) return;
        _isDisposed = true;

        _gpuBuffers?.Dispose();
        _cache?.Dispose();
        _plyIndex?.Dispose();
    }
}

/// <summary>
/// Combined result of cache and GPU upload operations.
/// </summary>
public class CacheAndUploadResult
{
    /// <summary>
    /// Total time in milliseconds.
    /// </summary>
    public long TotalTimeMs { get; set; }

    /// <summary>
    /// Number of nodes loaded to cache THIS FRAME (newly cached).
    /// </summary>
    public int NodesCached { get; set; }

    /// <summary>
    /// Number of points loaded to cache THIS FRAME (newly cached).
    /// </summary>
    public int PointsCached { get; set; }

    /// <summary>
    /// TOTAL nodes currently in cache (cumulative).
    /// </summary>
    public int TotalNodesCached { get; set; }

    /// <summary>
    /// TOTAL points currently in cache (cumulative).
    /// </summary>
    public int TotalPointsCached { get; set; }

    /// <summary>
    /// Cache version after loading.
    /// </summary>
    public int CacheVersion { get; set; }

    /// <summary>
    /// GPU buffer upload result.
    /// </summary>
    public BufferUploadResult BufferResult { get; set; } = new();

    /// <summary>
    /// Quick access to active sectors for rendering.
    /// </summary>
    public PointBufferSector[] ActiveSectors => BufferResult.ActiveSectors;

    /// <summary>
    /// Total points on GPU.
    /// </summary>
    public int TotalPointsOnGpu => BufferResult.TotalPointsInBuffer;

    /// <summary>
    /// Number of sectors uploaded this frame.
    /// </summary>
    public int SectorsUploaded => BufferResult.SectorsUploaded;
}

