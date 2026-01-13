using OctreeFlow.Core;
using OctreeFlow.Data;
using OctreeFlow.IO;
using Stride.Core.Mathematics;
using System.Diagnostics;

namespace OctreeFlow.Api;

/// <summary>
/// Main API for reading and traversing octree point cloud data.
/// Handles loading from .octree and .ply files, traversal, RAM caching, and buffer data output.
/// Designed for use with VVVV Gamma - outputs data ready for DynamicBufferAdvanced.
/// 
/// Workflow:
/// 1. Create reader with paths and settings
/// 2. Call Initialize() once
/// 3. Each frame: Call UpdateFrame() with your traversal delegate
/// 4. Use the BufferUpdateResult to upload new sectors to your DynamicBufferAdvanced buffers
/// 5. Use ActiveSectors for rendering dispatch
/// </summary>
public class OctreeFlowReader : IDisposable
{
    private readonly string _octreePath;
    private readonly string _plyPath;
    private readonly int _cacheSizeMB;
    private readonly int _bufferSizeMB;
    private readonly int _maxPointsPerSector;

    private CacheManager? _cache;
    private SectorManager? _sectorManager;
    private OctreeNode? _root;
    private OctreeFileInfo? _fileInfo;
    private PlyIndex? _plyIndex;
    private readonly Dictionary<string, NodeInfo> _nodeInfoCache = new();

    private int _traversalVersion;
    private bool _isInitialized;

    #region Public Properties

    /// <summary>
    /// Root node of the octree.
    /// </summary>
    public OctreeNode? Root => _root;

    /// <summary>
    /// File information from the octree file.
    /// </summary>
    public OctreeFileInfo? FileInfo => _fileInfo;

    /// <summary>
    /// The RAM cache manager.
    /// </summary>
    public CacheManager? Cache => _cache;

    /// <summary>
    /// The sector manager for buffer data.
    /// </summary>
    public SectorManager? SectorManager => _sectorManager;

    /// <summary>
    /// Buffer configuration.
    /// </summary>
    public BufferConfiguration? BufferConfig => _sectorManager?.Configuration;

    /// <summary>
    /// Whether the reader has been initialized.
    /// </summary>
    public bool IsInitialized => _isInitialized;

    /// <summary>
    /// Bounding box of the entire point cloud.
    /// </summary>
    public BoundingBox Bounds => _fileInfo?.Bounds ?? new BoundingBox();

    /// <summary>
    /// Total number of points in the point cloud.
    /// </summary>
    public int TotalPoints => _fileInfo?.TotalPoints ?? 0;

    /// <summary>
    /// Total number of nodes in the octree.
    /// </summary>
    public int TotalNodes => _fileInfo?.NodeCount ?? 0;

    /// <summary>
    /// Points per sector.
    /// </summary>
    public int MaxPointsPerSector => _maxPointsPerSector;

    /// <summary>
    /// Number of sectors in buffer.
    /// </summary>
    public int SectorCount => _sectorManager?.SectorCount ?? 0;

    /// <summary>
    /// Total buffer capacity in points.
    /// </summary>
    public int BufferCapacity => BufferConfig?.TotalCapacity ?? 0;

    /// <summary>
    /// Available properties from the PLY file.
    /// </summary>
    public IReadOnlyList<PlyProperty> PlyProperties => _plyIndex?.Properties ?? Array.Empty<PlyProperty>().ToList();

    #endregion

    /// <summary>
    /// Creates a new OctreeFlowReader.
    /// </summary>
    /// <param name="octreePath">Path to the .octree file.</param>
    /// <param name="plyPath">Path to the .ply file.</param>
    /// <param name="cacheSizeMB">RAM cache size in megabytes.</param>
    /// <param name="bufferSizeMB">Buffer size in megabytes (for sector calculation).</param>
    /// <param name="maxPointsPerSector">Maximum points per sector.</param>
    public OctreeFlowReader(
        string octreePath,
        string plyPath,
        int cacheSizeMB = 512,
        int bufferSizeMB = 256,
        int maxPointsPerSector = 65536)
    {
        _octreePath = octreePath;
        _plyPath = plyPath;
        _cacheSizeMB = cacheSizeMB;
        _bufferSizeMB = bufferSizeMB;
        _maxPointsPerSector = maxPointsPerSector;
    }

    /// <summary>
    /// Creates and initializes a new OctreeFlowReader.
    /// </summary>
    public static OctreeFlowReader Create(
        string octreePath,
        string plyPath,
        int cacheSizeMB = 512,
        int bufferSizeMB = 256,
        int maxPointsPerSector = 65536)
    {
        var reader = new OctreeFlowReader(octreePath, plyPath, cacheSizeMB, bufferSizeMB, maxPointsPerSector);
        reader.Initialize();
        return reader;
    }

    /// <summary>
    /// Initializes the reader by loading the octree structure and building the PLY index.
    /// </summary>
    public void Initialize()
    {
        if (_isInitialized) return;

        // Load octree structure
        var serializer = new StreamingOctreeSerializer();
        var (root, info) = serializer.LoadOctreeFile(_octreePath);

        _root = root ?? throw new InvalidOperationException("Failed to load octree file");
        _fileInfo = info;

        // Build PLY index (lightweight - just parse header)
        _plyIndex = new PlyIndex(_plyPath);
        _plyIndex.BuildIndexHeaderOnly();

        // Build node info cache
        BuildNodeInfoCache(_root);

        // Create RAM cache
        _cache = new CacheManager(_cacheSizeMB);

        // Create sector manager
        var config = BufferConfiguration.FromBufferSize(_bufferSizeMB, _maxPointsPerSector);
        _sectorManager = new SectorManager(_cache, config);

        _isInitialized = true;
    }

    /// <summary>
    /// Initializes the reader asynchronously.
    /// </summary>
    public async Task InitializeAsync(Action<string, int, int>? onProgress = null)
    {
        if (_isInitialized) return;

        await Task.Run(() =>
        {
            onProgress?.Invoke("Loading octree...", 0, 100);
            Initialize();
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
    /// Gets the NodeInfo for a node by ID.
    /// </summary>
    public NodeInfo? GetNodeInfo(string nodeId)
    {
        return _nodeInfoCache.TryGetValue(nodeId, out var info) ? info : null;
    }

    /// <summary>
    /// Traverses the octree using the provided delegate.
    /// Does NOT load any data - just determines what should be loaded.
    /// </summary>
    /// <param name="traversalDelegate">Delegate that decides how to handle each node.</param>
    /// <returns>Result containing caching and viewing node lists.</returns>
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

        // Get or create NodeInfo
        if (!_nodeInfoCache.TryGetValue(node.Id, out var nodeInfo))
        {
            nodeInfo = new NodeInfo(node);
            _nodeInfoCache[node.Id] = nodeInfo;
        }

        // Update status from cache and sector manager
        nodeInfo.UpdateStatus(
            _cache?.Contains(node.Id) ?? false,
            _sectorManager?.Contains(node.Id) ?? false,
            _sectorManager?.GetSectorFor(node.Id) ?? -1
        );

        // Call delegate
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

        // Continue to children if requested
        if (decision.ContinueToChildren)
        {
            foreach (var child in node.Children)
            {
                TraverseNode(child, del, result);
            }
        }
    }

    /// <summary>
    /// Loads specified nodes into RAM cache.
    /// </summary>
    /// <param name="nodes">Nodes to load into cache.</param>
    /// <returns>Result of the cache loading operation.</returns>
    public CacheLoadResult LoadToCache(IEnumerable<NodeInfo> nodes)
    {
        EnsureInitialized();

        var sw = Stopwatch.StartNew();
        var result = new CacheLoadResult();
        var nodesToLoad = nodes.Where(n => !_cache!.Contains(n.Id)).ToList();

        try
        {
            foreach (var nodeInfo in nodesToLoad)
            {
                var indices = nodeInfo.Node.PointIndices?.ToArray();
                if (indices != null && indices.Length > 0)
                {
                    var pointData = ReadPointData(indices);
                    _cache!.Add(nodeInfo.Id, indices, pointData);
                    result.LoadedNodes[nodeInfo.Id] = indices;
                }
            }

            result.Version = _cache!.Version;
            result.IsComplete = true;
        }
        catch (Exception ex)
        {
            result.Error = ex.Message;
            result.IsComplete = false;
        }

        sw.Stop();
        result.LoadTimeMs = sw.ElapsedMilliseconds;
        return result;
    }

    /// <summary>
    /// Loads specified nodes into RAM cache asynchronously.
    /// </summary>
    public async Task<CacheLoadResult> LoadToCacheAsync(
        IEnumerable<NodeInfo> nodes,
        CancellationToken cancellationToken = default)
    {
        EnsureInitialized();

        var sw = Stopwatch.StartNew();
        var result = new CacheLoadResult();
        var nodesToLoad = nodes.Where(n => !_cache!.Contains(n.Id)).ToList();

        try
        {
            await Task.Run(() =>
            {
                foreach (var nodeInfo in nodesToLoad)
                {
                    if (cancellationToken.IsCancellationRequested) break;

                    var indices = nodeInfo.Node.PointIndices?.ToArray();
                    if (indices != null && indices.Length > 0)
                    {
                        var pointData = ReadPointData(indices);
                        _cache!.Add(nodeInfo.Id, indices, pointData);
                        result.LoadedNodes[nodeInfo.Id] = indices;
                    }
                }
            }, cancellationToken);

            result.Version = _cache!.Version;
            result.IsComplete = !cancellationToken.IsCancellationRequested;
        }
        catch (OperationCanceledException)
        {
            result.IsComplete = false;
        }
        catch (Exception ex)
        {
            result.Error = ex.Message;
            result.IsComplete = false;
        }

        sw.Stop();
        result.LoadTimeMs = sw.ElapsedMilliseconds;
        return result;
    }

    /// <summary>
    /// Performs a complete frame update: Traverse → Cache → Buffer data output.
    /// This is the main method to call each frame.
    /// </summary>
    /// <param name="traversalDelegate">Your traversal logic.</param>
    /// <returns>Complete frame result with buffer data to upload.</returns>
    public FrameUpdateResult UpdateFrame(TraversalDelegate traversalDelegate)
    {
        EnsureInitialized();

        var sw = Stopwatch.StartNew();
        var result = new FrameUpdateResult();

        // 1. Traverse
        result.Traversal = Traverse(traversalDelegate);

        // 2. Load viewing nodes to cache (sync)
        var notInCache = result.Traversal.ViewingNodes
            .Where(n => !_cache!.Contains(n.Id))
            .ToList();

        if (notInCache.Count > 0)
        {
            result.CacheResult = LoadToCache(notInCache);
        }
        else
        {
            result.CacheResult = new CacheLoadResult { IsComplete = true, Version = _cache!.Version };
        }

        // 3. Update sector manager - get buffer data to upload
        result.BufferUpdate = _sectorManager!.Update(result.Traversal.ViewingNodes);

        sw.Stop();
        result.TotalTimeMs = sw.ElapsedMilliseconds;

        return result;
    }

    /// <summary>
    /// Async version of UpdateFrame.
    /// Cache loading happens async, sector update happens synchronously.
    /// </summary>
    public async Task<FrameUpdateResult> UpdateFrameAsync(
        TraversalDelegate traversalDelegate,
        CancellationToken cancellationToken = default)
    {
        EnsureInitialized();

        var sw = Stopwatch.StartNew();
        var result = new FrameUpdateResult();

        // 1. Traverse
        result.Traversal = Traverse(traversalDelegate);

        // 2. Load viewing nodes to cache (async)
        var notInCache = result.Traversal.ViewingNodes
            .Where(n => !_cache!.Contains(n.Id))
            .ToList();

        if (notInCache.Count > 0)
        {
            result.CacheResult = await LoadToCacheAsync(notInCache, cancellationToken);
        }
        else
        {
            result.CacheResult = new CacheLoadResult { IsComplete = true, Version = _cache!.Version };
        }

        // 3. Update sector manager - get buffer data to upload
        result.BufferUpdate = _sectorManager!.Update(result.Traversal.ViewingNodes);

        sw.Stop();
        result.TotalTimeMs = sw.ElapsedMilliseconds;

        return result;
    }

    /// <summary>
    /// Reads point data for the given indices from the PLY file.
    /// </summary>
    private PointData[] ReadPointData(int[] indices)
    {
        if (_plyIndex == null) return Array.Empty<PointData>();

        var result = new PointData[indices.Length];

        // For binary PLY, we can do random access
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
            // For ASCII, we need to stream (less efficient for random access)
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
                    point.Color = new Color4(val > 1 ? val / 255f : val, point.Color.G, point.Color.B, point.Color.A);
                    break;
                case "green" or "g":
                    point.Color = new Color4(point.Color.R, val > 1 ? val / 255f : val, point.Color.B, point.Color.A);
                    break;
                case "blue" or "b":
                    point.Color = new Color4(point.Color.R, point.Color.G, val > 1 ? val / 255f : val, point.Color.A);
                    break;
                case "alpha" or "a":
                    point.Color = new Color4(point.Color.R, point.Color.G, point.Color.B, val > 1 ? val / 255f : val);
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
            throw new InvalidOperationException("Reader not initialized. Call Initialize() first.");
    }

    public void Dispose()
    {
        _cache?.Dispose();
        _sectorManager?.Dispose();
        _plyIndex?.Dispose();
    }
}

/// <summary>
/// Complete result of a frame update.
/// Contains traversal info, cache status, and buffer data to upload.
/// </summary>
public class FrameUpdateResult
{
    /// <summary>
    /// Traversal result.
    /// </summary>
    public TraversalResult Traversal { get; set; } = new();

    /// <summary>
    /// Cache loading result.
    /// </summary>
    public CacheLoadResult CacheResult { get; set; } = new();

    /// <summary>
    /// Buffer update result with data to upload.
    /// </summary>
    public BufferUpdateResult BufferUpdate { get; set; } = new();

    /// <summary>
    /// Total time for the entire update.
    /// </summary>
    public long TotalTimeMs { get; set; }

    /// <summary>
    /// Quick access to new sector data to upload.
    /// Upload these to your DynamicBufferAdvanced buffers.
    /// </summary>
    public SectorData[] NewSectors => BufferUpdate.NewSectors;

    /// <summary>
    /// Quick access to active sectors for rendering.
    /// Use this for your rendering dispatch.
    /// </summary>
    public SectorInfo[] ActiveSectors => BufferUpdate.ActiveSectors;

    /// <summary>
    /// Total points ready for rendering.
    /// </summary>
    public int TotalPointsInBuffer => BufferUpdate.TotalPointsInBuffer;

    /// <summary>
    /// Whether there's new data to upload this frame.
    /// </summary>
    public bool HasNewData => BufferUpdate.HasNewData;
}
