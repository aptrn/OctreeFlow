using OctreeFlow.Core;
using OctreeFlow.Data;
using OctreeFlow.IO;
using Stride.Core.Mathematics;
using System.Collections.Concurrent;
using System.Diagnostics;

namespace OctreeFlow.Api;

/// <summary>
/// Main API for reading and traversing octree point cloud data.
/// Handles loading from .octree and .ply files, traversal, RAM caching, and GPU loading.
/// Designed for use with VVVV Gamma.
/// </summary>
public class OctreeFlowReader : IDisposable
{
    private readonly string _octreePath;
    private readonly string _plyPath;
    private readonly CacheManager _cache;
    private readonly GpuSectorManager _gpuManager;
    private readonly GpuLoader _gpuLoader;
    private readonly PlyIndex _plyIndex;
    private readonly Dictionary<string, NodeInfo> _nodeInfoCache = new();
    private readonly int _maxPointsPerNode;
    
    private OctreeNode? _root;
    private OctreeFileInfo? _fileInfo;
    private int _traversalVersion;
    private bool _isInitialized;

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
    public CacheManager Cache => _cache;

    /// <summary>
    /// The GPU sector manager (manual management).
    /// </summary>
    public GpuSectorManager GpuManager => _gpuManager;

    /// <summary>
    /// The automatic GPU loader (recommended - handles everything).
    /// </summary>
    public GpuLoader GpuLoader => _gpuLoader;

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
    /// Creates a new OctreeFlowReader.
    /// </summary>
    /// <param name="octreePath">Path to the .octree file.</param>
    /// <param name="plyPath">Path to the .ply file.</param>
    /// <param name="cacheSizeMB">RAM cache size in megabytes.</param>
    /// <param name="gpuBufferSizeMB">Maximum GPU buffer size in megabytes.</param>
    /// <param name="maxPointsPerNode">Maximum points per octree node (determines GPU sector size).</param>
    public OctreeFlowReader(
        string octreePath,
        string plyPath,
        int cacheSizeMB = 512,
        int gpuBufferSizeMB = 256,
        int maxPointsPerNode = 65536)
    {
        _octreePath = octreePath;
        _plyPath = plyPath;
        _maxPointsPerNode = maxPointsPerNode;
        _cache = new CacheManager(cacheSizeMB);
        _gpuManager = new GpuSectorManager(gpuBufferSizeMB, maxPointsPerNode);
        _gpuLoader = new GpuLoader(_cache, gpuBufferSizeMB, maxPointsPerNode);
        _plyIndex = new PlyIndex(plyPath);
    }

    /// <summary>
    /// Creates and initializes a new OctreeFlowReader.
    /// </summary>
    public static OctreeFlowReader Create(
        string octreePath,
        string plyPath,
        int cacheSizeMB = 512,
        int gpuBufferSizeMB = 256,
        int maxPointsPerNode = 65536)
    {
        var reader = new OctreeFlowReader(octreePath, plyPath, cacheSizeMB, gpuBufferSizeMB, maxPointsPerNode);
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

        // Build PLY index (lightweight - just parse header, skip bounds computation)
        _plyIndex.BuildIndexHeaderOnly();

        // Build node info cache
        BuildNodeInfoCache(_root);

        _isInitialized = true;
    }

    /// <summary>
    /// Initializes the reader asynchronously.
    /// </summary>
    public async Task InitializeAsync(Action<int, int>? onProgress = null)
    {
        if (_isInitialized) return;

        await Task.Run(() =>
        {
            // Load octree structure
            var serializer = new StreamingOctreeSerializer();
            var (root, info) = serializer.LoadOctreeFile(_octreePath);
            
            _root = root ?? throw new InvalidOperationException("Failed to load octree file");
            _fileInfo = info;

            // Build PLY index (lightweight - just parse header, skip bounds computation)
            // We use the bounds from the octree file instead
            _plyIndex.BuildIndexHeaderOnly();

            // Build node info cache
            BuildNodeInfoCache(_root);

            _isInitialized = true;
            
            onProgress?.Invoke(1, 1);
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
    /// The delegate is called for each node starting from the root.
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

        // Update status from cache and GPU
        nodeInfo.UpdateStatus(
            _cache.Contains(node.Id),
            _gpuManager.Contains(node.Id),
            _gpuManager.GetSectorForNode(node.Id)
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
    /// Loads specified nodes into RAM cache asynchronously.
    /// </summary>
    /// <param name="nodes">Nodes to load into cache.</param>
    /// <param name="cancellationToken">Cancellation token.</param>
    /// <returns>Result of the cache loading operation.</returns>
    public async Task<CacheLoadResult> LoadToCacheAsync(
        IEnumerable<NodeInfo> nodes,
        CancellationToken cancellationToken = default)
    {
        EnsureInitialized();

        var sw = Stopwatch.StartNew();
        var result = new CacheLoadResult();
        var nodesToLoad = nodes.Where(n => !_cache.Contains(n.Id)).ToList();

        try
        {
            await Task.Run(() =>
            {
                foreach (var nodeInfo in nodesToLoad)
                {
                    if (cancellationToken.IsCancellationRequested) break;

                    // Get point indices from the octree node
                    var indices = nodeInfo.Node.PointIndices?.ToArray();
                    if (indices != null && indices.Length > 0)
                    {
                        // Read point data from PLY file
                        var pointData = ReadPointData(indices);
                        
                        _cache.Add(nodeInfo.Id, indices, pointData);
                        result.LoadedNodes[nodeInfo.Id] = indices;
                    }
                }
            }, cancellationToken);

            result.Version = _cache.Version;
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
    /// Loads specified nodes into RAM cache synchronously.
    /// </summary>
    public CacheLoadResult LoadToCache(IEnumerable<NodeInfo> nodes)
    {
        EnsureInitialized();

        var sw = Stopwatch.StartNew();
        var result = new CacheLoadResult();
        var nodesToLoad = nodes.Where(n => !_cache.Contains(n.Id)).ToList();

        try
        {
            foreach (var nodeInfo in nodesToLoad)
            {
                var indices = nodeInfo.Node.PointIndices?.ToArray();
                if (indices != null && indices.Length > 0)
                {
                    var pointData = ReadPointData(indices);
                    _cache.Add(nodeInfo.Id, indices, pointData);
                    result.LoadedNodes[nodeInfo.Id] = indices;
                }
            }

            result.Version = _cache.Version;
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
    /// Loads specified nodes to GPU asynchronously.
    /// First ensures nodes are in RAM cache, then uploads to GPU sectors.
    /// </summary>
    /// <param name="nodes">Nodes to load to GPU.</param>
    /// <param name="onSectorData">Callback to upload data to actual GPU buffer (called per sector).</param>
    /// <param name="cancellationToken">Cancellation token.</param>
    /// <returns>Result of the GPU loading operation.</returns>
    public async Task<GpuLoadResult> LoadToGpuAsync(
        IEnumerable<NodeInfo> nodes,
        Action<int, string, PointData[]>? onSectorData = null,
        CancellationToken cancellationToken = default)
    {
        EnsureInitialized();

        var sw = Stopwatch.StartNew();
        var result = new GpuLoadResult();
        var nodeList = nodes.ToList();

        // First, ensure all nodes are in cache
        var notInCache = nodeList.Where(n => !_cache.Contains(n.Id)).ToList();
        if (notInCache.Count > 0)
        {
            await LoadToCacheAsync(notInCache, cancellationToken);
        }

        try
        {
            // Release sectors for nodes no longer needed
            var currentGpuNodes = _gpuManager.GetLoadedNodeIds().ToHashSet();
            var neededNodes = nodeList.Select(n => n.Id).ToHashSet();
            
            foreach (var nodeId in currentGpuNodes)
            {
                if (!neededNodes.Contains(nodeId))
                {
                    _gpuManager.ReleaseSector(nodeId);
                }
            }

            // Allocate sectors for new nodes
            int loaded = 0;
            int totalPoints = 0;

            foreach (var nodeInfo in nodeList)
            {
                if (cancellationToken.IsCancellationRequested) break;

                if (!_gpuManager.Contains(nodeInfo.Id))
                {
                    int sectorIndex = _gpuManager.AllocateSector(nodeInfo.Id, nodeInfo.PointCount);
                    if (sectorIndex >= 0)
                    {
                        // Get point data from cache
                        var pointData = _cache.GetPointData(nodeInfo.Id);
                        if (pointData != null && onSectorData != null)
                        {
                            onSectorData(sectorIndex, nodeInfo.Id, pointData);
                        }
                        loaded++;
                        totalPoints += nodeInfo.PointCount;
                    }
                }
                else
                {
                    loaded++;
                    totalPoints += nodeInfo.PointCount;
                }
            }

            result.Version = _gpuManager.Version;
            result.SectorActivations = _gpuManager.GetSectorActivations();
            result.NodesLoaded = loaded;
            result.TotalPointsLoaded = totalPoints;
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
    /// Loads specified nodes to GPU synchronously.
    /// </summary>
    public GpuLoadResult LoadToGpu(
        IEnumerable<NodeInfo> nodes,
        Action<int, string, PointData[]>? onSectorData = null)
    {
        EnsureInitialized();

        var sw = Stopwatch.StartNew();
        var result = new GpuLoadResult();
        var nodeList = nodes.ToList();

        // First, ensure all nodes are in cache
        var notInCache = nodeList.Where(n => !_cache.Contains(n.Id)).ToList();
        if (notInCache.Count > 0)
        {
            LoadToCache(notInCache);
        }

        try
        {
            // Release sectors for nodes no longer needed
            var currentGpuNodes = _gpuManager.GetLoadedNodeIds().ToHashSet();
            var neededNodes = nodeList.Select(n => n.Id).ToHashSet();
            
            foreach (var nodeId in currentGpuNodes)
            {
                if (!neededNodes.Contains(nodeId))
                {
                    _gpuManager.ReleaseSector(nodeId);
                }
            }

            // Allocate sectors for new nodes
            int loaded = 0;
            int totalPoints = 0;

            foreach (var nodeInfo in nodeList)
            {
                if (!_gpuManager.Contains(nodeInfo.Id))
                {
                    int sectorIndex = _gpuManager.AllocateSector(nodeInfo.Id, nodeInfo.PointCount);
                    if (sectorIndex >= 0)
                    {
                        var pointData = _cache.GetPointData(nodeInfo.Id);
                        if (pointData != null && onSectorData != null)
                        {
                            onSectorData(sectorIndex, nodeInfo.Id, pointData);
                        }
                        loaded++;
                        totalPoints += nodeInfo.PointCount;
                    }
                }
                else
                {
                    loaded++;
                    totalPoints += nodeInfo.PointCount;
                }
            }

            result.Version = _gpuManager.Version;
            result.SectorActivations = _gpuManager.GetSectorActivations();
            result.NodesLoaded = loaded;
            result.TotalPointsLoaded = totalPoints;
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
    /// Performs a complete traversal and loading cycle.
    /// Traverses the tree, loads caching nodes to RAM, and viewing nodes to GPU.
    /// </summary>
    public async Task<(TraversalResult traversal, CacheLoadResult cache, GpuLoadResult gpu)> TraverseAndLoadAsync(
        TraversalDelegate traversalDelegate,
        Action<int, string, PointData[]>? onSectorData = null,
        CancellationToken cancellationToken = default)
    {
        // Traverse
        var traversalResult = Traverse(traversalDelegate);

        // Load to cache
        var cacheResult = await LoadToCacheAsync(traversalResult.CachingNodes, cancellationToken);

        // Load to GPU
        var gpuResult = await LoadToGpuAsync(traversalResult.ViewingNodes, onSectorData, cancellationToken);

        return (traversalResult, cacheResult, gpuResult);
    }

    /// <summary>
    /// Reads point data for the given indices from the PLY file.
    /// </summary>
    private PointData[] ReadPointData(int[] indices)
    {
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
            // In practice, indices should be sorted for better performance
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
        var point = new PointData();

        // Map values to PointData based on property names
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
            }
        }

        return point;
    }

    private void EnsureInitialized()
    {
        if (!_isInitialized)
            throw new InvalidOperationException("Reader not initialized. Call Initialize() first.");
    }

    /// <summary>
    /// Simple one-call update: Traverse → Cache → GPU.
    /// Just pass your traversal logic and get back everything you need for rendering.
    /// </summary>
    /// <param name="traversalDelegate">Your traversal logic.</param>
    /// <returns>Complete frame result with GPU uploads ready.</returns>
    public FrameUpdateResult UpdateFrame(TraversalDelegate traversalDelegate)
    {
        EnsureInitialized();

        var sw = System.Diagnostics.Stopwatch.StartNew();
        var result = new FrameUpdateResult();

        // 1. Traverse
        result.Traversal = Traverse(traversalDelegate);

        // 2. Load viewing nodes to cache (sync - for simplicity)
        var notInCache = result.Traversal.ViewingNodes
            .Where(n => !_cache.Contains(n.Id))
            .ToList();
        
        if (notInCache.Count > 0)
        {
            result.CacheResult = LoadToCache(notInCache);
        }
        else
        {
            result.CacheResult = new CacheLoadResult { IsComplete = true, Version = _cache.Version };
        }

        // 3. Update GPU (auto-managed)
        result.GpuUpdate = _gpuLoader.Update(result.Traversal.ViewingNodes);

        sw.Stop();
        result.TotalTimeMs = sw.ElapsedMilliseconds;

        return result;
    }

    /// <summary>
    /// Async version of UpdateFrame.
    /// </summary>
    public async Task<FrameUpdateResult> UpdateFrameAsync(
        TraversalDelegate traversalDelegate,
        CancellationToken cancellationToken = default)
    {
        EnsureInitialized();

        var sw = System.Diagnostics.Stopwatch.StartNew();
        var result = new FrameUpdateResult();

        // 1. Traverse
        result.Traversal = Traverse(traversalDelegate);

        // 2. Load viewing nodes to cache (async)
        var notInCache = result.Traversal.ViewingNodes
            .Where(n => !_cache.Contains(n.Id))
            .ToList();
        
        if (notInCache.Count > 0)
        {
            result.CacheResult = await LoadToCacheAsync(notInCache, cancellationToken);
        }
        else
        {
            result.CacheResult = new CacheLoadResult { IsComplete = true, Version = _cache.Version };
        }

        // 3. Update GPU (auto-managed)
        result.GpuUpdate = _gpuLoader.Update(result.Traversal.ViewingNodes);

        sw.Stop();
        result.TotalTimeMs = sw.ElapsedMilliseconds;

        return result;
    }

    public void Dispose()
    {
        _cache.Dispose();
        _gpuManager.Dispose();
        _gpuLoader.Dispose();
        _plyIndex.Dispose();
    }
}

/// <summary>
/// Complete result of a frame update.
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
    /// GPU update result with uploads ready.
    /// </summary>
    public GpuUpdateResult GpuUpdate { get; set; } = new();

    /// <summary>
    /// Total time for the entire update.
    /// </summary>
    public long TotalTimeMs { get; set; }

    /// <summary>
    /// Quick access to pending GPU uploads.
    /// </summary>
    public SectorUpload[] Uploads => GpuUpdate.Uploads;

    /// <summary>
    /// Quick access to active sectors for rendering.
    /// </summary>
    public SectorState[] ActiveSectors => GpuUpdate.ActiveSectors;

    /// <summary>
    /// Total points ready for rendering.
    /// </summary>
    public int TotalPointsOnGpu => GpuUpdate.TotalPointsOnGpu;
}

