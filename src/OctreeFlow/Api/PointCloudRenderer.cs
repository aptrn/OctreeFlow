using Stride.Graphics;
using OctreeFlow.Core;
using OctreeFlow.IO;
using System.Diagnostics;

namespace OctreeFlow.Api;

/// <summary>
/// Complete point cloud renderer with automatic GPU management.
/// Combines octree traversal, RAM caching, and GPU buffer management into one simple API.
/// 
/// Usage in VVVV Gamma:
/// 1. Create once with GraphicsDevice
/// 2. Call UpdateFrame() each frame with your traversal delegate
/// 3. Use the Buffer property in your shader
/// 4. Render using ActiveSectors to know which sectors have valid data
/// </summary>
public class PointCloudRenderer : IDisposable
{
    private readonly OctreeFlowReader _reader;
    private readonly PointCloudBuffer _gpuBuffer;
    private bool _isDisposed;

    /// <summary>
    /// The underlying octree reader.
    /// </summary>
    public OctreeFlowReader Reader => _reader;

    /// <summary>
    /// The GPU point cloud buffer.
    /// </summary>
    public PointCloudBuffer GpuBuffer => _gpuBuffer;

    /// <summary>
    /// The GPU buffer to bind to your shader.
    /// </summary>
    public Stride.Graphics.Buffer? Buffer => _gpuBuffer.Buffer;

    /// <summary>
    /// Number of points per sector (for rendering calculations).
    /// </summary>
    public int SectorSizePoints => _gpuBuffer.SectorSizePoints;

    /// <summary>
    /// Number of sectors in the buffer.
    /// </summary>
    public int SectorCount => _gpuBuffer.SectorCount;

    /// <summary>
    /// Total buffer capacity in points.
    /// </summary>
    public int TotalCapacity => _gpuBuffer.TotalCapacity;

    /// <summary>
    /// Whether the renderer is initialized.
    /// </summary>
    public bool IsInitialized => _reader.IsInitialized;

    /// <summary>
    /// Creates a point cloud renderer.
    /// </summary>
    /// <param name="graphicsDevice">Stride GraphicsDevice from VVVV.</param>
    /// <param name="octreePath">Path to the .octree file.</param>
    /// <param name="plyPath">Path to the .ply file.</param>
    /// <param name="cacheSizeMB">RAM cache size in MB.</param>
    /// <param name="gpuBufferSizeMB">GPU buffer size in MB.</param>
    /// <param name="maxPointsPerNode">Maximum points per octree node.</param>
    public PointCloudRenderer(
        GraphicsDevice graphicsDevice,
        string octreePath,
        string plyPath,
        int cacheSizeMB = 512,
        int gpuBufferSizeMB = 256,
        int maxPointsPerNode = 65536)
    {
        _reader = new OctreeFlowReader(octreePath, plyPath, cacheSizeMB, gpuBufferSizeMB, maxPointsPerNode);
        _gpuBuffer = new PointCloudBuffer(graphicsDevice, gpuBufferSizeMB, maxPointsPerNode);
    }

    /// <summary>
    /// Initializes the renderer (loads octree structure).
    /// Call this once before using UpdateFrame.
    /// </summary>
    public void Initialize()
    {
        _reader.Initialize();
    }

    /// <summary>
    /// Initializes the renderer asynchronously.
    /// </summary>
    public async Task InitializeAsync(Action<int, int>? onProgress = null)
    {
        await _reader.InitializeAsync(onProgress);
    }

    /// <summary>
    /// Creates and initializes a renderer in one call.
    /// </summary>
    public static PointCloudRenderer Create(
        GraphicsDevice graphicsDevice,
        string octreePath,
        string plyPath,
        int cacheSizeMB = 512,
        int gpuBufferSizeMB = 256,
        int maxPointsPerNode = 65536)
    {
        var renderer = new PointCloudRenderer(
            graphicsDevice, octreePath, plyPath,
            cacheSizeMB, gpuBufferSizeMB, maxPointsPerNode);
        renderer.Initialize();
        return renderer;
    }

    /// <summary>
    /// Updates the renderer for the current frame.
    /// Traverses the octree, loads data to cache, and uploads to GPU.
    /// 
    /// Call this every frame with your traversal logic.
    /// </summary>
    /// <param name="commandList">Stride CommandList for GPU uploads.</param>
    /// <param name="traversalDelegate">Your traversal logic.</param>
    /// <returns>Complete render result with active sectors.</returns>
    public RenderFrameResult UpdateFrame(CommandList commandList, TraversalDelegate traversalDelegate)
    {
        if (!_reader.IsInitialized)
            throw new InvalidOperationException("Renderer not initialized. Call Initialize() first.");

        var sw = Stopwatch.StartNew();
        var result = new RenderFrameResult();

        // 1. Traverse the octree
        result.Traversal = _reader.Traverse(traversalDelegate);

        // 2. Load viewing nodes to cache (if not already cached)
        var notInCache = result.Traversal.ViewingNodes
            .Where(n => !_reader.Cache.Contains(n.Id))
            .ToList();

        if (notInCache.Count > 0)
        {
            result.CacheResult = _reader.LoadToCache(notInCache);
        }
        else
        {
            result.CacheResult = new CacheLoadResult { IsComplete = true, Version = _reader.Cache.Version };
        }

        // 3. Update GPU buffer
        result.BufferUpdate = _gpuBuffer.Update(commandList, result.Traversal.ViewingNodes, _reader.Cache);

        sw.Stop();
        result.TotalTimeMs = sw.ElapsedMilliseconds;

        return result;
    }

    /// <summary>
    /// Gets information about active sectors for rendering.
    /// Use this to know which sectors to render and how many points each has.
    /// </summary>
    public ActiveSector[] GetActiveSectors()
    {
        // This returns a snapshot - call after UpdateFrame
        return Array.Empty<ActiveSector>(); // Use the result from UpdateFrame instead
    }

    public void Dispose()
    {
        if (_isDisposed) return;
        _isDisposed = true;
        
        _gpuBuffer.Dispose();
        _reader.Dispose();
    }
}

/// <summary>
/// Complete result from a render frame update.
/// </summary>
public class RenderFrameResult
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
    /// GPU buffer update result.
    /// </summary>
    public BufferUpdateResult BufferUpdate { get; set; } = new();

    /// <summary>
    /// Total time for the entire update in ms.
    /// </summary>
    public long TotalTimeMs { get; set; }

    /// <summary>
    /// Quick access to active sectors for rendering.
    /// Each sector has StartIndex and PointCount for DrawInstanced.
    /// </summary>
    public ActiveSector[] ActiveSectors => BufferUpdate.ActiveSectors;

    /// <summary>
    /// Total points currently in GPU buffer.
    /// </summary>
    public int TotalPointsOnGpu => BufferUpdate.TotalPointsInBuffer;

    /// <summary>
    /// Number of sectors uploaded this frame.
    /// </summary>
    public int SectorsUploaded => BufferUpdate.SectorsUploaded;
}

