using Stride.Core.Mathematics;
using OctreeFlow.IO;
using System.Buffers;
using System.Collections.Concurrent;
using System.IO.MemoryMappedFiles;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace OctreeFlow.Core;

/// <summary>
/// High-performance parallel octree builder using multi-threading and memory-mapped I/O.
/// Significantly faster than the sequential StreamingOctreeBuilder for large point clouds.
/// </summary>
public class ParallelStreamingOctreeBuilder : IDisposable
{
    private readonly OctreeConfiguration _config;
    private readonly string _plyPath;
    private readonly string _tempPath;
    private readonly int _threadCount;

    // Memory-mapped positions cache for fast parallel access
    private string? _positionsCachePath;
    private MemoryMappedFile? _positionsMmf;
    private MemoryMappedViewAccessor? _positionsAccessor;

    // Pre-loaded positions in memory for faster access (used for smaller point clouds)
    private Vector3[]? _positionsInMemory;
    private bool _useInMemoryPositions;

    // Memory limit for in-memory positions (512MB)
    private const long InMemoryPositionsLimit = 512 * 1024 * 1024;

    private PlyIndex? _plyIndex;
    private int _totalPoints;
    private long _processedPoints;
    private long _assignedPoints;

    // Thread-local random for parallel operations
    private readonly ThreadLocal<Random> _threadLocalRandom;

    // Minimum points to use parallel processing (below this, sequential is faster)
    private const int ParallelThreshold = 10000;

    // Batch size for parallel filtering
    private const int FilterBatchSize = 50000;

    /// <summary>
    /// Progress callback: (phase, current, total, depth)
    /// Phase: 0=indexing, 1=building, 2=writing
    /// </summary>
    public Action<int, int, int, int>? OnProgress { get; set; }

    /// <summary>
    /// Called when a node is completed.
    /// </summary>
    public Action<OctreeNode>? OnNodeCompleted { get; set; }

    public ParallelStreamingOctreeBuilder(OctreeConfiguration configuration, string plyPath, int threadCount = 0)
    {
        _config = configuration;
        _config.Validate();
        _plyPath = plyPath;
        _tempPath = Path.GetTempPath();

        // Use all available cores if not specified, leave 1 for the system
        _threadCount = threadCount > 0 ? threadCount : Math.Max(1, Environment.ProcessorCount - 1);

        // Thread-local random with different seeds
        _threadLocalRandom = new ThreadLocal<Random>(() =>
        {
            int seed = configuration.RandomSeed ?? Environment.TickCount;
            return new Random(seed ^ Environment.CurrentManagedThreadId ^ DateTime.UtcNow.Ticks.GetHashCode());
        });
    }

    /// <summary>
    /// Builds the octree using parallel processing, returning the root node.
    /// </summary>
    public OctreeNode Build()
    {
        _processedPoints = 0;
        _assignedPoints = 0;

        // Phase 0: Index PLY and create positions cache with parallel processing
        Console.WriteLine($"  Indexing PLY file (using {_threadCount} threads)...");
        IndexPlyFileParallel();

        if (_plyIndex == null || _totalPoints == 0)
            throw new InvalidOperationException("No points found in PLY file");

        // Create root with cubic bounds
        var bounds = MakeCubicBounds(_plyIndex.Bounds);
        var root = new OctreeNode("0_0_0_0", bounds, 0);

        // Phase 1: Build octree structure using parallel processing
        Console.WriteLine($"  Building octree structure ({_totalPoints:N0} points)...");

        // Create shuffled indices array for randomness
        var allIndices = CreateShuffledIndices(_totalPoints);

        // Fill the tree with parallel processing
        FillNodeParallel(root, allIndices, 0);

        return root;
    }

    private void IndexPlyFileParallel()
    {
        _plyIndex = new PlyIndex(_plyPath);

        // Create positions cache file path
        _positionsCachePath = Path.Combine(_tempPath, $"octreeflow_pos_{Guid.NewGuid():N}.bin");

        // Build index with positions cache - this part needs PLY streaming
        _plyIndex.BuildIndexWithPositionsCache(_positionsCachePath, (current, total) =>
        {
            OnProgress?.Invoke(0, current, total, 0);
        });

        _totalPoints = _plyIndex.VertexCount;

        // Determine whether to load positions into memory or use memory-mapped file
        long positionsSize = (long)_totalPoints * 12;
        _useInMemoryPositions = positionsSize <= InMemoryPositionsLimit;

        if (_useInMemoryPositions)
        {
            // Load all positions into memory for faster random access
            Console.WriteLine($"  Loading {_totalPoints:N0} positions into memory ({positionsSize / (1024 * 1024):N0} MB)...");
            _positionsInMemory = new Vector3[_totalPoints];
            
            using var fileStream = new FileStream(_positionsCachePath!, FileMode.Open, FileAccess.Read, FileShare.Read, 65536);
            using var reader = new BinaryReader(fileStream);
            
            for (int i = 0; i < _totalPoints; i++)
            {
                _positionsInMemory[i] = new Vector3(
                    reader.ReadSingle(),
                    reader.ReadSingle(),
                    reader.ReadSingle()
                );
            }
        }
        else
        {
            // Use memory-mapped file for very large point clouds
            Console.WriteLine($"  Using memory-mapped file for positions ({positionsSize / (1024 * 1024):N0} MB)...");
            _positionsMmf = MemoryMappedFile.CreateFromFile(_positionsCachePath!, FileMode.Open, null, 0, MemoryMappedFileAccess.Read);
            _positionsAccessor = _positionsMmf.CreateViewAccessor(0, positionsSize, MemoryMappedFileAccess.Read);
        }
    }

    /// <summary>
    /// Reads a position from the cache (thread-safe).
    /// Uses in-memory array for smaller clouds, memory-mapped file for larger ones.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private Vector3 ReadPosition(int index)
    {
        if (_useInMemoryPositions && _positionsInMemory != null)
        {
            return _positionsInMemory[index];
        }

        if (_positionsAccessor == null)
            throw new InvalidOperationException("Positions cache not initialized");

        long offset = (long)index * 12;
        float x = 0, y = 0, z = 0;
        _positionsAccessor.Read(offset, out x);
        _positionsAccessor.Read(offset + 4, out y);
        _positionsAccessor.Read(offset + 8, out z);
        return new Vector3(x, y, z);
    }

    /// <summary>
    /// Batch read positions for better cache performance.
    /// </summary>
    private void ReadPositionsBatch(int[] indices, Vector3[] positions, int start, int count)
    {
        if (_positionsAccessor == null)
            throw new InvalidOperationException("Positions cache not initialized");

        for (int i = 0; i < count; i++)
        {
            int idx = indices[start + i];
            long offset = (long)idx * 12;
            float x = 0, y = 0, z = 0;
            _positionsAccessor.Read(offset, out x);
            _positionsAccessor.Read(offset + 4, out y);
            _positionsAccessor.Read(offset + 8, out z);
            positions[i] = new Vector3(x, y, z);
        }
    }

    private BoundingBox MakeCubicBounds(BoundingBox bounds)
    {
        var size = bounds.Maximum - bounds.Minimum;
        var maxDim = Math.Max(Math.Max(size.X, size.Y), size.Z);
        var center = bounds.Center;
        var halfSize = maxDim / 2f * 1.01f;
        return new BoundingBox(
            center - new Vector3(halfSize),
            center + new Vector3(halfSize)
        );
    }

    private int[] CreateShuffledIndices(int count)
    {
        var indices = new int[count];
        for (int i = 0; i < count; i++)
            indices[i] = i;

        // Fisher-Yates shuffle with parallel chunks for large arrays
        var random = _threadLocalRandom.Value!;
        for (int i = count - 1; i > 0; i--)
        {
            int j = random.Next(i + 1);
            (indices[i], indices[j]) = (indices[j], indices[i]);
        }

        return indices;
    }

    private void FillNodeParallel(OctreeNode node, int[] availableIndices, int depth)
    {
        if (availableIndices.Length == 0)
            return;

        // Check max depth
        if (_config.MaxDepth > 0 && node.Level >= _config.MaxDepth)
        {
            // At max depth, add all remaining points
            foreach (var idx in availableIndices)
                node.AddPointIndex(idx);
            OnNodeCompleted?.Invoke(node);
            return;
        }

        // Filter points within this node's bounds - use parallel for large sets
        int[] nodeIndices;
        if (availableIndices.Length > ParallelThreshold)
        {
            nodeIndices = FilterIndicesInBoundsParallel(availableIndices, node.BoundingBox);
        }
        else
        {
            nodeIndices = FilterIndicesInBounds(availableIndices, node.BoundingBox);
        }

        if (nodeIndices.Length < _config.MinPointsForNode)
            return;

        // Get distance threshold for this level
        float distanceThreshold = _config.GetDistanceThreshold(node.Level);

        // Create spatial hash for this node with estimated capacity
        int expectedPoints = Math.Min(_config.PointsPerNode, nodeIndices.Length);
        var spatialHash = new SpatialHash(distanceThreshold, expectedPoints);

        // Selected indices for this node
        var selectedIndices = new List<int>(Math.Min(_config.PointsPerNode, nodeIndices.Length));
        var remainingIndices = new List<int>(nodeIndices.Length);

        int targetCount = Math.Min(_config.PointsPerNode, nodeIndices.Length);

        // Stream through available points
        foreach (var idx in nodeIndices)
        {
            Interlocked.Increment(ref _processedPoints);

            if (selectedIndices.Count >= targetCount)
            {
                remainingIndices.Add(idx);
                continue;
            }

            var position = ReadPosition(idx);

            if (spatialHash.TryAdd(position))
            {
                selectedIndices.Add(idx);
                Interlocked.Increment(ref _assignedPoints);
            }
            else
            {
                remainingIndices.Add(idx);
            }

            // Progress update (throttled)
            long processed = Interlocked.Read(ref _processedPoints);
            if (processed % 50000 == 0)
            {
                long assigned = Interlocked.Read(ref _assignedPoints);
                OnProgress?.Invoke(1, (int)assigned, _totalPoints, node.Level);
            }
        }

        // Add selected to node
        foreach (var idx in selectedIndices)
            node.AddPointIndex(idx);

        OnNodeCompleted?.Invoke(node);

        // Create children if there are remaining points
        if (remainingIndices.Count >= _config.MinPointsForNode)
        {
            CreateChildrenParallel(node, remainingIndices.ToArray());
        }
    }

    private int[] FilterIndicesInBounds(int[] indices, BoundingBox bounds)
    {
        var result = new List<int>(indices.Length / 2);

        foreach (var idx in indices)
        {
            var pos = ReadPosition(idx);
            if (bounds.Contains(ref pos) != ContainmentType.Disjoint)
            {
                result.Add(idx);
            }
        }

        return result.ToArray();
    }

    private int[] FilterIndicesInBoundsParallel(int[] indices, BoundingBox bounds)
    {
        int count = indices.Length;
        var results = new ConcurrentBag<int>();

        // Process in parallel chunks
        int chunkSize = Math.Max(1000, count / (_threadCount * 4));

        Parallel.ForEach(
            Partitioner.Create(0, count, chunkSize),
            new ParallelOptions { MaxDegreeOfParallelism = _threadCount },
            range =>
            {
                var localResults = new List<int>((range.Item2 - range.Item1) / 2);

                for (int i = range.Item1; i < range.Item2; i++)
                {
                    int idx = indices[i];
                    var pos = ReadPosition(idx);
                    if (bounds.Contains(ref pos) != ContainmentType.Disjoint)
                    {
                        localResults.Add(idx);
                    }
                }

                foreach (var idx in localResults)
                    results.Add(idx);
            });

        return results.ToArray();
    }

    private void CreateChildrenParallel(OctreeNode node, int[] remainingIndices)
    {
        var childBounds = node.GenerateChildBounds();

        // Bin points into octants - use parallel for large sets
        var bins = new ConcurrentBag<int>[8];
        for (int i = 0; i < 8; i++)
            bins[i] = new ConcurrentBag<int>();

        if (remainingIndices.Length > ParallelThreshold)
        {
            // Parallel binning
            int chunkSize = Math.Max(1000, remainingIndices.Length / (_threadCount * 4));

            Parallel.ForEach(
                Partitioner.Create(0, remainingIndices.Length, chunkSize),
                new ParallelOptions { MaxDegreeOfParallelism = _threadCount },
                range =>
                {
                    for (int i = range.Item1; i < range.Item2; i++)
                    {
                        int idx = remainingIndices[i];
                        var pos = ReadPosition(idx);
                        int octant = node.GetOctantForPosition(pos);
                        bins[octant].Add(idx);
                    }
                });
        }
        else
        {
            // Sequential binning for small sets
            foreach (var idx in remainingIndices)
            {
                var pos = ReadPosition(idx);
                int octant = node.GetOctantForPosition(pos);
                bins[octant].Add(idx);
            }
        }

        // Create child nodes - process children in parallel at higher levels
        var childTasks = new List<(int octant, int[] indices)>();

        for (int octant = 0; octant < 8; octant++)
        {
            var binIndices = bins[octant].ToArray();
            if (binIndices.Length >= _config.MinPointsForNode)
            {
                childTasks.Add((octant, binIndices));
            }
        }

        // Parallel child processing for levels 0-3 (where parallelism helps most)
        // Sequential for deeper levels to avoid thread contention
        if (node.Level < 3 && childTasks.Count > 1)
        {
            var childNodes = new OctreeNode[childTasks.Count];

            Parallel.For(0, childTasks.Count, new ParallelOptions { MaxDegreeOfParallelism = _threadCount }, i =>
            {
                var (octant, indices) = childTasks[i];

                // Shuffle for randomness
                ShuffleArray(indices);

                string childId = GenerateChildId(node.Id, node.Level, octant);
                var child = new OctreeNode(childId, childBounds[octant], node.Level + 1);
                childNodes[i] = child;

                // Recursively fill child
                FillNodeParallel(child, indices, node.Level + 1);
            });

            // Add children to parent (must be sequential)
            foreach (var child in childNodes)
            {
                if (child != null)
                    node.AddChild(child);
            }
        }
        else
        {
            // Sequential processing for deeper levels
            foreach (var (octant, indices) in childTasks)
            {
                ShuffleArray(indices);

                string childId = GenerateChildId(node.Id, node.Level, octant);
                var child = new OctreeNode(childId, childBounds[octant], node.Level + 1);
                node.AddChild(child);

                FillNodeParallel(child, indices, node.Level + 1);
            }
        }
    }

    private void ShuffleArray(int[] array)
    {
        var random = _threadLocalRandom.Value!;
        int n = array.Length;
        for (int i = n - 1; i > 0; i--)
        {
            int j = random.Next(i + 1);
            (array[i], array[j]) = (array[j], array[i]);
        }
    }

    private string GenerateChildId(string parentId, int parentLevel, int octant)
    {
        int x = (octant & 1) != 0 ? 1 : 0;
        int y = (octant & 2) != 0 ? 1 : 0;
        int z = (octant & 4) != 0 ? 1 : 0;

        int newLevel = parentLevel + 1;

        if (parentLevel == 0)
            return $"{newLevel}_{x}_{y}_{z}";

        var pathStart = parentId.IndexOf('_') + 1;
        var parentPath = parentId.Substring(pathStart);
        return $"{newLevel}_{parentPath}_{x}_{y}_{z}";
    }

    public PlyIndex? GetPlyIndex() => _plyIndex;

    public void Dispose()
    {
        _threadLocalRandom.Dispose();
        _positionsAccessor?.Dispose();
        _positionsMmf?.Dispose();
        _plyIndex?.Dispose();

        // Release in-memory positions
        _positionsInMemory = null;

        // Clean up temp file
        if (_positionsCachePath != null && File.Exists(_positionsCachePath))
        {
            try { File.Delete(_positionsCachePath); }
            catch { }
        }
    }
}
