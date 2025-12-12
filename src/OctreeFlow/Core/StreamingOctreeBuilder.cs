using Stride.Core.Mathematics;
using OctreeFlow.IO;

namespace OctreeFlow.Core;

/// <summary>
/// Memory-efficient octree builder that streams data from disk.
/// Uses spatial hashing for O(1) distance queries.
/// </summary>
public class StreamingOctreeBuilder : IDisposable
{
    private readonly OctreeConfiguration _config;
    private readonly Random _random;
    private readonly string _plyPath;
    private readonly string _tempPath;

    // Positions cache - 12 bytes per point instead of full point data
    private string? _positionsCachePath;
    private FileStream? _positionsStream;
    private BinaryReader? _positionsReader;

    private PlyIndex? _plyIndex;
    private int _totalPoints;
    private int _processedPoints;
    private int _assignedPoints;

    /// <summary>
    /// Progress callback: (phase, current, total, depth)
    /// Phase: 0=indexing, 1=building, 2=writing
    /// </summary>
    public Action<int, int, int, int>? OnProgress { get; set; }

    /// <summary>
    /// Called when a node is completed.
    /// </summary>
    public Action<OctreeNode>? OnNodeCompleted { get; set; }

    public StreamingOctreeBuilder(OctreeConfiguration configuration, string plyPath)
    {
        _config = configuration;
        _config.Validate();
        _random = configuration.RandomSeed.HasValue
            ? new Random(configuration.RandomSeed.Value)
            : new Random();
        _plyPath = plyPath;
        _tempPath = Path.GetTempPath();
    }

    /// <summary>
    /// Builds the octree, returning the root node.
    /// Point indices in nodes reference the original PLY file order.
    /// </summary>
    public OctreeNode Build()
    {
        // Reset counters
        _processedPoints = 0;
        _assignedPoints = 0;
        
        // Phase 0: Index PLY and create positions cache
        Console.WriteLine("  Indexing PLY file...");
        IndexPlyFile();

        if (_plyIndex == null || _totalPoints == 0)
            throw new InvalidOperationException("No points found in PLY file");

        // Create root with cubic bounds
        var bounds = MakeCubicBounds(_plyIndex.Bounds);
        var root = new OctreeNode("0_0_0_0", bounds, 0);

        // Phase 1: Build octree structure
        Console.WriteLine("  Building octree structure...");

        // Create initial available indices (all points)
        var availableIndices = new List<int>(_totalPoints);
        for (int i = 0; i < _totalPoints; i++)
            availableIndices.Add(i);

        // Shuffle for randomness
        ShuffleList(availableIndices);

        // Fill the tree
        FillNode(root, availableIndices);

        return root;
    }

    private void IndexPlyFile()
    {
        _plyIndex = new PlyIndex(_plyPath);
        
        // Create positions cache file path
        _positionsCachePath = Path.Combine(_tempPath, $"octreeflow_pos_{Guid.NewGuid():N}.bin");
        
        // Single pass: build index AND create positions cache simultaneously
        _plyIndex.BuildIndexWithPositionsCache(_positionsCachePath, (current, total) =>
        {
            OnProgress?.Invoke(0, current, total, 0);
        });

        _totalPoints = _plyIndex.VertexCount;

        // Open positions cache for reading
        _positionsStream = File.OpenRead(_positionsCachePath!);
        _positionsReader = new BinaryReader(_positionsStream);
    }

    /// <summary>
    /// Reads a position from the cache file.
    /// </summary>
    private Vector3 ReadPosition(int index)
    {
        if (_positionsStream == null || _positionsReader == null)
            throw new InvalidOperationException("Positions cache not initialized");

        _positionsStream.Position = (long)index * 12; // 3 floats * 4 bytes
        return new Vector3(
            _positionsReader.ReadSingle(),
            _positionsReader.ReadSingle(),
            _positionsReader.ReadSingle()
        );
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

    private void FillNode(OctreeNode node, List<int> availableIndices)
    {
        if (availableIndices.Count == 0)
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

        // Filter points within this node's bounds
        var nodeIndices = FilterIndicesInBounds(availableIndices, node.BoundingBox);

        if (nodeIndices.Count < _config.MinPointsForNode)
            return;

        // Get distance threshold for this level
        float distanceThreshold = _config.GetDistanceThreshold(node.Level);

        // Create spatial hash for this node
        var spatialHash = new SpatialHash(distanceThreshold);

        // Selected indices for this node
        var selectedIndices = new List<int>();
        var remainingIndices = new List<int>();

        int targetCount = Math.Min(_config.PointsPerNode, nodeIndices.Count);
        int processed = 0;

        // Stream through available points (already shuffled)
        foreach (var idx in nodeIndices)
        {
            processed++;
            _processedPoints++;

            if (selectedIndices.Count >= targetCount)
            {
                // Node full, add to remaining
                remainingIndices.Add(idx);
                continue;
            }

            var position = ReadPosition(idx);

            // Use spatial hash for O(1) distance check
            if (spatialHash.TryAdd(position))
            {
                selectedIndices.Add(idx);
                _assignedPoints++;
            }
            else
            {
                // Too close to existing point, goes to children
                remainingIndices.Add(idx);
            }

            // Progress for building phase - report total assigned vs total points
            if (_processedPoints % 5000 == 0)
                OnProgress?.Invoke(1, _assignedPoints, _totalPoints, node.Level);
        }

        // Add selected to node
        foreach (var idx in selectedIndices)
            node.AddPointIndex(idx);

        OnNodeCompleted?.Invoke(node);

        // Create children if there are remaining points
        if (remainingIndices.Count >= _config.MinPointsForNode)
        {
            CreateChildren(node, remainingIndices);
        }
    }

    private List<int> FilterIndicesInBounds(List<int> indices, BoundingBox bounds)
    {
        var result = new List<int>();

        foreach (var idx in indices)
        {
            var pos = ReadPosition(idx);
            if (bounds.Contains(ref pos) != ContainmentType.Disjoint)
            {
                result.Add(idx);
            }
        }

        return result;
    }

    private void CreateChildren(OctreeNode node, List<int> remainingIndices)
    {
        var childBounds = node.GenerateChildBounds();

        // Bin points into octants
        var bins = new List<int>[8];
        for (int i = 0; i < 8; i++)
            bins[i] = new List<int>();

        foreach (var idx in remainingIndices)
        {
            var pos = ReadPosition(idx);
            int octant = node.GetOctantForPosition(pos);
            bins[octant].Add(idx);
        }

        // Create child nodes
        for (int octant = 0; octant < 8; octant++)
        {
            if (bins[octant].Count >= _config.MinPointsForNode)
            {
                string childId = GenerateChildId(node.Id, node.Level, octant);
                var child = new OctreeNode(childId, childBounds[octant], node.Level + 1);
                node.AddChild(child);

                // Shuffle for randomness
                ShuffleList(bins[octant]);

                // Recursively fill child
                FillNode(child, bins[octant]);
            }
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

    private void ShuffleList<T>(List<T> list)
    {
        int n = list.Count;
        while (n > 1)
        {
            n--;
            int k = _random.Next(n + 1);
            (list[k], list[n]) = (list[n], list[k]);
        }
    }

    /// <summary>
    /// Gets the PLY index (for serialization).
    /// </summary>
    public PlyIndex? GetPlyIndex() => _plyIndex;

    public void Dispose()
    {
        _positionsReader?.Dispose();
        _positionsStream?.Dispose();
        _plyIndex?.Dispose();

        // Clean up temp file
        if (_positionsCachePath != null && File.Exists(_positionsCachePath))
        {
            try { File.Delete(_positionsCachePath); } catch { }
        }
    }
}

