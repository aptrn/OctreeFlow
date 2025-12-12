using Stride.Core.Mathematics;
using OctreeFlow.Data;

namespace OctreeFlow.Core;

/// <summary>
/// Builds an octree from a point cloud using the specified configuration.
/// Implements the filling cycle algorithm with distance-based point selection.
/// </summary>
public class OctreeBuilder
{
    private readonly OctreeConfiguration _config;
    private readonly Random _random;

    /// <summary>
    /// Progress callback: (currentPoints, totalPoints, currentDepth)
    /// </summary>
    public Action<int, int, int>? OnProgress { get; set; }

    /// <summary>
    /// Called when a node is completed.
    /// </summary>
    public Action<OctreeNode>? OnNodeCompleted { get; set; }

    public OctreeBuilder(OctreeConfiguration configuration)
    {
        _config = configuration;
        _config.Validate();
        _random = configuration.RandomSeed.HasValue
            ? new Random(configuration.RandomSeed.Value)
            : new Random();
    }

    /// <summary>
    /// Builds an octree from the given point cloud.
    /// </summary>
    public OctreeNode Build(PointCloud cloud)
    {
        if (cloud.TotalCount == 0)
            throw new ArgumentException("Point cloud is empty");

        // Ensure bounds are computed
        cloud.ComputeBounds();

        // Create root node with expanded bounds (cubic)
        var bounds = MakeCubicBounds(cloud.Bounds);
        var root = new OctreeNode("0_0_0_0", bounds, 0);

        // Start filling from root
        var availableIndices = cloud.GetAvailableIndices();
        FillNode(root, cloud, availableIndices);

        return root;
    }

    /// <summary>
    /// Makes bounds cubic by expanding to the largest dimension.
    /// </summary>
    private BoundingBox MakeCubicBounds(BoundingBox bounds)
    {
        var size = bounds.Maximum - bounds.Minimum;
        var maxDim = Math.Max(Math.Max(size.X, size.Y), size.Z);
        var center = bounds.Center;

        var halfSize = maxDim / 2f * 1.01f; // Small padding
        return new BoundingBox(
            center - new Vector3(halfSize),
            center + new Vector3(halfSize)
        );
    }

    /// <summary>
    /// Fills a node with points using the distance-based selection algorithm.
    /// </summary>
    private void FillNode(OctreeNode node, PointCloud cloud, List<int> availableIndices)
    {
        if (availableIndices.Count == 0)
            return;

        if (_config.MaxDepth > 0 && node.Level >= _config.MaxDepth)
        {
            // At max depth, just add all remaining points
            foreach (var idx in availableIndices)
            {
                node.AddPointIndex(idx);
            }
            OnNodeCompleted?.Invoke(node);
            return;
        }

        // Filter points that are within this node's bounding box
        var nodePoints = FilterPointsInBounds(availableIndices, cloud, node.BoundingBox);

        if (nodePoints.Count < _config.MinPointsForNode)
        {
            // Not enough points to create this node meaningfully
            return;
        }

        // Get distance threshold for this level
        float distanceThreshold = _config.GetDistanceThreshold(node.Level);
        float distanceThresholdSq = distanceThreshold * distanceThreshold;

        // Selected points for this node (as positions for distance checking)
        var selectedPositions = new List<Vector3>();
        var selectedIndices = new List<int>();

        // Remaining points after selection
        var remainingIndices = new HashSet<int>(nodePoints);

        int attempts = 0;
        int targetCount = Math.Min(_config.PointsPerNode, nodePoints.Count);

        // Filling cycle: select points until we have N or exhaust attempts
        while (selectedIndices.Count < targetCount && remainingIndices.Count > 0 && attempts < _config.MaxSelectionAttempts)
        {
            attempts++;

            // Get random point from remaining
            int randomIndex = GetRandomFromSet(remainingIndices);
            var point = cloud.GetPoint(randomIndex);

            // Check distance from all previously selected points
            if (IsDistantEnough(point.Position, selectedPositions, distanceThresholdSq))
            {
                // Point passes distance check - add it
                selectedPositions.Add(point.Position);
                selectedIndices.Add(randomIndex);
                remainingIndices.Remove(randomIndex);

                // Report progress
                OnProgress?.Invoke(selectedIndices.Count, cloud.TotalCount, node.Level);
            }
            // If not distant enough, we just try again with another random point
            // The point stays in remainingIndices for potential child nodes
        }

        // Add selected indices to this node
        foreach (var idx in selectedIndices)
        {
            node.AddPointIndex(idx);
        }

        OnNodeCompleted?.Invoke(node);

        // If there are remaining points, create children
        if (remainingIndices.Count > 0)
        {
            CreateChildren(node, cloud, remainingIndices.ToList());
        }
    }

    /// <summary>
    /// Filters indices to only those within the bounding box.
    /// </summary>
    private List<int> FilterPointsInBounds(List<int> indices, PointCloud cloud, BoundingBox bounds)
    {
        var result = new List<int>();
        foreach (var idx in indices)
        {
            var pos = cloud.GetPoint(idx).Position;
            if (bounds.Contains(ref pos) != ContainmentType.Disjoint)
            {
                result.Add(idx);
            }
        }
        return result;
    }

    /// <summary>
    /// Gets a random element from a set.
    /// </summary>
    private int GetRandomFromSet(HashSet<int> set)
    {
        int index = _random.Next(set.Count);
        return set.Skip(index).First();
    }

    /// <summary>
    /// Checks if a position is distant enough from all selected positions.
    /// </summary>
    private bool IsDistantEnough(Vector3 position, List<Vector3> selectedPositions, float distanceThresholdSq)
    {
        foreach (var selected in selectedPositions)
        {
            var diff = position - selected;
            float distSq = diff.X * diff.X + diff.Y * diff.Y + diff.Z * diff.Z;

            if (distSq < distanceThresholdSq)
                return false;
        }
        return true;
    }

    /// <summary>
    /// Creates child nodes by splitting the bounding box and distributing remaining points.
    /// </summary>
    private void CreateChildren(OctreeNode node, PointCloud cloud, List<int> remainingIndices)
    {
        // Generate 8 child bounding boxes
        var childBounds = node.GenerateChildBounds();

        // Bin points into octants
        var bins = new List<int>[8];
        for (int i = 0; i < 8; i++)
        {
            bins[i] = new List<int>();
        }

        foreach (var idx in remainingIndices)
        {
            var pos = cloud.GetPoint(idx).Position;
            int octant = node.GetOctantForPosition(pos);
            bins[octant].Add(idx);
        }

        // Create child nodes for non-empty bins
        for (int octant = 0; octant < 8; octant++)
        {
            if (bins[octant].Count >= _config.MinPointsForNode)
            {
                string childId = GenerateChildId(node.Id, node.Level, octant);
                var child = new OctreeNode(childId, childBounds[octant], node.Level + 1);
                node.AddChild(child);

                // Recursively fill child
                FillNode(child, cloud, bins[octant]);
            }
        }
    }

    /// <summary>
    /// Generates a child node ID.
    /// Format: level_path where path accumulates the octant choices.
    /// </summary>
    private string GenerateChildId(string parentId, int parentLevel, int octant)
    {
        int x = (octant & 1) != 0 ? 1 : 0;
        int y = (octant & 2) != 0 ? 1 : 0;
        int z = (octant & 4) != 0 ? 1 : 0;

        int newLevel = parentLevel + 1;

        if (parentLevel == 0)
        {
            // First level children: just level_x_y_z
            return $"{newLevel}_{x}_{y}_{z}";
        }
        else
        {
            // Subsequent levels: append octant to path
            // Remove the level prefix from parent ID and append
            var pathStart = parentId.IndexOf('_') + 1;
            var parentPath = parentId.Substring(pathStart);
            return $"{newLevel}_{parentPath}_{x}_{y}_{z}";
        }
    }
}

