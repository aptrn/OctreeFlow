using Stride.Core.Mathematics;
using System.Text.Json.Serialization;

namespace OctreeFlow.Core;

/// <summary>
/// Represents a node in the octree structure.
/// Each node contains a set of point indices and can have up to 8 children.
/// </summary>
public class OctreeNode
{
    /// <summary>
    /// Unique identifier in format: depth_x_y_z (e.g., "0_0_0_0" for root, "1_0_1_0" for child).
    /// </summary>
    public string Id { get; set; }

    /// <summary>
    /// The bounding box defining this node's spatial extent.
    /// </summary>
    [JsonIgnore]
    public BoundingBox BoundingBox { get; set; }

    /// <summary>
    /// JSON-serializable bounding box representation.
    /// </summary>
    public float[] BoundingBoxData
    {
        get => new[]
        {
            BoundingBox.Minimum.X, BoundingBox.Minimum.Y, BoundingBox.Minimum.Z,
            BoundingBox.Maximum.X, BoundingBox.Maximum.Y, BoundingBox.Maximum.Z
        };
        set
        {
            if (value.Length >= 6)
            {
                BoundingBox = new BoundingBox(
                    new Vector3(value[0], value[1], value[2]),
                    new Vector3(value[3], value[4], value[5])
                );
            }
        }
    }

    /// <summary>
    /// Indices of points assigned to this node.
    /// </summary>
    public List<int> PointIndices { get; set; }

    /// <summary>
    /// Depth level in the octree (0 = root).
    /// </summary>
    public int Level { get; set; }

    /// <summary>
    /// Child nodes (up to 8).
    /// </summary>
    public List<OctreeNode> Children { get; set; }

    /// <summary>
    /// Byte offset in the .octree data blob where this node's points start.
    /// Used for streaming/loading.
    /// </summary>
    public long DataOffset { get; set; }

    /// <summary>
    /// Number of bytes this node's point data occupies in the blob.
    /// </summary>
    public int DataSize { get; set; }

    public OctreeNode()
    {
        Id = "0_0_0_0";
        BoundingBox = new BoundingBox();
        PointIndices = new List<int>();
        Level = 0;
        Children = new List<OctreeNode>();
        DataOffset = 0;
        DataSize = 0;
    }

    public OctreeNode(string id, BoundingBox boundingBox, int level)
    {
        Id = id;
        BoundingBox = boundingBox;
        PointIndices = new List<int>();
        Level = level;
        Children = new List<OctreeNode>();
        DataOffset = 0;
        DataSize = 0;
    }

    /// <summary>
    /// Returns true if this node has no children.
    /// </summary>
    [JsonIgnore]
    public bool IsLeaf => Children.Count == 0;

    /// <summary>
    /// Returns the number of points in this node.
    /// </summary>
    [JsonIgnore]
    public int PointCount => PointIndices.Count;

    /// <summary>
    /// Adds a point index to this node.
    /// </summary>
    public void AddPointIndex(int index)
    {
        PointIndices.Add(index);
    }

    /// <summary>
    /// Adds a child node.
    /// </summary>
    public void AddChild(OctreeNode child)
    {
        Children.Add(child);
    }

    /// <summary>
    /// Generates child bounding boxes by splitting this node's bounds.
    /// Returns 8 boxes indexed by octant (0-7).
    /// </summary>
    public BoundingBox[] GenerateChildBounds()
    {
        var result = new BoundingBox[8];
        var center = BoundingBox.Center;
        var min = BoundingBox.Minimum;
        var max = BoundingBox.Maximum;

        // Octant layout (looking down -Z):
        //   Y+
        //   |  6---7
        //   | /|  /|
        //   |4---5 |
        //   || 2-|-3
        //   |/   |/
        //   0---1----X+
        //  /
        // Z+

        // 0: min corner
        result[0] = new BoundingBox(min, center);

        // 1: +X
        result[1] = new BoundingBox(
            new Vector3(center.X, min.Y, min.Z),
            new Vector3(max.X, center.Y, center.Z));

        // 2: +Y
        result[2] = new BoundingBox(
            new Vector3(min.X, center.Y, min.Z),
            new Vector3(center.X, max.Y, center.Z));

        // 3: +X +Y
        result[3] = new BoundingBox(
            new Vector3(center.X, center.Y, min.Z),
            new Vector3(max.X, max.Y, center.Z));

        // 4: +Z
        result[4] = new BoundingBox(
            new Vector3(min.X, min.Y, center.Z),
            new Vector3(center.X, center.Y, max.Z));

        // 5: +X +Z
        result[5] = new BoundingBox(
            new Vector3(center.X, min.Y, center.Z),
            new Vector3(max.X, center.Y, max.Z));

        // 6: +Y +Z
        result[6] = new BoundingBox(
            new Vector3(min.X, center.Y, center.Z),
            new Vector3(center.X, max.Y, max.Z));

        // 7: +X +Y +Z (max corner)
        result[7] = new BoundingBox(center, max);

        return result;
    }

    /// <summary>
    /// Generates the child ID for a given octant.
    /// </summary>
    public string GenerateChildId(int octant)
    {
        int x = (octant & 1) != 0 ? 1 : 0;
        int y = (octant & 2) != 0 ? 1 : 0;
        int z = (octant & 4) != 0 ? 1 : 0;

        // Parse current ID to get position
        var parts = Id.Split('_');
        if (parts.Length >= 4)
        {
            // For non-root nodes, append the octant position
            return $"{Level + 1}_{Id.Substring(Id.IndexOf('_') + 1)}_{x}_{y}_{z}";
        }

        return $"{Level + 1}_{x}_{y}_{z}";
    }

    /// <summary>
    /// Gets the octant index (0-7) for a given position within this node.
    /// </summary>
    public int GetOctantForPosition(Vector3 position)
    {
        var center = BoundingBox.Center;
        int octant = 0;

        if (position.X >= center.X) octant |= 1;
        if (position.Y >= center.Y) octant |= 2;
        if (position.Z >= center.Z) octant |= 4;

        return octant;
    }

    /// <summary>
    /// Returns total number of nodes in this subtree (including this node).
    /// </summary>
    public int GetTotalNodeCount()
    {
        int count = 1;
        foreach (var child in Children)
        {
            count += child.GetTotalNodeCount();
        }
        return count;
    }

    /// <summary>
    /// Returns total number of points in this subtree.
    /// </summary>
    public int GetTotalPointCount()
    {
        int count = PointIndices.Count;
        foreach (var child in Children)
        {
            count += child.GetTotalPointCount();
        }
        return count;
    }

    /// <summary>
    /// Returns the maximum depth in this subtree.
    /// </summary>
    public int GetMaxDepth()
    {
        int maxChildDepth = Level;
        foreach (var child in Children)
        {
            maxChildDepth = Math.Max(maxChildDepth, child.GetMaxDepth());
        }
        return maxChildDepth;
    }
}

