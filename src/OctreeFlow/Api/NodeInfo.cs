using Stride.Core.Mathematics;
using OctreeFlow.Core;

namespace OctreeFlow.Api;

/// <summary>
/// Read-only information about a node passed to traversal delegates.
/// Contains all relevant node metadata for making traversal decisions.
/// </summary>
public class NodeInfo
{
    /// <summary>
    /// Unique identifier in format: depth_x_y_z (e.g., "0_0_0_0" for root).
    /// </summary>
    public string Id { get; }

    /// <summary>
    /// The bounding box defining this node's spatial extent.
    /// </summary>
    public BoundingBox BoundingBox { get; }

    /// <summary>
    /// Center point of the bounding box.
    /// </summary>
    public Vector3 Center => BoundingBox.Center;

    /// <summary>
    /// Size of the bounding box (max - min).
    /// </summary>
    public Vector3 Size => BoundingBox.Maximum - BoundingBox.Minimum;

    /// <summary>
    /// Number of points in this node.
    /// </summary>
    public int PointCount { get; }

    /// <summary>
    /// Depth level in the octree (0 = root).
    /// </summary>
    public int Level { get; }

    /// <summary>
    /// Number of direct children this node has.
    /// </summary>
    public int ChildCount { get; }

    /// <summary>
    /// Returns true if this node has no children.
    /// </summary>
    public bool IsLeaf { get; }

    /// <summary>
    /// Whether this node is currently loaded in RAM cache.
    /// </summary>
    public bool IsInCache { get; internal set; }

    /// <summary>
    /// Whether this node is currently loaded on GPU.
    /// </summary>
    public bool IsOnGpu { get; internal set; }

    /// <summary>
    /// The GPU sector index if loaded, -1 otherwise.
    /// </summary>
    public int GpuSectorIndex { get; internal set; } = -1;

    /// <summary>
    /// Reference to the underlying octree node (internal use).
    /// </summary>
    internal OctreeNode Node { get; }

    public NodeInfo(OctreeNode node)
    {
        Node = node;
        Id = node.Id;
        BoundingBox = node.BoundingBox;
        PointCount = node.PointCount;
        Level = node.Level;
        ChildCount = node.Children?.Count ?? 0;
        IsLeaf = node.IsLeaf;
    }

    /// <summary>
    /// Updates cache/GPU status from managers.
    /// </summary>
    internal void UpdateStatus(bool isInCache, bool isOnGpu, int gpuSectorIndex)
    {
        IsInCache = isInCache;
        IsOnGpu = isOnGpu;
        GpuSectorIndex = gpuSectorIndex;
    }
}

