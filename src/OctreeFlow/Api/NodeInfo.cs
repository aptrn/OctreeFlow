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
    /// Spacing/characteristic size of this node (largest dimension).
    /// Use this directly for LOD calculations: screenSize = Spacing / distance
    /// Larger values = coarser nodes (closer to root), smaller values = finer nodes (leaves).
    /// </summary>
    public float Spacing { get; }

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
        
        // Calculate spacing as the largest dimension of the node
        var size = BoundingBox.Maximum - BoundingBox.Minimum;
        Spacing = Math.Max(Math.Max(size.X, size.Y), size.Z);
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

    /// <summary>
    /// Calculates the screen-space size of this node for LOD decisions.
    /// Returns Spacing / distance, where distance is measured to the closest point on the bounding box.
    /// Higher values mean the node appears larger on screen and may need more detail.
    /// </summary>
    /// <param name="cameraPosition">The camera/viewer position.</param>
    /// <returns>Screen-space size metric. Compare against a threshold (e.g., 1.0) to decide if more detail is needed.</returns>
    public float ScreenSize(Vector3 cameraPosition)
    {
        // Get closest point on bounding box to camera
        var closest = Vector3.Clamp(cameraPosition, BoundingBox.Minimum, BoundingBox.Maximum);
        var distance = Vector3.Distance(cameraPosition, closest);
        
        // Avoid division by zero when camera is inside the node
        return Spacing / Math.Max(distance, 0.0001f);
    }

    /// <summary>
    /// Checks if this node needs more detail (should continue to children) based on camera distance.
    /// </summary>
    /// <param name="cameraPosition">The camera/viewer position.</param>
    /// <param name="threshold">Screen-size threshold. Lower = more detail. Default 1.0, try 0.5-2.0.</param>
    /// <returns>True if the node appears large enough on screen to warrant subdivision.</returns>
    public bool NeedsMoreDetail(Vector3 cameraPosition, float threshold = 1.0f)
    {
        return ScreenSize(cameraPosition) > threshold && !IsLeaf;
    }
}

