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
    /// Checks if this node needs more detail (should continue to children) based on camera distance and a 0–100% threshold.
    /// At 100% threshold: target detail is maximum level over the whole frustum (full detail everywhere).
    /// At 0% threshold: target detail is minimum level over the whole frustum (coarse everywhere).
    /// In between: detail scales from close (more detail) to far (less detail); curvature skews where the "middle ground" sits.
    /// </summary>
    /// <param name="cameraPosition">The camera/viewer position.</param>
    /// <param name="thresholdPercent">0–100. At 100% use max level everywhere; at 0% use min level everywhere; in between use gradient by distance.</param>
    /// <param name="minLevel">Minimum octree level to use (coarser detail).</param>
    /// <param name="maxLevel">Maximum octree level to use (finest detail).</param>
    /// <param name="curvature">0–1, default 0.5. Skews the distance-to-detail curve: 0.5 = linear; &lt; 0.5 = middle shifts toward camera (faster falloff); &gt; 0.5 = middle shifts toward far (longer high-detail range).</param>
    /// <param name="frustumNear">Distance to near plane; points at this distance are treated as "close" (detail factor 1).</param>
    /// <param name="frustumFar">Distance to far plane; points at this distance are treated as "far" (detail factor 0).</param>
    /// <returns>True if this node should subdivide to reach the target level for its distance.</returns>
    public bool NeedsMoreDetail(
        Vector3 cameraPosition,
        float thresholdPercent,
        int minLevel,
        int maxLevel,
        float curvature = 0.5f,
        float frustumNear = 0.1f,
        float frustumFar = 1000f)
    {
        if (IsLeaf)
            return false;

        int lo = Math.Min(minLevel, maxLevel);
        int hi = Math.Max(minLevel, maxLevel);
        float t = Math.Clamp(thresholdPercent / 100f, 0f, 1f);

        // Distance from camera to closest point on this node's bounds
        var closest = Vector3.Clamp(cameraPosition, BoundingBox.Minimum, BoundingBox.Maximum);
        float distance = Vector3.Distance(cameraPosition, closest);

        // Map distance to 0 (far)..1 (close) over the frustum length
        float range = Math.Max(frustumFar - frustumNear, 0.0001f);
        float distanceFactor = 1f - Math.Clamp((distance - frustumNear) / range, 0f, 1f);

        // Curvature skews linearity: 0.5 = linear (exponent 1), <0.5 = faster falloff (exponent <1), >0.5 = longer high-detail range (exponent >1)
        float exponent = MathF.Pow(2f, (Math.Clamp(curvature, 0f, 1f) - 0.5f) * 2f);
        distanceFactor = MathF.Pow(Math.Max(distanceFactor, 1e-6f), exponent);

        // At 0%: alpha=0 (min level everywhere). At 100%: alpha=1 (max level everywhere).
        // In between: gradient; strength k = 4*t*(1-t), so k=0 at 0% and 100%, k=1 at 50%.
        float k = 4f * t * (1f - t);
        float alpha = Math.Clamp(t + k * (distanceFactor - 0.5f), 0f, 1f);

        float targetLevelF = lo + (hi - lo) * alpha;
        int targetLevel = (int)Math.Round(targetLevelF);

        return Level < targetLevel;
    }
}

