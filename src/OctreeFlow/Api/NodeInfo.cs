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
    /// Checks if this node needs more detail (should continue to children) based on camera distance and a curve through a middle point.
    /// The curve connects (Near Plane, Max LOD) to (Far Plane, Min LOD) and passes through a controllable middle point.
    /// When both middle point controls are 0, the curve is linear; moving either skews the curve.
    /// </summary>
    /// <param name="cameraPosition">The camera/viewer position.</param>
    /// <param name="minLevel">Minimum octree level to use (coarser detail).</param>
    /// <param name="maxLevel">Maximum octree level to use (finest detail).</param>
    /// <param name="middlePointX">-1 to 1, default 0. Where along the distance axis the middle point sits: -1 = near plane, 0 = halfway, 1 = far plane.</param>
    /// <param name="middlePointY">-1 to 1, default 0. LOD at the middle point: -1 = min LOD, 0 = halfway, 1 = max LOD.</param>
    /// <param name="frustumNear">Distance to near plane.</param>
    /// <param name="frustumFar">Distance to far plane.</param>
    /// <returns>True if this node should subdivide to reach the target level for its distance.</returns>
    public bool NeedsMoreDetail(
        Vector3 cameraPosition,
        int minLevel,
        int maxLevel,
        float middlePointX = 0f,
        float middlePointY = 0f,
        float frustumNear = 0.1f,
        float frustumFar = 1000f)
    {
        if (IsLeaf)
            return false;

        int lo = Math.Min(minLevel, maxLevel);
        int hi = Math.Max(minLevel, maxLevel);

        // Distance from camera to closest point on this node's bounds
        var closest = Vector3.Clamp(cameraPosition, BoundingBox.Minimum, BoundingBox.Maximum);
        float distance = Vector3.Distance(cameraPosition, closest);

        // Normalized distance along frustum: 0 = near plane, 1 = far plane
        float range = Math.Max(frustumFar - frustumNear, 0.0001f);
        float dNorm = Math.Clamp((distance - frustumNear) / range, 0f, 1f);

        // Middle point in normalized space: X/Y in [-1,1] -> (0.5 + 0.5*x) in [0,1]
        float midX = Math.Clamp(middlePointX, -1f, 1f);
        float midY = Math.Clamp(middlePointY, -1f, 1f);
        float p1x = (1f + midX) * 0.5f;
        float p1y = (1f + midY) * 0.5f;

        // Quadratic curve through P0=(0,1) [near, max LOD], P1=(p1x, p1y) [middle], P2=(1,0) [far, min LOD]
        // Parametric: x(t) and y(t) with t in [0,1], using Lagrange-style quadratic through t=0, 0.5, 1
        // x(t) = 4*p1x*t*(1-t) + t*(2t-1)  =>  solve for t given dNorm
        float lodNorm = EvaluateLodCurve(dNorm, p1x, p1y);

        float targetLevelF = lo + (hi - lo) * lodNorm;
        int targetLevel = (int)Math.Round(Math.Clamp(targetLevelF, lo, hi));

        return Level < targetLevel;
    }

    /// <summary>
    /// Given normalized distance dNorm in [0,1], returns LOD norm in [0,1] from the quadratic through (0,1), (p1x,p1y), (1,0).
    /// </summary>
    private static float EvaluateLodCurve(float dNorm, float p1x, float p1y)
    {
        // Solve for t such that x(t) = dNorm, then return y(t).
        // x(t) = (2 - 4*p1x)*t^2 + (4*p1x - 1)*t = dNorm  =>  a*t^2 + b*t - dNorm = 0
        float a = 2f - 4f * p1x;
        float b = 4f * p1x - 1f;

        float t;
        if (Math.Abs(a) < 1e-6f)
        {
            // Linear: x(t) = b*t, so t = dNorm / b (b = 1 when p1x = 0.5)
            t = Math.Abs(b) < 1e-6f ? dNorm : Math.Clamp(dNorm / b, 0f, 1f);
        }
        else
        {
            float disc = b * b + 4f * a * dNorm;
            if (disc < 0f)
                disc = 0f;
            float sqrt = MathF.Sqrt(disc);
            float t1 = (-b - sqrt) / (2f * a);
            float t2 = (-b + sqrt) / (2f * a);
            // Pick the root in [0,1] for which x(t) is closest to dNorm
            float x1 = a * t1 * t1 + b * t1;
            float x2 = a * t2 * t2 + b * t2;
            bool t1In = t1 >= 0f && t1 <= 1f;
            bool t2In = t2 >= 0f && t2 <= 1f;
            if (t1In && t2In)
                t = Math.Abs(x1 - dNorm) <= Math.Abs(x2 - dNorm) ? t1 : t2;
            else if (t1In)
                t = t1;
            else if (t2In)
                t = t2;
            else
                t = Math.Clamp(dNorm, 0f, 1f);
        }

        t = Math.Clamp(t, 0f, 1f);

        // y(t) = (1-t)(1-2t)*1 + 4*t*(1-t)*p1y + t*(2t-1)*0 = (1-t)(1-2t) + 4*p1y*t*(1-t)
        float y = (1f - t) * (1f - 2f * t) + 4f * p1y * t * (1f - t);
        return Math.Clamp(y, 0f, 1f);
    }
}

