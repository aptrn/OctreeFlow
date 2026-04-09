using Stride.Core.Mathematics;
using OctreeFlow.Core;
using System.Runtime.CompilerServices;

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

    // ── Precomputed metadata ──────────────────────────────────────────────────

    /// <summary>
    /// Average color of the points in this node (RGBA, normalized 0–1).
    /// Returns white if metadata has not been computed for this node.
    /// </summary>
    public Color4 AverageColor => Node.AverageColor;

    /// <summary>
    /// Raw point density: points per unit volume of the bounding box.
    /// 0 if metadata has not been computed.
    /// </summary>
    public float PointDensityRaw => Node.PointDensityRaw;

    /// <summary>
    /// Normalized point density in [0, 1], where 1 is the densest node in the tree.
    /// 0 if metadata has not been computed.
    /// </summary>
    public float PointDensity => Node.PointDensity;

    /// <summary>
    /// True if per-node metadata (AverageColor, PointDensity) has been computed for this node.
    /// </summary>
    public bool HasMetadata => Node.HasMetadata;

    /// <summary>
    /// Sequential integer identity assigned at load time (0 = root, DFS order).
    /// Stable across traversal passes — the same node always returns the same value.
    /// Indexes linear BF buffers from <see cref="OctreeFlowReader.BuildStaticNodeData"/> and matches <c>Point_NodeID</c>.
    /// </summary>
    public int NodeId { get; }

    // ── Internal ─────────────────────────────────────────────────────────────

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
        NodeId = node.IntId;

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

    // ── Distance helpers ──────────────────────────────────────────────────────

    /// <summary>
    /// Returns the shortest Euclidean distance from an arbitrary world-space point to this node's
    /// bounding box. Returns 0 if the point is inside the box.
    /// </summary>
    public float DistanceFromPoint(Vector3 point)
    {
        var closest = Vector3.Clamp(point, BoundingBox.Minimum, BoundingBox.Maximum);
        return Vector3.Distance(point, closest);
    }

    /// <summary>
    /// Returns the shortest world-space distance from the camera (derived from the View matrix)
    /// to the nearest point on this node's bounding box. Returns 0 when the camera is inside the node.
    /// </summary>
    /// <param name="view">The camera view matrix (world → camera space).</param>
    /// <param name="projection">The projection matrix (unused here, included for API consistency).</param>
    public float DistanceFromCamera(Matrix view, Matrix projection)
    {
        var cameraPos = ExtractCameraPosition(view);
        var closest = Vector3.Clamp(cameraPos, BoundingBox.Minimum, BoundingBox.Maximum);
        return Vector3.Distance(cameraPos, closest);
    }

    // ── Screen-space helpers ──────────────────────────────────────────────────

    /// <summary>
    /// Computes the approximate screen-space size of this node's bounding box by projecting
    /// all 8 corners through the combined ViewProjection matrix and measuring the 2-D extent.
    /// </summary>
    /// <param name="view">Camera View matrix.</param>
    /// <param name="projection">Camera Projection matrix.</param>
    /// <param name="viewportWidth">
    /// Viewport width in pixels. Pass 1.0 (default) to get a normalised [0,1] fraction.
    /// </param>
    /// <param name="viewportHeight">
    /// Viewport height in pixels. Pass 1.0 (default) to get a normalised [0,1] fraction.
    /// </param>
    /// <returns>
    /// Approximate screen-space width and height. Units depend on the viewport dimensions supplied.
    /// Returns <see cref="Vector2.Zero"/> if the entire node is behind the camera.
    /// </returns>
    public Vector2 ScreenSpaceSize(Matrix view, Matrix projection,
        float viewportWidth = 1f, float viewportHeight = 1f)
    {
        var vp = view * projection;
        ProjectBoundingBoxToNdc(vp, BoundingBox,
            out float ndcMinX, out float ndcMaxX,
            out float ndcMinY, out float ndcMaxY,
            out bool anyVisible);

        if (!anyVisible) return Vector2.Zero;

        // NDC ranges [-1,1]; multiply by 0.5 * viewport to convert to pixels.
        float w = Math.Max(0f, ndcMaxX - ndcMinX) * 0.5f * viewportWidth;
        float h = Math.Max(0f, ndcMaxY - ndcMinY) * 0.5f * viewportHeight;
        return new Vector2(w, h);
    }

    // ── Visibility ────────────────────────────────────────────────────────────

    /// <summary>
    /// Determines this node's visibility status against the camera frustum and, optionally,
    /// a set of potential occluders.
    /// </summary>
    /// <param name="view">Camera View matrix.</param>
    /// <param name="projection">Camera Projection matrix.</param>
    /// <param name="viewportWidth">
    /// Viewport width used for the occlusion screen-space test (pixels or normalised).
    /// Set to 0 to skip the occlusion test entirely.
    /// </param>
    /// <param name="viewportHeight">Viewport height (same units as <paramref name="viewportWidth"/>).</param>
    /// <param name="potentialOccluders">
    /// Nodes that might occlude this node. If null or empty the occlusion test is skipped.
    /// Pass coarser parent / sibling nodes for an approximate hierarchical occlusion check.
    /// </param>
    /// <returns>
    /// <see cref="VisibilityStatus.Hidden"/> — outside frustum or fully occluded.<br/>
    /// <see cref="VisibilityStatus.HalfCovered"/> — partially inside frustum or partially occluded.<br/>
    /// <see cref="VisibilityStatus.InView"/> — fully inside frustum and not occluded.
    /// </returns>
    public VisibilityStatus GetVisibility(
        Matrix view,
        Matrix projection,
        float viewportWidth = 0f,
        float viewportHeight = 0f,
        IEnumerable<NodeInfo>? potentialOccluders = null)
    {
        var vp = view * projection;

        // ── 1. Frustum test ──────────────────────────────────────────────────
        var frustumStatus = FrustumTest(vp, BoundingBox);
        if (frustumStatus == VisibilityStatus.Hidden)
            return VisibilityStatus.Hidden;

        // ── 2. Approximate occlusion test ────────────────────────────────────
        if (viewportWidth > 0f && viewportHeight > 0f && potentialOccluders != null)
        {
            ProjectBoundingBoxToNdc(vp, BoundingBox,
                out float minX, out float maxX,
                out float minY, out float maxY,
                out bool visible);

            if (!visible)
                return VisibilityStatus.Hidden;

            foreach (var occluder in potentialOccluders)
            {
                if (ReferenceEquals(occluder, this)) continue;

                ProjectBoundingBoxToNdc(vp, occluder.BoundingBox,
                    out float oMinX, out float oMaxX,
                    out float oMinY, out float oMaxY,
                    out bool oVisible);

                if (!oVisible) continue;

                // Check whether this node's screen-rect is fully inside the occluder's.
                if (oMinX <= minX && oMaxX >= maxX &&
                    oMinY <= minY && oMaxY >= maxY)
                {
                    return VisibilityStatus.Hidden;
                }

                // Partial overlap → at least HalfCovered.
                if (oMaxX > minX && oMinX < maxX &&
                    oMaxY > minY && oMinY < maxY)
                {
                    frustumStatus = VisibilityStatus.HalfCovered;
                }
            }
        }

        return frustumStatus;
    }

    // ── Private helpers ───────────────────────────────────────────────────────

    /// <summary>
    /// Extracts the camera world-space position from the View matrix.
    /// Assumes an orthonormal rotation block (no scale).
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector3 ExtractCameraPosition(Matrix view)
    {
        // For an orthonormal view matrix V = [R | -R*camPos] (row-major):
        //   camPos = -R^T * t  where t = (M41, M42, M43)
        float tx = view.M41, ty = view.M42, tz = view.M43;
        return new Vector3(
            -(view.M11 * tx + view.M21 * ty + view.M31 * tz),
            -(view.M12 * tx + view.M22 * ty + view.M32 * tz),
            -(view.M13 * tx + view.M23 * ty + view.M33 * tz));
    }

    /// <summary>
    /// Classifies the AABB against all 6 frustum planes extracted from the ViewProjection matrix.
    /// Uses the Gribb-Hartmann method for DirectX (row-vector, depth [0,1]).
    /// </summary>
    private static VisibilityStatus FrustumTest(Matrix vp, BoundingBox box)
    {
        // Extract frustum planes: each plane is (A, B, C, D) where Ax+By+Cz+D >= 0 = inside.
        // Planes derived from VP columns for row-vector convention.
        Span<float> planes = stackalloc float[24]; // 6 planes × 4 floats
        // Left:   clip.x + clip.w >= 0
        planes[0]  = vp.M11 + vp.M14; planes[1]  = vp.M21 + vp.M24;
        planes[2]  = vp.M31 + vp.M34; planes[3]  = vp.M41 + vp.M44;
        // Right:  -clip.x + clip.w >= 0
        planes[4]  = vp.M14 - vp.M11; planes[5]  = vp.M24 - vp.M21;
        planes[6]  = vp.M34 - vp.M31; planes[7]  = vp.M44 - vp.M41;
        // Bottom: clip.y + clip.w >= 0
        planes[8]  = vp.M12 + vp.M14; planes[9]  = vp.M22 + vp.M24;
        planes[10] = vp.M32 + vp.M34; planes[11] = vp.M42 + vp.M44;
        // Top:    -clip.y + clip.w >= 0
        planes[12] = vp.M14 - vp.M12; planes[13] = vp.M24 - vp.M22;
        planes[14] = vp.M34 - vp.M32; planes[15] = vp.M44 - vp.M42;
        // Near:   clip.z >= 0  (DirectX depth [0,1])
        planes[16] = vp.M13; planes[17] = vp.M23;
        planes[18] = vp.M33; planes[19] = vp.M43;
        // Far:    -clip.z + clip.w >= 0
        planes[20] = vp.M14 - vp.M13; planes[21] = vp.M24 - vp.M23;
        planes[22] = vp.M34 - vp.M33; planes[23] = vp.M44 - vp.M43;

        var min = box.Minimum;
        var max = box.Maximum;
        bool anyPartial = false;

        for (int p = 0; p < 6; p++)
        {
            int b = p * 4;
            float a = planes[b], bv = planes[b + 1], c = planes[b + 2], d = planes[b + 3];

            // P-vertex (most positive against plane normal).
            float px = a >= 0f ? max.X : min.X;
            float py = bv >= 0f ? max.Y : min.Y;
            float pz = c >= 0f ? max.Z : min.Z;
            if (a * px + bv * py + c * pz + d < 0f)
                return VisibilityStatus.Hidden; // Entire AABB outside this plane.

            // N-vertex (most negative).
            float nx = a >= 0f ? min.X : max.X;
            float ny = bv >= 0f ? min.Y : max.Y;
            float nz = c >= 0f ? min.Z : max.Z;
            if (a * nx + bv * ny + c * nz + d < 0f)
                anyPartial = true; // Partially inside.
        }

        return anyPartial ? VisibilityStatus.HalfCovered : VisibilityStatus.InView;
    }

    /// <summary>
    /// Projects all 8 corners of the AABB through the VP matrix and returns the NDC extents.
    /// Corners behind the near plane (w &lt;= 0) are excluded.
    /// </summary>
    private static void ProjectBoundingBoxToNdc(
        Matrix vp, BoundingBox box,
        out float minX, out float maxX,
        out float minY, out float maxY,
        out bool anyVisible)
    {
        minX = float.MaxValue; maxX = float.MinValue;
        minY = float.MaxValue; maxY = float.MinValue;
        anyVisible = false;

        var bMin = box.Minimum;
        var bMax = box.Maximum;

        // Inline corner enumeration to avoid heap allocation.
        for (int i = 0; i < 8; i++)
        {
            float cx = (i & 1) != 0 ? bMax.X : bMin.X;
            float cy = (i & 2) != 0 ? bMax.Y : bMin.Y;
            float cz = (i & 4) != 0 ? bMax.Z : bMin.Z;

            // clip = corner * VP  (row-vector)
            float clipX = cx * vp.M11 + cy * vp.M21 + cz * vp.M31 + vp.M41;
            float clipY = cx * vp.M12 + cy * vp.M22 + cz * vp.M32 + vp.M42;
            float clipW = cx * vp.M14 + cy * vp.M24 + cz * vp.M34 + vp.M44;

            if (clipW <= 0f) continue; // Behind near plane — skip.
            anyVisible = true;

            float ndcX = clipX / clipW;
            float ndcY = clipY / clipW;
            if (ndcX < minX) minX = ndcX;
            if (ndcX > maxX) maxX = ndcX;
            if (ndcY < minY) minY = ndcY;
            if (ndcY > maxY) maxY = ndcY;
        }
    }
}

