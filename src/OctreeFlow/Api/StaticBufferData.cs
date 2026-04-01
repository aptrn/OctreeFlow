using Stride.Core.Mathematics;

namespace OctreeFlow.Api;

/// <summary>
/// Static per-node (BF = Box/node Features) buffer data.
/// Built once after Initialize() via OctreeFlowReader.BuildStaticNodeData().
///
/// LAYOUT — vertex granularity (NodeCount × 8 entries):
///   Each node n occupies 8 consecutive slots: indices n*8 … n*8+7.
///   At buffer index i: all BF values describe the owning node (i / 8).
///   This matches VertexBufferData exactly, so both buffers can share the same
///   dispatch index in VL.Fuse without any shader-side indirection.
///
///     BF_Position[i]  = center of node (i/8)
///     Vertex_Position[i] = corner (i%8) of node (i/8)   ← from VertexBufferData
///
/// Naming schema: BF_DataName
///
/// BF_View is the only dynamic field — update it each frame via
/// OctreeFlowReader.UpdateNodeViewState(), then re-upload only the BF_View buffer.
/// BF_View[i] = 1 only for i = n*8+0 of in-view nodes, so a single-instance-per-node
/// box render can be driven by the BF_View flag without extra shader logic.
/// </summary>
public class NodeBufferData
{
    private readonly Dictionary<string, Vector4[]> _vector4 = new();
    private readonly Dictionary<string, float[]>   _float32 = new();
    private readonly Dictionary<string, int[]>     _int32   = new();

    /// <summary>Number of octree nodes.</summary>
    public int NodeCount { get; }

    /// <summary>
    /// Number of slots in every feature array = NodeCount × 8.
    /// Use this as the GPU dispatch count so BF and Vertex buffers are index-aligned.
    /// </summary>
    public int VertexCount { get; }

    /// <summary>
    /// Total allocated size of all feature arrays.
    /// Equals VertexCount when no maximumSize was specified, otherwise equals the
    /// requested maximumSize (clamped to at least VertexCount).
    /// Slots [VertexCount..MaximumSize-1] are zero-padded.
    /// </summary>
    public int MaximumSize { get; }

    /// <param name="nodeCount">Actual number of octree nodes.</param>
    /// <param name="maximumSize">
    /// Desired GPU buffer size. When &gt; 0, arrays are allocated at this length
    /// (must be &gt;= nodeCount × 8). Pass the same value to BuildStaticVertexData
    /// so both BF and Vertex buffers share identical element counts.
    /// Defaults to nodeCount × 8 when 0.
    /// </param>
    public NodeBufferData(int nodeCount, int maximumSize = 0)
    {
        NodeCount   = nodeCount;
        VertexCount = nodeCount * 8;
        MaximumSize = maximumSize > 0 ? Math.Max(VertexCount, maximumSize) : VertexCount;

        // Vector4 features
        _vector4["BF_Position"] = new Vector4[MaximumSize];
        _vector4["BF_Size"]     = new Vector4[MaximumSize];
        _vector4["BF_Color"]    = new Vector4[MaximumSize];

        // Float32 features
        _float32["BF_Density"] = new float[MaximumSize];
        _float32["BF_Spacing"] = new float[MaximumSize];

        // Int32 features (BF_NodeID first — it is the owning node index)
        _int32["BF_NodeID"]     = new int[MaximumSize];
        _int32["BF_PointCount"] = new int[MaximumSize];
        _int32["BF_Level"]      = new int[MaximumSize];
        _int32["BF_View"]       = new int[MaximumSize];
    }

    // ── Dictionary-style access ───────────────────────────────────────────────

    /// <summary>
    /// BF Vector4 features keyed by name.
    /// Keys: "BF_Position", "BF_Size", "BF_Color".
    /// Values: Vector4[] arrays of length MaximumSize, filled at vertex granularity.
    /// </summary>
    public IEnumerable<KeyValuePair<string, Vector4[]>> FeaturesVector4 => _vector4;

    /// <summary>
    /// BF Float32 features keyed by name.
    /// Keys: "BF_Density", "BF_Spacing".
    /// Values: float[] arrays of length MaximumSize, filled at vertex granularity.
    /// </summary>
    public IEnumerable<KeyValuePair<string, float[]>> FeaturesFloat32 => _float32;

    /// <summary>
    /// BF Int32 features keyed by name.
    /// Keys: "BF_NodeID", "BF_PointCount", "BF_Level", "BF_View".
    /// Values: int[] arrays of length MaximumSize, filled at vertex granularity.
    /// </summary>
    public IEnumerable<KeyValuePair<string, int[]>> FeaturesInt32 => _int32;

    // ── Typed convenience accessors ───────────────────────────────────────────

    /// <summary>
    /// Owning node index: BF_NodeID[i] == i / 8.
    /// At vertex slot i this gives the NodeId of the node whose bounding-box corner is at that slot.
    /// </summary>
    public int[]     BF_NodeID    => _int32["BF_NodeID"];

    /// <summary>Owning node center in world space. XYZ = center, W = 1. Same value for all 8 slots of a node.</summary>
    public Vector4[] BF_Position  => _vector4["BF_Position"];

    /// <summary>Node bounding box extent. XYZ = size (max−min), W = spacing (largest dimension).</summary>
    public Vector4[] BF_Size      => _vector4["BF_Size"];

    /// <summary>Average point color inside the node. RGBA normalized [0, 1].</summary>
    public Vector4[] BF_Color     => _vector4["BF_Color"];

    /// <summary>Normalized point density in [0, 1]. 1 = densest node in the tree.</summary>
    public float[]   BF_Density   => _float32["BF_Density"];

    /// <summary>Spacing = largest bounding-box dimension of this node.</summary>
    public float[]   BF_Spacing   => _float32["BF_Spacing"];

    /// <summary>Number of points stored inside this node.</summary>
    public int[]     BF_PointCount => _int32["BF_PointCount"];

    /// <summary>Octree depth level (0 = root, increases towards leaves).</summary>
    public int[]     BF_Level     => _int32["BF_Level"];

    /// <summary>
    /// Per-frame visibility flag: 0 = not in view, 1 = actively rendered.
    /// Updated by OctreeFlowReader.UpdateNodeViewState(). Re-upload after each call.
    /// Only the first slot of each node (i = n*8+0) is set to 1; the other 7 slots remain 0.
    /// This ensures a box shader dispatching on VertexCount renders exactly one box per node.
    /// </summary>
    public int[]     BF_View      => _int32["BF_View"];
}

/// <summary>
/// Static per-vertex buffer data for all node bounding-box corners.
/// Length = TotalNodes × 8. Built once after Initialize() via
/// OctreeFlowReader.BuildStaticVertexData().
///
/// Naming schema: Vertex_DataName
///
/// Layout: 8 consecutive vertices per node, in NodeId order.
///   vertex i → node (i / 8), corner (i % 8)
///
/// Corner ordering (binary bit-encoding):
///   bit 0 → X axis (0 = min, 1 = max)
///   bit 1 → Y axis
///   bit 2 → Z axis
///
/// Index alignment: NodeBufferData is also laid out at vertex granularity (8 slots per node),
/// so BF and Vertex buffers share the same dispatch index without any shader-side indirection:
///   BF_Position[i]     = center of owning node (i/8)
///   Vertex_Position[i] = bounding-box corner (i%8) of owning node (i/8)
/// </summary>
public class VertexBufferData
{
    private readonly Dictionary<string, Vector4[]> _vector4 = new();
    private readonly Dictionary<string, float[]>   _float32 = new();
    private readonly Dictionary<string, int[]>     _int32   = new();

    /// <summary>Total number of vertices = TotalNodes × 8.</summary>
    public int VertexCount { get; }

    /// <summary>
    /// Total allocated size of all feature arrays.
    /// Equals VertexCount when no maximumSize was specified, otherwise equals the requested maximumSize.
    /// Slots [VertexCount..MaximumSize-1] are zero-padded.
    /// </summary>
    public int MaximumSize { get; }

    /// <param name="vertexCount">Actual number of vertices (= TotalNodes × 8).</param>
    /// <param name="maximumSize">
    /// Desired GPU buffer size. When &gt; 0, all arrays are allocated at this length
    /// so every buffer in the synchronized set shares the same element count.
    /// Must be &gt;= vertexCount. Defaults to vertexCount when 0.
    /// </param>
    public VertexBufferData(int vertexCount, int maximumSize = 0)
    {
        VertexCount = vertexCount;
        MaximumSize = maximumSize > 0 ? Math.Max(vertexCount, maximumSize) : vertexCount;

        // Vector4 features
        _vector4["Vertex_Position"] = new Vector4[MaximumSize];

        // Float32 features — empty for now, reserved for future per-vertex scalar data

        // Int32 features (Vertex_NodeID first — it is the Buffer Index)
        _int32["Vertex_NodeID"] = new int[MaximumSize];
        _int32["Vertex_Index"]  = new int[MaximumSize];
        _int32["Vertex_Level"]  = new int[MaximumSize];
        _int32["Vertex_ID"]     = new int[MaximumSize];
        _int32["Vertex_View"]   = new int[MaximumSize];
    }


    // ── Dictionary-style access ───────────────────────────────────────────────

    /// <summary>
    /// Per-vertex Vector4 features keyed by name.
    /// Keys: "Vertex_Position".
    /// Values: Vector4[] arrays of length VertexCount.
    /// </summary>
    public IEnumerable<KeyValuePair<string, Vector4[]>> FeaturesVector4 => _vector4;

    /// <summary>
    /// Per-vertex Float32 features keyed by name.
    /// Currently empty — reserved for future per-vertex scalar data (e.g., Vertex_*).
    /// Values: float[] arrays of length VertexCount.
    /// </summary>
    public IEnumerable<KeyValuePair<string, float[]>> FeaturesFloat32 => _float32;

    /// <summary>
    /// Per-vertex Int32 features keyed by name.
    /// Keys: "Vertex_NodeID" (cross-reference index into BF buffers),
        ///       "Vertex_Index" (0–7 corner), "Vertex_Level", "Vertex_ID", "Vertex_View".
    /// Values: int[] arrays of length VertexCount.
    /// </summary>
    public IEnumerable<KeyValuePair<string, int[]>> FeaturesInt32 => _int32;

    // ── Typed convenience accessors ───────────────────────────────────────────

    /// <summary>NodeId of the owning node (0…TotalNodes−1). Index into BF buffers.</summary>
    public int[]     Vertex_NodeID   => _int32["Vertex_NodeID"];

    /// <summary>World-space corner position. XYZ = position, W = 1.</summary>
    public Vector4[] Vertex_Position => _vector4["Vertex_Position"];

    /// <summary>Corner index 0–7. Bit0=X, Bit1=Y, Bit2=Z (0=min side, 1=max side).</summary>
    public int[]     Vertex_Index    => _int32["Vertex_Index"];

    /// <summary>Octree depth level of the owning node (0 = root).</summary>
    public int[]     Vertex_Level    => _int32["Vertex_Level"];

        /// <summary>This vertex's own sequential index in the buffer (0…VertexCount−1).</summary>
        public int[]     Vertex_ID       => _int32["Vertex_ID"];

        /// <summary>
        /// Per-frame visibility flag: 0 = owning node not in view, 1 = actively rendered.
        /// Fill manually after each traversal (e.g., set all 8 vertices of a node to the
        /// same value as BF_View[Vertex_NodeID[i]]). Re-upload only this buffer each frame.
        /// </summary>
        public int[]     Vertex_View     => _int32["Vertex_View"];
}
