using Stride.Core.Mathematics;

namespace OctreeFlow.Api;

/// <summary>
/// Static per-node (BF = Box/node Features) buffer data. Length = TotalNodes.
/// Built once after Initialize() via OctreeFlowReader.BuildStaticNodeData().
/// Each entry is indexed by NodeId (DFS sequential integer, 0 = root).
///
/// Naming schema: BF_DataName
///
/// Iterate FeaturesVector4 / FeaturesFloat32 / FeaturesInt32 exactly like
/// OctreeFlowReader.FeaturesVector4/Float32/Int32 to create your structured buffers
/// in a single ForEach loop.
///
/// Cross-buffer synchronization in GPU shaders:
///   A noise or filter computed per-node (indexed by NodeId) can be applied to both
///   vertex and point buffers via their respective index buffers:
///     float noise = PerNodeBuffer[Point_NodeID[pointId]];
///     float noise = PerNodeBuffer[Vertex_NodeID[vertexId]];
///
/// BF_View is the only dynamic field — update it each frame via
/// OctreeFlowReader.UpdateNodeViewState(), then re-upload only the BF_View buffer.
/// </summary>
public class NodeBufferData
{
    private readonly Dictionary<string, Vector4[]> _vector4 = new();
    private readonly Dictionary<string, float[]>   _float32 = new();
    private readonly Dictionary<string, int[]>     _int32   = new();

    /// <summary>Number of nodes (= TotalNodes).</summary>
    public int NodeCount { get; }

        public NodeBufferData(int nodeCount)
    {
        NodeCount = nodeCount;

        // Vector4 features
        _vector4["BF_Position"] = new Vector4[nodeCount];
        _vector4["BF_Size"]     = new Vector4[nodeCount];
        _vector4["BF_Color"]    = new Vector4[nodeCount];

        // Float32 features
        _float32["BF_Density"] = new float[nodeCount];
        _float32["BF_Spacing"] = new float[nodeCount];

        // Int32 features (BF_NodeID first — it is a Buffer Index, listed first by convention)
        _int32["BF_NodeID"]    = new int[nodeCount];
        _int32["BF_PointCount"] = new int[nodeCount];
        _int32["BF_Level"]     = new int[nodeCount];
        _int32["BF_View"]      = new int[nodeCount];
    }

    // ── Dictionary-style access ───────────────────────────────────────────────
    // Same pattern as OctreeFlowReader.FeaturesVector4/Float32/Int32.
    // Loop over these in VL to create one structured buffer per entry.

    /// <summary>
    /// Per-node Vector4 features keyed by name.
    /// Keys: "BF_Position", "BF_Size", "BF_Color".
    /// Values: Vector4[] arrays of length NodeCount.
    /// </summary>
    public IEnumerable<KeyValuePair<string, Vector4[]>> FeaturesVector4 => _vector4;

    /// <summary>
    /// Per-node Float32 features keyed by name.
    /// Keys: "BF_Density", "BF_Spacing".
    /// Values: float[] arrays of length NodeCount.
    /// </summary>
    public IEnumerable<KeyValuePair<string, float[]>> FeaturesFloat32 => _float32;

    /// <summary>
    /// Per-node Int32 features keyed by name.
    /// Keys: "BF_NodeID" (identity/index), "BF_PointCount", "BF_Level", "BF_View".
    /// Values: int[] arrays of length NodeCount.
    /// </summary>
    public IEnumerable<KeyValuePair<string, int[]>> FeaturesInt32 => _int32;

    // ── Typed convenience accessors ───────────────────────────────────────────

    /// <summary>Identity index: BF_NodeID[i] == i. Use as structured-buffer self-index in shaders.</summary>
    public int[]     BF_NodeID    => _int32["BF_NodeID"];

    /// <summary>Node center in world space. XYZ = center, W = 1.</summary>
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
/// Iterate FeaturesVector4 / FeaturesInt32 exactly like the point features to create
/// your structured buffers in a single ForEach loop.
///
/// Layout: 8 consecutive vertices per node, in NodeId order.
///   vertex i → node (i / 8), corner (i % 8)
///
/// Corner ordering (binary bit-encoding):
///   bit 0 → X axis (0 = min, 1 = max)
///   bit 1 → Y axis
///   bit 2 → Z axis
///
/// Cross-buffer synchronization:
///   Vertex_NodeID lets you look up any BF (per-node) buffer value for a vertex:
///     int   nodeIdx = Vertex_NodeID[vertexId];
///     float density = BF_Density_buffer[nodeIdx];
///     int   inView  = BF_View_buffer[nodeIdx];
/// </summary>
public class VertexBufferData
{
    private readonly Dictionary<string, Vector4[]> _vector4 = new();
    private readonly Dictionary<string, float[]>   _float32 = new();
    private readonly Dictionary<string, int[]>     _int32   = new();

    /// <summary>Total number of vertices = TotalNodes × 8.</summary>
    public int VertexCount { get; }

        public VertexBufferData(int vertexCount)
    {
        VertexCount = vertexCount;

        // Vector4 features
        _vector4["Vertex_Position"] = new Vector4[vertexCount];

        // Float32 features — empty for now, reserved for future per-vertex scalar data
        // (no entries added here yet)

        // Int32 features (Vertex_NodeID first — it is the Buffer Index)
        _int32["Vertex_NodeID"] = new int[vertexCount];
        _int32["Vertex_Index"]  = new int[vertexCount];
        _int32["Vertex_Level"]  = new int[vertexCount];
        _int32["Vertex_ID"]     = new int[vertexCount];
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
    ///       "Vertex_Index" (0–7 corner), "Vertex_Level", "Vertex_ID".
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
}
