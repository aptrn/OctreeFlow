using Stride.Core.Mathematics;

namespace OctreeFlow.Api;

/// <summary>
/// Static per-node (BF = Box/node Features) buffer data.
/// Built once after Initialize() via OctreeFlowReader.BuildStaticNodeData().
///
/// LAYOUT — one entry per octree node (linear index = <see cref="NodeInfo.NodeId"/>):
///   BF_*[nodeId] describes the node with that ID. Point buffers use the same IDs in
///   <c>Point_NodeID</c>, so shaders can do <c>BF_Density[Point_NodeID[pointId]]</c>.
///
/// Naming schema: BF_DataName
///
/// BF_View is the only dynamic field — update it each frame via
/// OctreeFlowReader.UpdateNodeViewState(), then re-upload only the BF_View buffer.
/// BF_View[nodeId] = 1 when that node is in view, else 0.
/// </summary>
public class NodeBufferData
{
    private readonly Dictionary<string, Vector4[]> _vector4 = new();
    private readonly Dictionary<string, float[]>   _float32 = new();
    private readonly Dictionary<string, int[]>     _int32   = new();

    /// <summary>Number of octree nodes.</summary>
    public int NodeCount { get; }

    /// <summary>
    /// Total allocated size of all feature arrays.
    /// Equals <see cref="NodeCount"/> when no maximumSize was specified, otherwise the
    /// requested maximumSize (clamped to at least NodeCount).
    /// Slots [NodeCount..MaximumSize-1] are zero-padded.
    /// </summary>
    public int MaximumSize { get; }

    /// <param name="nodeCount">Actual number of octree nodes.</param>
    /// <param name="maximumSize">
    /// Desired GPU buffer length. When &gt; 0, arrays are allocated at this length
    /// (must be &gt;= nodeCount). Pass the same value to GetCombinedAllActiveData if you
    /// pad point buffers to match. Defaults to nodeCount when 0.
    /// </param>
    public NodeBufferData(int nodeCount, int maximumSize = 0)
    {
        NodeCount   = nodeCount;
        MaximumSize = maximumSize > 0 ? Math.Max(nodeCount, maximumSize) : nodeCount;

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
    /// Values: Vector4[] arrays of length MaximumSize, filled for indices 0..NodeCount-1.
    /// </summary>
    public IEnumerable<KeyValuePair<string, Vector4[]>> FeaturesVector4 => _vector4;

    /// <summary>
    /// BF Float32 features keyed by name.
    /// Keys: "BF_Density", "BF_Spacing".
    /// Values: float[] arrays of length MaximumSize.
    /// </summary>
    public IEnumerable<KeyValuePair<string, float[]>> FeaturesFloat32 => _float32;

    /// <summary>
    /// BF Int32 features keyed by name.
    /// Keys: "BF_NodeID", "BF_PointCount", "BF_Level", "BF_View".
    /// Values: int[] arrays of length MaximumSize.
    /// </summary>
    public IEnumerable<KeyValuePair<string, int[]>> FeaturesInt32 => _int32;

    // ── Typed convenience accessors ───────────────────────────────────────────

    /// <summary>
    /// Owning node index: BF_NodeID[i] == i for filled slots (same as <see cref="NodeInfo.NodeId"/>).
    /// </summary>
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
