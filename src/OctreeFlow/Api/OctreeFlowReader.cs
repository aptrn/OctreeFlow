using OctreeFlow.Core;
using OctreeFlow.Data;
using OctreeFlow.IO;
using Stride.Core.Mathematics;
using System.Diagnostics;

namespace OctreeFlow.Api;

/// <summary>
/// Main API for reading and traversing octree point cloud data.
/// Handles loading from .octree and .ply files, traversal, RAM caching, and buffer data output.
/// Designed for use with VVVV Gamma - outputs data ready for DynamicBufferAdvanced.
/// 
/// Workflow:
/// 1. Create reader with paths and settings
/// 2. Call Initialize() once
/// 3. Each frame: Call UpdateFrame() with your traversal delegate
/// 4. Use the BufferUpdateResult to upload new sectors to your DynamicBufferAdvanced buffers
/// 5. Use ActiveSectors for rendering dispatch
/// </summary>
public class OctreeFlowReader : IDisposable
{
    private readonly string _octreePath;
    private readonly string _plyPath;
    private readonly int _cacheSizeMB;
    private readonly int _maxBufferSizeMB;
    private readonly int? _pointsPerNodeOverride;

    private CacheManager? _cache;
    private SectorManager? _sectorManager;
    private OctreeNode? _root;
    private OctreeFileInfo? _fileInfo;
    private PlyIndex? _plyIndex;
    private readonly Dictionary<string, NodeInfo> _nodeInfoCache = new();
    private Dictionary<string, Vector4[]>? _featuresVector4;
    private Dictionary<string, float[]>? _featuresFloat32;
    private Dictionary<string, int[]>? _featuresInt32;
    
    private int _pointsPerNode; // Resolved from file or override (informational only)

    private int _traversalVersion;
    private bool _isInitialized;
    
    // Cached traversal result - return same object if content unchanged
    private TraversalResult? _cachedTraversalResult;
    private HashSet<string> _lastViewingNodeIds = new();
    
    // Cached frame update result - return same object if buffer state unchanged
    private FrameUpdateResult? _cachedFrameUpdateResult;
    private int _lastBufferVersion;

    #region Public Properties

    /// <summary>
    /// Root node of the octree.
    /// </summary>
    public OctreeNode? Root => _root;

    /// <summary>
    /// File information from the octree file.
    /// </summary>
    public OctreeFileInfo? FileInfo => _fileInfo;

    /// <summary>
    /// The RAM cache manager.
    /// </summary>
    public CacheManager? Cache => _cache;

    /// <summary>
    /// The sector manager for buffer data.
    /// </summary>
    public SectorManager? SectorManager => _sectorManager;

    /// <summary>
    /// Maximum buffer capacity in points (from SectorManager).
    /// </summary>
    public long MaxBufferCapacity => _sectorManager?.MaxBufferCapacityPoints ?? 0;

    /// <summary>
    /// Whether the reader has been initialized.
    /// </summary>
    public bool IsInitialized => _isInitialized;

    /// <summary>
    /// Bounding box of the entire point cloud.
    /// </summary>
    public BoundingBox Bounds => _fileInfo?.Bounds ?? new BoundingBox();

    /// <summary>
    /// Total number of points in the point cloud.
    /// </summary>
    public int TotalPoints => _fileInfo?.TotalPoints ?? 0;

    /// <summary>
    /// Total number of nodes in the octree.
    /// </summary>
    public int TotalNodes => _fileInfo?.NodeCount ?? 0;

    /// <summary>
    /// Points per node from octree file (informational only - not used for buffer sizing).
    /// With variable-size sectors, each node uses exactly the space it needs.
    /// </summary>
    public int PointsPerNode => _pointsPerNode;

    /// <summary>
    /// Source of the PointsPerNode value: "override", "file", or "auto-detected".
    /// </summary>
    public string PointsPerNodeSource => _pointsPerNodeOverride.HasValue ? "override" 
        : (_fileInfo?.PointsPerNode > 0 ? "file" : "auto-detected");

    /// <summary>
    /// Maximum buffer size per buffer in megabytes (as specified).
    /// </summary>
    public int MaxBufferSizeMB => _maxBufferSizeMB;

    /// <summary>
    /// Maximum buffer capacity in points.
    /// This is the limit - total points across all active nodes cannot exceed this.
    /// </summary>
    public long BufferCapacityPoints => (long)_maxBufferSizeMB * 1024 * 1024 / 16;

    /// <summary>
    /// Number of nodes currently in buffer.
    /// </summary>
    public int ActiveNodeCount => _sectorManager?.ActiveNodeCount ?? 0;

    /// <summary>
    /// Total points currently in buffer.
    /// </summary>
    public int CurrentPointsInBuffer => _sectorManager?.TotalPointsInBuffer ?? 0;

    /// <summary>
    /// Total buffer size in bytes for Vector4 buffers (Position, Colors, Normals).
    /// Use this to create your DynamicBufferAdvanced&lt;Vector4&gt; with the correct size.
    /// </summary>
    public long BufferSizeBytesVector4 => (long)_maxBufferSizeMB * 1024 * 1024;

    /// <summary>
    /// Total buffer size in bytes for Float32 buffers (Intensity, scalars).
    /// Use this to create your DynamicBufferAdvanced&lt;float&gt; with the correct size.
    /// (1/4 the size of Vector4 buffers since float is 4 bytes vs 16 bytes).
    /// </summary>
    public long BufferSizeBytesFloat32 => (long)_maxBufferSizeMB * 1024 * 1024 / 4;

    /// <summary>
    /// Available properties from the PLY file.
    /// </summary>
    public IReadOnlyList<PlyProperty> PlyProperties => _plyIndex?.Properties ?? Array.Empty<PlyProperty>().ToList();

    /// <summary>
    /// Point Vector4 feature names mapped to typed arrays for buffer initialization.
    /// Keys follow the Point_ prefix convention: "Point_Position", "Point_Color", "Point_Normal".
    /// Values: Vector4[] arrays sized to buffer capacity (maxBufferSizeMB).
    /// Use this to create your DynamicBufferAdvanced instances in a ForEach.
    /// </summary>
    public IEnumerable<KeyValuePair<string, Vector4[]>> PointFeaturesVector4 =>
        _featuresVector4 ?? Enumerable.Empty<KeyValuePair<string, Vector4[]>>();

    /// <summary>
    /// Point Float32 feature names mapped to typed arrays for buffer initialization.
    /// Keys follow the Point_ prefix convention: "Point_Intensity", "Point_{scalarName}", …
    /// Values: float[] arrays sized to buffer capacity.
    /// Use this to create your DynamicBufferAdvanced instances in a ForEach.
    /// </summary>
    public IEnumerable<KeyValuePair<string, float[]>> PointFeaturesFloat32 =>
        _featuresFloat32 ?? Enumerable.Empty<KeyValuePair<string, float[]>>();

    /// <summary>
    /// Point Int32 feature names mapped to typed arrays for buffer initialization.
    /// Keys follow the Point_ prefix convention:
    ///   "Point_Id"     — original PLY vertex index.
    ///   "Point_Level"  — octree depth level.
    ///   "Point_NodeID" — NodeId of the owning node (cross-reference index into BF buffers).
    /// Values: int[] arrays sized to buffer capacity.
    /// Use this to create your DynamicBufferAdvanced instances in a ForEach.
    /// </summary>
    public IEnumerable<KeyValuePair<string, int[]>> PointFeaturesInt32 =>
        _featuresInt32 ?? Enumerable.Empty<KeyValuePair<string, int[]>>();

    /// <summary>
    /// Total buffer size in bytes for Point Int32 buffers.
    /// (1/4 the size of Vector4 buffers since int is 4 bytes vs 16 bytes).
    /// </summary>
    public long BufferSizeBytesInt32 => (long)_maxBufferSizeMB * 1024 * 1024 / 4;

    /// <summary>
    /// Returns a flat registry of every feature across all three buffer classes.
    /// Key   = feature name (e.g., "Point_Position", "BF_Level", "Vertex_NodeID").
    /// Value = (BufferClass, DataType) where:
    ///   BufferClass ∈ { "Point", "BF", "Vertex" }
    ///   DataType    ∈ { "Vector", "Float", "Int" }
    ///
    /// Point features are dynamic (depend on the PLY file).
    /// BF and Vertex features are fixed and always present after Initialize().
    ///
    /// Use this in VL to drive a single ForEach that dispatches to the right buffer
    /// based on the two tag strings, instead of maintaining three separate loops.
    /// </summary>
    public IReadOnlyDictionary<string, (string BufferClass, string DataType)> GetFeatures()
    {
        EnsureInitialized();

        var dict = new Dictionary<string, (string, string)>();

        // ── Point features (dynamic — depend on PLY content) ─────────────────
        foreach (var kvp in _featuresVector4 ?? Enumerable.Empty<KeyValuePair<string, Vector4[]>>())
            dict[kvp.Key] = ("Point", "Vector");

        foreach (var kvp in _featuresFloat32 ?? Enumerable.Empty<KeyValuePair<string, float[]>>())
            dict[kvp.Key] = ("Point", "Float");

        foreach (var kvp in _featuresInt32 ?? Enumerable.Empty<KeyValuePair<string, int[]>>())
            dict[kvp.Key] = ("Point", "Int");

        // ── BF (per-node) features (fixed set — enumerate from empty instance) 
        var bf = new NodeBufferData(0);
        foreach (var kvp in bf.FeaturesVector4) dict[kvp.Key] = ("BF", "Vector");
        foreach (var kvp in bf.FeaturesFloat32) dict[kvp.Key] = ("BF", "Float");
        foreach (var kvp in bf.FeaturesInt32)   dict[kvp.Key] = ("BF", "Int");

        // ── Vertex features (fixed set — enumerate from empty instance) ───────
        var vx = new VertexBufferData(0);
        foreach (var kvp in vx.FeaturesVector4) dict[kvp.Key] = ("Vertex", "Vector");
        foreach (var kvp in vx.FeaturesFloat32) dict[kvp.Key] = ("Vertex", "Float");
        foreach (var kvp in vx.FeaturesInt32)   dict[kvp.Key] = ("Vertex", "Int");

        return dict;
    }

    // ── BF (per-node) buffer sizing ───────────────────────────────────────────

    /// <summary>
    /// Total bytes required for a per-node Vector4 structured buffer (e.g., BF_Position, BF_Size, BF_Color).
    /// Use this when creating a StructuredBuffer&lt;float4&gt; of size TotalNodes in VL.Fuse.
    /// </summary>
    public long BFBufferSizeBytesVector4 => (long)TotalNodes * 16;

    /// <summary>
    /// Total bytes required for a per-node Float32 structured buffer (e.g., BF_Density, BF_Spacing).
    /// </summary>
    public long BFBufferSizeBytesFloat32 => (long)TotalNodes * 4;

    /// <summary>
    /// Total bytes required for a per-node Int32 structured buffer (e.g., BF_NodeID, BF_Level, BF_View).
    /// </summary>
    public long BFBufferSizeBytesInt32 => (long)TotalNodes * 4;

    // ── Vertex (per node-corner) buffer sizing ────────────────────────────────

    /// <summary>
    /// Total number of octree box vertices = TotalNodes × 8.
    /// </summary>
    public int TotalVertices => TotalNodes * 8;

    /// <summary>
    /// Total bytes required for a per-vertex Vector4 structured buffer (e.g., Vertex_Position).
    /// </summary>
    public long VertexBufferSizeBytesVector4 => (long)TotalVertices * 16;

    /// <summary>
    /// Total bytes required for a per-vertex Int32 structured buffer (e.g., Vertex_NodeID, Vertex_Index, Vertex_Level, Vertex_ID).
    /// </summary>
    public long VertexBufferSizeBytesInt32 => (long)TotalVertices * 4;

    /// <summary>
    /// Checks if a Vector4 feature exists (Position, Colors, Normals).
    /// </summary>
    /// <param name="name">Feature name (e.g., "Position", "Colors", "Normals").</param>
    /// <returns>True if the feature is available.</returns>
    public bool HasFeatureVector4(string name) => _featuresVector4?.ContainsKey(name) ?? false;

    /// <summary>
    /// Checks if a Float32 feature exists (Intensity or any scalar).
    /// </summary>
    /// <param name="name">Feature name (e.g., "Intensity" or scalar property name).</param>
    /// <returns>True if the feature is available.</returns>
    public bool HasFeatureFloat32(string name) => _featuresFloat32?.ContainsKey(name) ?? false;

    /// <summary>
    /// Checks if an Int32 feature exists (Id).
    /// </summary>
    /// <param name="name">Feature name (e.g., "Id").</param>
    /// <returns>True if the feature is available.</returns>
    public bool HasFeatureInt32(string name) => _featuresInt32?.ContainsKey(name) ?? false;

    /// <summary>
    /// Checks if a custom PLY scalar feature exists by full Point_-prefixed name.
    /// Standard features (Point_Intensity) are excluded.
    /// </summary>
    /// <param name="name">Full feature name, e.g. "Point_classification".</param>
    /// <returns>True if the scalar feature is available.</returns>
    public bool HasScalarFeature(string name)
    {
        if (_featuresFloat32 == null) return false;
        return name != "Point_Intensity" && _featuresFloat32.ContainsKey(name);
    }

    /// <summary>
    /// Names of all custom PLY scalar features (Float32, excluding Point_Intensity).
    /// All names carry the "Point_" prefix (e.g., "Point_classification").
    /// </summary>
    public IEnumerable<string> ScalarFeatureNames =>
        _featuresFloat32?.Keys.Where(k => k != "Point_Intensity") ?? Enumerable.Empty<string>();

    /// <summary>
    /// Gets the buffer array for a specific Vector4 feature.
    /// Use this to get the correctly-sized array for a specific feature.
    /// </summary>
    /// <param name="name">Feature name (e.g., "Position", "Colors", "Normals").</param>
    /// <returns>The Vector4 array, or null if not available.</returns>
    public Vector4[]? GetFeatureVector4(string name)
    {
        if (_featuresVector4 == null) return null;
        return _featuresVector4.TryGetValue(name, out var arr) ? arr : null;
    }

    /// <summary>
    /// Gets the buffer array for a specific Float32 feature.
    /// Use this to get the correctly-sized array for a specific feature (Intensity or scalar).
    /// </summary>
    /// <param name="name">Feature name (e.g., "Intensity" or scalar property name).</param>
    /// <returns>The float array, or null if not available.</returns>
    public float[]? GetFeatureFloat32(string name)
    {
        if (_featuresFloat32 == null) return null;
        return _featuresFloat32.TryGetValue(name, out var arr) ? arr : null;
    }

    /// <summary>
    /// Gets the buffer array for a specific Int32 feature.
    /// Use this to get the correctly-sized array for a specific feature (e.g., Id).
    /// </summary>
    /// <param name="name">Feature name (e.g., "Id").</param>
    /// <returns>The int array, or null if not available.</returns>
    public int[]? GetFeatureInt32(string name)
    {
        if (_featuresInt32 == null) return null;
        return _featuresInt32.TryGetValue(name, out var arr) ? arr : null;
    }

    /// <summary>
    /// Gets the buffer array for a specific scalar feature.
    /// Alias for GetFeatureFloat32 but more semantic for scalar usage.
    /// </summary>
    /// <param name="name">Scalar property name from the PLY file.</param>
    /// <returns>The float array, or null if not available.</returns>
    public float[]? GetScalarData(string name) => GetFeatureFloat32(name);

    #endregion

    /// <summary>
    /// Creates a new OctreeFlowReader.
    /// </summary>
    /// <param name="octreePath">Path to the .octree file.</param>
    /// <param name="plyPath">Path to the .ply file.</param>
    /// <param name="cacheSizeMB">RAM cache size in megabytes.</param>
    /// <param name="maxBufferSizeMB">Maximum size per buffer in megabytes (e.g., 512, 1024, 2048). Determines how many nodes can be active simultaneously.</param>
    /// <param name="pointsPerNodeOverride">Optional override for points per node. If null, reads from octree file (or uses default 1000 for legacy files).</param>
    public OctreeFlowReader(
        string octreePath,
        string plyPath,
        int cacheSizeMB = 512,
        int maxBufferSizeMB = 512,
        int? pointsPerNodeOverride = null)
    {
        _octreePath = octreePath;
        _plyPath = plyPath;
        _cacheSizeMB = cacheSizeMB;
        _maxBufferSizeMB = maxBufferSizeMB;
        _pointsPerNodeOverride = pointsPerNodeOverride;
    }

    /// <summary>
    /// Creates and initializes a new OctreeFlowReader.
    /// </summary>
    public static OctreeFlowReader Create(
        string octreePath,
        string plyPath,
        int cacheSizeMB = 512,
        int maxBufferSizeMB = 512,
        int? pointsPerNodeOverride = null)
    {
        var reader = new OctreeFlowReader(octreePath, plyPath, cacheSizeMB, maxBufferSizeMB, pointsPerNodeOverride);
        reader.Initialize();
        return reader;
    }

    /// <summary>
    /// Initializes the reader by loading the octree structure and building the PLY index.
    /// </summary>
    public void Initialize()
    {
        if (_isInitialized) return;

        // Load octree structure
        var serializer = new StreamingOctreeSerializer();
        var (root, info) = serializer.LoadOctreeFile(_octreePath);

        _root = root ?? throw new InvalidOperationException("Failed to load octree file");
        _fileInfo = info;

        // Resolve points per node: override > file > auto-detect from structure
        if (_pointsPerNodeOverride.HasValue)
        {
            _pointsPerNode = _pointsPerNodeOverride.Value;
        }
        else if (info.PointsPerNode > 0)
        {
            _pointsPerNode = info.PointsPerNode;
        }
        else
        {
            // Legacy file without PointsPerNode - auto-detect from octree structure
            // Find the maximum point count in any node
            _pointsPerNode = GetMaxPointsInAnyNode(_root);
            if (_pointsPerNode <= 0) _pointsPerNode = 1000; // Fallback
        }

        // Calculate max buffer capacity in points
        // Using Vector4 size (16 bytes) as the reference since it's the largest element type
        long bufferSizeBytes = (long)_maxBufferSizeMB * 1024 * 1024;
        long maxBufferCapacityPoints = bufferSizeBytes / 16; // 16 bytes per Vector4

        // Build PLY index (lightweight - just parse header)
        _plyIndex = new PlyIndex(_plyPath);
        _plyIndex.BuildIndexHeaderOnly();

        // Build node info cache
        BuildNodeInfoCache(_root);

        // Create RAM cache
        _cache = new CacheManager(_cacheSizeMB);

        // Create sector manager with variable-size sectors
        // Just pass the max capacity in points - no fixed sector size needed!
        _sectorManager = new SectorManager(_cache, maxBufferCapacityPoints);

        // Build feature info dictionaries with correctly sized arrays
        BuildFeatureInfo((int)maxBufferCapacityPoints);

        // Tell SectorManager which features are available (so it only creates matching data)
        _sectorManager.SetAvailableFeatures(
            _featuresVector4?.Keys ?? Enumerable.Empty<string>(),
            _featuresFloat32?.Keys ?? Enumerable.Empty<string>(),
            _featuresInt32?.Keys ?? Enumerable.Empty<string>());

        _isInitialized = true;
    }

    /// <summary>
    /// Builds the feature info dictionaries from PLY properties.
    /// All keys use the Point_ prefix (e.g., "Point_Position", "Point_Color").
    /// Populates FeaturesVector4, FeaturesFloat32, and FeaturesInt32 with correctly-sized arrays
    /// for buffer initialization and DynamicBufferAdvanced creation in VL.
    /// </summary>
    /// <param name="bufferCapacity">Total buffer capacity in points (used to size the arrays).</param>
    private void BuildFeatureInfo(int bufferCapacity)
    {
        _featuresVector4 = new Dictionary<string, Vector4[]>();
        _featuresFloat32 = new Dictionary<string, float[]>();
        _featuresInt32 = new Dictionary<string, int[]>();

        if (_plyIndex == null) return;

        bool hasPosition = false;
        bool hasColors = false;
        bool hasNormals = false;
        bool hasIntensity = false;

        foreach (var prop in _plyIndex.Properties)
        {
            var name = prop.Name.ToLower();

            if (name == "x" || name == "y" || name == "z")
                hasPosition = true;
            else if (name == "red" || name == "r" || name == "green" || name == "g" ||
                     name == "blue" || name == "b" || name == "alpha" || name == "a")
                hasColors = true;
            else if (name == "nx" || name == "ny" || name == "nz")
                hasNormals = true;
            else if (name == "intensity" || name == "scalar_intensity")
                hasIntensity = true;
            else
            {
                // Custom PLY scalar → "Point_{lowercaseName}"
                // ConvertToPointData stores scalars with the lowercase name, so we match here.
                _featuresFloat32["Point_" + name] = new float[bufferCapacity];
            }
        }

        // Standard Vector4 point features
        if (hasPosition)  _featuresVector4["Point_Position"] = new Vector4[bufferCapacity];
        if (hasColors)    _featuresVector4["Point_Color"]    = new Vector4[bufferCapacity];
        if (hasNormals)   _featuresVector4["Point_Normal"]   = new Vector4[bufferCapacity];

        // Standard Float32 point features
        if (hasIntensity) _featuresFloat32["Point_Intensity"] = new float[bufferCapacity];

        // Standard Int32 point features
        _featuresInt32["Point_Id"]     = new int[bufferCapacity];
        _featuresInt32["Point_Level"]  = new int[bufferCapacity];

        // Cross-buffer index: for each active point, the NodeId of its owning node.
        // Shaders use this to look up BF (per-node) buffer values:
        //   float density = BF_Density[Point_NodeID[pointId]];
        _featuresInt32["Point_NodeID"] = new int[bufferCapacity];
    }

    // ── Static buffer builders (call after Initialize) ────────────────────────

    /// <summary>
    /// Builds the static per-node (BF) buffer data. Call once after Initialize().
    /// Returns one entry per octree node, indexed by NodeId (DFS sequential, 0 = root).
    ///
    /// The result is the CPU-side data you should upload to your per-node structured buffers
    /// in VL.Fuse (BF_Position, BF_Size, BF_Color, BF_Density, BF_Spacing, BF_NodeID,
    /// BF_PointCount, BF_Level, BF_View).
    ///
    /// BF_View is initialised to 0; call UpdateNodeViewState() every frame to refresh it.
    /// </summary>
    public NodeBufferData BuildStaticNodeData()
    {
        EnsureInitialized();

        var nodes = _nodeInfoCache.Values.OrderBy(n => n.NodeId).ToArray();
        int count = nodes.Length;

        var data = new NodeBufferData(count);

        for (int i = 0; i < count; i++)
        {
            var node = nodes[i];
            data.BF_NodeID[i]      = node.NodeId; // identity: always equals i
            data.BF_Position[i]    = new Vector4(node.Center.X, node.Center.Y, node.Center.Z, 1f);
            data.BF_Size[i]        = new Vector4(node.Size.X, node.Size.Y, node.Size.Z, node.Spacing);
            var c = node.AverageColor;
            data.BF_Color[i]       = new Vector4(c.R, c.G, c.B, c.A);
            data.BF_Density[i]     = node.PointDensity;
            data.BF_Spacing[i]     = node.Spacing;
            data.BF_PointCount[i]  = node.PointCount;
            data.BF_Level[i]       = node.Level;
            // BF_View initialised to 0 by NodeBufferData constructor
        }

        return data;
    }

    /// <summary>
    /// Builds the static per-vertex buffer data for all octree bounding-box corners.
    /// Call once after Initialize(). Returns TotalNodes × 8 entries.
    ///
    /// Layout: vertices are stored 8 consecutive entries per node in NodeId order.
    ///   vertex i → node (i / 8), corner (i % 8).
    ///
    /// Upload the arrays to your per-vertex structured buffers in VL.Fuse
    /// (Vertex_NodeID, Vertex_Position, Vertex_Index, Vertex_Level, Vertex_ID).
    /// </summary>
    public VertexBufferData BuildStaticVertexData()
    {
        EnsureInitialized();

        var nodes = _nodeInfoCache.Values.OrderBy(n => n.NodeId).ToArray();
        int nodeCount   = nodes.Length;
        int vertexCount = nodeCount * 8;

        var data = new VertexBufferData(vertexCount);

        for (int n = 0; n < nodeCount; n++)
        {
            var node = nodes[n];
            var bb = node.BoundingBox;
            float minX = bb.Minimum.X, minY = bb.Minimum.Y, minZ = bb.Minimum.Z;
            float maxX = bb.Maximum.X, maxY = bb.Maximum.Y, maxZ = bb.Maximum.Z;

            for (int c = 0; c < 8; c++)
            {
                int vi = n * 8 + c;
                data.Vertex_NodeID[vi]   = node.NodeId;
                data.Vertex_Position[vi] = new Vector4(
                    (c & 1) != 0 ? maxX : minX,
                    (c & 2) != 0 ? maxY : minY,
                    (c & 4) != 0 ? maxZ : minZ,
                    1f);
                data.Vertex_Index[vi]    = c;
                data.Vertex_Level[vi]    = node.Level;
                data.Vertex_ID[vi]       = vi;
            }
        }

        return data;
    }

    /// <summary>
    /// Updates the BF_View array in an existing NodeBufferData to reflect the current traversal.
    /// Call this every frame after Traverse() / UpdateFrame() to keep BF_View in sync.
    ///
    /// Sets BF_View[NodeId] = 1 for every node in viewingNodes, 0 for all others.
    /// Re-upload nodeData.BF_View to your GPU buffer after calling this.
    /// </summary>
    /// <param name="viewingNodes">Nodes currently selected for display (TraversalResult.ViewingNodes).</param>
    /// <param name="nodeData">The NodeBufferData previously returned by BuildStaticNodeData().</param>
    public void UpdateNodeViewState(IEnumerable<NodeInfo> viewingNodes, NodeBufferData nodeData)
    {
        Array.Clear(nodeData.BF_View, 0, nodeData.NodeCount);
        foreach (var node in viewingNodes)
        {
            if ((uint)node.NodeId < (uint)nodeData.NodeCount)
                nodeData.BF_View[node.NodeId] = 1;
        }
    }

    /// <summary>
    /// Initializes the reader asynchronously.
    /// </summary>
    public async Task InitializeAsync(Action<string, int, int>? onProgress = null)
    {
        if (_isInitialized) return;

        await Task.Run(() =>
        {
            onProgress?.Invoke("Loading octree...", 0, 100);
            Initialize();
            onProgress?.Invoke("Ready", 100, 100);
        });
    }

    private int _nextNodeIntId;

    private void BuildNodeInfoCache(OctreeNode node)
    {
        node.IntId = _nextNodeIntId++;
        var info = new NodeInfo(node);
        _nodeInfoCache[node.Id] = info;

        foreach (var child in node.Children)
        {
            BuildNodeInfoCache(child);
        }
    }

    /// <summary>
    /// Finds the maximum number of points in any node of the octree.
    /// Used to auto-detect pointsPerNode for legacy files.
    /// </summary>
    private int GetMaxPointsInAnyNode(OctreeNode node)
    {
        int max = node.PointIndices?.Count ?? 0;
        foreach (var child in node.Children)
        {
            max = Math.Max(max, GetMaxPointsInAnyNode(child));
        }
        return max;
    }

    /// <summary>
    /// Gets the NodeInfo for a node by ID.
    /// </summary>
    public NodeInfo? GetNodeInfo(string nodeId)
    {
        return _nodeInfoCache.TryGetValue(nodeId, out var info) ? info : null;
    }

    /// <summary>
    /// Gets statistics about the octree structure per level.
    /// Useful for debugging to see how points are distributed.
    /// </summary>
    /// <returns>Dictionary where key is level and value is (nodeCount, totalPoints).</returns>
    public Dictionary<int, (int NodeCount, int TotalPoints)> GetLevelStats()
    {
        EnsureInitialized();
        
        var stats = new Dictionary<int, (int NodeCount, int TotalPoints)>();
        
        foreach (var nodeInfo in _nodeInfoCache.Values)
        {
            if (!stats.ContainsKey(nodeInfo.Level))
            {
                stats[nodeInfo.Level] = (0, 0);
            }
            
            var current = stats[nodeInfo.Level];
            stats[nodeInfo.Level] = (current.NodeCount + 1, current.TotalPoints + nodeInfo.PointCount);
        }
        
        return stats;
    }

    /// <summary>
    /// Gets all NodeInfo objects at a specific level.
    /// </summary>
    public IEnumerable<NodeInfo> GetNodesAtLevel(int level)
    {
        EnsureInitialized();
        return _nodeInfoCache.Values.Where(n => n.Level == level);
    }

    /// <summary>
    /// Gets the maximum depth of the octree.
    /// </summary>
    public int MaxDepth => _nodeInfoCache.Values.Max(n => n.Level);

    #region Simple Traversal Methods (No Delegate Required)

    /// <summary>
    /// Traverses and selects all nodes at exactly the specified level.
    /// Easy to use without regions - just pass the level number.
    /// </summary>
    /// <param name="targetLevel">The level to select (0 = root, 1 = first children, etc.)</param>
    /// <returns>Result containing nodes at the target level.</returns>
    public TraversalResult TraverseToLevel(int targetLevel)
    {
        return Traverse(node =>
        {
            if (node.Level == targetLevel)
                return TraversalDecision.DisplayAndStop;
            else if (node.Level < targetLevel)
                return TraversalDecision.SkipButContinue;
            else
                return TraversalDecision.Reject;
        });
    }

    /// <summary>
    /// Traverses and selects all nodes from root up to and including the specified level.
    /// </summary>
    /// <param name="maxLevel">The maximum level to include (0 = root only, 1 = root + first children, etc.)</param>
    /// <returns>Result containing nodes up to maxLevel.</returns>
    public TraversalResult TraverseUpToLevel(int maxLevel)
    {
        return Traverse(node =>
        {
            if (node.Level <= maxLevel)
            {
                bool continueDeeper = node.Level < maxLevel && !node.IsLeaf;
                return new TraversalDecision(true, true, continueDeeper);
            }
            return TraversalDecision.Reject;
        });
    }

    /// <summary>
    /// Traverses and selects nodes until reaching approximately the target point count.
    /// Prioritizes coarser levels (lower detail) first.
    /// </summary>
    /// <param name="targetPointCount">Approximate maximum number of points to select.</param>
    /// <returns>Result containing nodes up to the point budget.</returns>
    public TraversalResult TraverseByPointBudget(int targetPointCount)
    {
        int currentPoints = 0;
        
        return Traverse(node =>
        {
            if (currentPoints >= targetPointCount)
                return TraversalDecision.Reject;

            currentPoints += node.PointCount;
            
            bool continueDeeper = currentPoints < targetPointCount && !node.IsLeaf;
            return new TraversalDecision(true, true, continueDeeper);
        });
    }

    /// <summary>
    /// Traverses and selects all leaf nodes (nodes with no children).
    /// This gives the highest detail available.
    /// </summary>
    /// <returns>Result containing all leaf nodes.</returns>
    public TraversalResult TraverseLeaves()
    {
        return Traverse(node =>
        {
            if (node.IsLeaf)
                return TraversalDecision.DisplayAndStop;
            else
                return TraversalDecision.SkipButContinue;
        });
    }

    /// <summary>
    /// Traverses and selects nodes that intersect with the given bounding box.
    /// Only includes nodes whose bounds overlap with the view bounds.
    /// </summary>
    /// <param name="viewBounds">The bounding box to test intersection with.</param>
    /// <param name="maxLevel">Maximum level to traverse to (-1 for unlimited).</param>
    /// <returns>Result containing nodes that intersect the view bounds.</returns>
    public TraversalResult TraverseByBounds(BoundingBox viewBounds, int maxLevel = -1)
    {
        return Traverse(node =>
        {
            // Check if node intersects view bounds
            bool intersects = node.BoundingBox.Intersects(ref viewBounds);
            
            if (!intersects)
                return TraversalDecision.Reject;

            // Check level limit
            if (maxLevel >= 0 && node.Level >= maxLevel)
                return TraversalDecision.DisplayAndStop;

            // Node intersects - include it and continue to children
            if (node.IsLeaf)
                return TraversalDecision.DisplayAndStop;
            else
                return TraversalDecision.DisplayAndContinue;
        });
    }

    /// <summary>
    /// Traverses and selects nodes based on distance from a camera position.
    /// Closer nodes get more detail (deeper levels), farther nodes get less detail.
    /// </summary>
    /// <param name="cameraPosition">The camera/viewer position.</param>
    /// <param name="detailBias">Higher values = more detail. Default 1.0. Range: 0.1 to 10.0</param>
    /// <param name="maxPoints">Maximum points to select (0 = unlimited).</param>
    /// <returns>Result containing nodes selected by distance-based LOD.</returns>
    public TraversalResult TraverseByDistance(Vector3 cameraPosition, float detailBias = 1.0f, int maxPoints = 0)
    {
        int currentPoints = 0;
        
        return Traverse(node =>
        {
            // Check point budget
            if (maxPoints > 0 && currentPoints >= maxPoints)
                return TraversalDecision.Reject;

            // Calculate distance from camera to node center
            float distance = Vector3.Distance(cameraPosition, node.Center);
            
            // Calculate node size (use largest dimension)
            float nodeSize = Math.Max(Math.Max(node.Size.X, node.Size.Y), node.Size.Z);
            
            // Screen-space size heuristic: larger nodes or closer nodes should subdivide
            // screenSize approximates how big the node appears on screen
            float screenSize = (nodeSize / Math.Max(distance, 0.001f)) * detailBias;
            
            // Threshold for subdivision (tune this value)
            float subdivideThreshold = 0.1f;
            
            currentPoints += node.PointCount;

            if (node.IsLeaf || screenSize < subdivideThreshold)
                return TraversalDecision.DisplayAndStop;
            else
                return TraversalDecision.DisplayAndContinue;
        });
    }

    #endregion

    /// <summary>
    /// Traverses the octree using the provided function.
    /// This appears as a regular node input in vvvv gamma (not a region).
    /// Pass a Func&lt;NodeInfo, TraversalDecision&gt; that decides how to handle each node.
    /// 
    /// IMPORTANT: Returns the SAME TraversalResult object if the viewing nodes haven't changed.
    /// This prevents unnecessary downstream updates in vvvv gamma.
    /// </summary>
    /// <param name="traversalFunction">Function that takes NodeInfo and returns TraversalDecision.</param>
    /// <returns>Result containing caching and viewing node lists. Same object if unchanged.</returns>
    public TraversalResult Traverse(Func<NodeInfo, TraversalDecision> traversalFunction)
    {
        EnsureInitialized();

        var sw = Stopwatch.StartNew();
        var result = new TraversalResult();

        if (_root == null)
        {
            result.IsComplete = true;
            result.TraversalTimeMs = sw.ElapsedMilliseconds;
            result.Version = _traversalVersion;
            return _cachedTraversalResult ?? result;
        }

        TraverseNode(_root, traversalFunction, result);

        sw.Stop();
        result.TraversalTimeMs = sw.ElapsedMilliseconds;
        result.IsComplete = true;

        // Check if viewing nodes changed
        var currentNodeIds = new HashSet<string>(result.ViewingNodes.Select(n => n.Id));
        
        if (_cachedTraversalResult != null && currentNodeIds.SetEquals(_lastViewingNodeIds))
        {
            // Content unchanged - return the SAME object (no "Changed" trigger in vvvv)
            return _cachedTraversalResult;
        }

        // Content changed - increment version and cache new result
        result.Version = Interlocked.Increment(ref _traversalVersion);
        _lastViewingNodeIds = currentNodeIds;
        _cachedTraversalResult = result;

        return result;
    }

    private void TraverseNode(OctreeNode node, Func<NodeInfo, TraversalDecision> func, TraversalResult result)
    {
        result.NodesVisited++;

        // Get or create NodeInfo
        if (!_nodeInfoCache.TryGetValue(node.Id, out var nodeInfo))
        {
            nodeInfo = new NodeInfo(node);
            _nodeInfoCache[node.Id] = nodeInfo;
        }

        // Update status from cache and sector manager
        nodeInfo.UpdateStatus(
            _cache?.Contains(node.Id) ?? false,
            _sectorManager?.Contains(node.Id) ?? false,
            _sectorManager?.GetSectorFor(node.Id) ?? -1
        );

        // Call function
        var decision = func(nodeInfo);

        if (decision.IsAccepted)
        {
            result.NodesAccepted++;
            result.CachingNodes.Add(nodeInfo);

            if (decision.IsForDisplay)
            {
                result.ViewingNodes.Add(nodeInfo);
            }
        }

        // Continue to children if requested
        if (decision.ContinueToChildren)
        {
            foreach (var child in node.Children)
            {
                TraverseNode(child, func, result);
            }
        }
    }

    /// <summary>
    /// Empties the RAM cache. Use this to free memory or force a full reload on the next update.
    /// Cached traversal and frame results are invalidated so the next call reflects correct cache status.
    /// </summary>
    public void ClearCache()
    {
        _cache?.Clear();
        _cachedTraversalResult = null;
        _cachedFrameUpdateResult = null;
    }

    /// <summary>
    /// Loads specified nodes into RAM cache.
    /// </summary>
    /// <param name="nodes">Nodes to load into cache.</param>
    /// <returns>Result of the cache loading operation.</returns>
    public CacheLoadResult LoadToCache(IEnumerable<NodeInfo> nodes)
    {
        EnsureInitialized();

        var sw = Stopwatch.StartNew();
        var result = new CacheLoadResult();
        var nodesToLoad = nodes.Where(n => !_cache!.Contains(n.Id)).ToList();

        try
        {
            foreach (var nodeInfo in nodesToLoad)
            {
                var indices = nodeInfo.Node.PointIndices?.ToArray();
                if (indices != null && indices.Length > 0)
                {
                    var pointData = ReadPointData(indices);
                    _cache!.Add(nodeInfo.Id, indices, pointData);
                    result.LoadedNodes[nodeInfo.Id] = indices;
                }
            }

            result.Version = _cache!.Version;
            result.IsComplete = true;
        }
        catch (Exception ex)
        {
            result.Error = ex.Message;
            result.IsComplete = false;
        }

        sw.Stop();
        result.LoadTimeMs = sw.ElapsedMilliseconds;
        return result;
    }

    /// <summary>
    /// Loads specified nodes into RAM cache asynchronously.
    /// </summary>
    public async Task<CacheLoadResult> LoadToCacheAsync(
        IEnumerable<NodeInfo> nodes,
        CancellationToken cancellationToken = default)
    {
        EnsureInitialized();

        var sw = Stopwatch.StartNew();
        var result = new CacheLoadResult();
        var nodesToLoad = nodes.Where(n => !_cache!.Contains(n.Id)).ToList();

        try
        {
            await Task.Run(() =>
            {
                foreach (var nodeInfo in nodesToLoad)
                {
                    if (cancellationToken.IsCancellationRequested) break;

                    var indices = nodeInfo.Node.PointIndices?.ToArray();
                    if (indices != null && indices.Length > 0)
                    {
                        var pointData = ReadPointData(indices);
                        _cache!.Add(nodeInfo.Id, indices, pointData);
                        result.LoadedNodes[nodeInfo.Id] = indices;
                    }
                }
            }, cancellationToken);

            result.Version = _cache!.Version;
            result.IsComplete = !cancellationToken.IsCancellationRequested;
        }
        catch (OperationCanceledException)
        {
            result.IsComplete = false;
        }
        catch (Exception ex)
        {
            result.Error = ex.Message;
            result.IsComplete = false;
        }

        sw.Stop();
        result.LoadTimeMs = sw.ElapsedMilliseconds;
        return result;
    }

    /// <summary>
    /// Updates buffer with the given viewing nodes.
    /// Loads nodes to cache if needed, then updates sector manager.
    /// Use this after calling Traverse() separately.
    /// </summary>
    /// <param name="viewingNodes">Nodes to display (from TraversalResult.ViewingNodes).</param>
    /// <returns>Buffer update result with new sectors and active sectors.</returns>
    public BufferUpdateResult UpdateBuffer(IEnumerable<NodeInfo> viewingNodes)
    {
        EnsureInitialized();

        var nodeList = viewingNodes.ToList();

        // Load viewing nodes to cache (sync)
        var notInCache = nodeList.Where(n => !_cache!.Contains(n.Id)).ToList();

        if (notInCache.Count > 0)
        {
            LoadToCache(notInCache);
        }

        // Update sector manager - get buffer data to upload
        return _sectorManager!.Update(nodeList);
    }

    /// <summary>
    /// Updates buffer with the given viewing nodes asynchronously.
    /// Loads nodes to cache if needed, then updates sector manager.
    /// Use this after calling Traverse() separately.
    /// </summary>
    /// <param name="viewingNodes">Nodes to display (from TraversalResult.ViewingNodes).</param>
    /// <param name="cancellationToken">Cancellation token.</param>
    /// <returns>Buffer update result with new sectors and active sectors.</returns>
    public async Task<BufferUpdateResult> UpdateBufferAsync(
        IEnumerable<NodeInfo> viewingNodes,
        CancellationToken cancellationToken = default)
    {
        EnsureInitialized();

        var nodeList = viewingNodes.ToList();

        // Load viewing nodes to cache (async)
        var notInCache = nodeList.Where(n => !_cache!.Contains(n.Id)).ToList();

        if (notInCache.Count > 0)
        {
            await LoadToCacheAsync(notInCache, cancellationToken);
        }

        // Update sector manager - get buffer data to upload
        return _sectorManager!.Update(nodeList);
    }

    /// <summary>
    /// Performs a complete frame update: Traverse → Cache → Buffer data output.
    /// Convenience method that combines Traverse() and UpdateBuffer().
    /// 
    /// IMPORTANT: Returns the SAME FrameUpdateResult object if nothing changed.
    /// This prevents unnecessary downstream updates in vvvv gamma.
    /// </summary>
    /// <param name="traversalResult">Result from a previous Traverse() call.</param>
    /// <returns>Complete frame result with buffer data to upload. Same object if unchanged.</returns>
    public FrameUpdateResult UpdateFrame(TraversalResult traversalResult)
    {
        EnsureInitialized();

        var sw = Stopwatch.StartNew();
        var result = new FrameUpdateResult
        {
            Traversal = traversalResult
        };

        // Load viewing nodes to cache (sync)
        var notInCache = traversalResult.ViewingNodes
            .Where(n => !_cache!.Contains(n.Id))
            .ToList();

        if (notInCache.Count > 0)
        {
            result.CacheResult = LoadToCache(notInCache);
        }
        else
        {
            result.CacheResult = new CacheLoadResult { IsComplete = true, Version = _cache!.Version };
        }

        // Update sector manager - get buffer data to upload
        result.BufferUpdate = _sectorManager!.Update(traversalResult.ViewingNodes);

        sw.Stop();
        result.TotalTimeMs = sw.ElapsedMilliseconds;

        // Check if buffer state changed
        int currentBufferVersion = result.BufferUpdate.Version;
        
        if (_cachedFrameUpdateResult != null && 
            currentBufferVersion == _lastBufferVersion && 
            !result.BufferUpdate.HasNewData)
        {
            // Buffer state unchanged - return the SAME object (no "Changed" trigger in vvvv)
            return _cachedFrameUpdateResult;
        }

        // Buffer state changed - cache new result
        _lastBufferVersion = currentBufferVersion;
        _cachedFrameUpdateResult = result;

        return result;
    }

    /// <summary>
    /// Async version of UpdateFrame.
    /// </summary>
    /// <param name="traversalResult">Result from a previous Traverse() call.</param>
    /// <param name="cancellationToken">Cancellation token.</param>
    /// <returns>Complete frame result with buffer data to upload.</returns>
    public async Task<FrameUpdateResult> UpdateFrameAsync(
        TraversalResult traversalResult,
        CancellationToken cancellationToken = default)
    {
        EnsureInitialized();

        var sw = Stopwatch.StartNew();
        var result = new FrameUpdateResult
        {
            Traversal = traversalResult
        };

        // Load viewing nodes to cache (async)
        var notInCache = traversalResult.ViewingNodes
            .Where(n => !_cache!.Contains(n.Id))
            .ToList();

        if (notInCache.Count > 0)
        {
            result.CacheResult = await LoadToCacheAsync(notInCache, cancellationToken);
        }
        else
        {
            result.CacheResult = new CacheLoadResult { IsComplete = true, Version = _cache!.Version };
        }

        // Update sector manager - get buffer data to upload
        result.BufferUpdate = _sectorManager!.Update(traversalResult.ViewingNodes);

        sw.Stop();
        result.TotalTimeMs = sw.ElapsedMilliseconds;

        return result;
    }

    /// <summary>
    /// Reads point data for the given indices from the PLY file.
    /// Each point's Id is set to its original index in the PLY file.
    /// </summary>
    private PointData[] ReadPointData(int[] indices)
    {
        if (_plyIndex == null) return Array.Empty<PointData>();

        var result = new PointData[indices.Length];

        // For binary PLY, we can do random access
        if (_plyIndex.Format != PlyFormat.Ascii)
        {
            for (int i = 0; i < indices.Length; i++)
            {
                var values = _plyIndex.ReadVertex(indices[i]);
                result[i] = ConvertToPointData(values);
                result[i].Id = indices[i]; // Set Id to original PLY index
            }
        }
        else
        {
            // For ASCII, we need to stream (less efficient for random access)
            var sortedIndices = indices.Select((idx, i) => (idx, i)).OrderBy(x => x.idx).ToArray();
            int currentIndex = 0;

            _plyIndex.StreamVertices((vertexIndex, pos, values) =>
            {
                while (currentIndex < sortedIndices.Length && sortedIndices[currentIndex].idx == vertexIndex)
                {
                    result[sortedIndices[currentIndex].i] = ConvertToPointData(values);
                    result[sortedIndices[currentIndex].i].Id = vertexIndex; // Set Id to original PLY index
                    currentIndex++;
                }
            });
        }

        return result;
    }

    private PointData ConvertToPointData(float[] values)
    {
        if (_plyIndex == null) return new PointData();

        var point = new PointData();
        var props = _plyIndex.Properties;

        for (int i = 0; i < props.Count && i < values.Length; i++)
        {
            var name = props[i].Name.ToLower();
            var val = values[i];
            var type = props[i].Type;

            switch (name)
            {
                case "x": point.Position.X = val; break;
                case "y": point.Position.Y = val; break;
                case "z": point.Position.Z = val; break;
                case "red" or "r":
                    point.Color = new Color4(NormalizeColorChannel(val, type), point.Color.G, point.Color.B, point.Color.A);
                    break;
                case "green" or "g":
                    point.Color = new Color4(point.Color.R, NormalizeColorChannel(val, type), point.Color.B, point.Color.A);
                    break;
                case "blue" or "b":
                    point.Color = new Color4(point.Color.R, point.Color.G, NormalizeColorChannel(val, type), point.Color.A);
                    break;
                case "alpha" or "a":
                    point.Color = new Color4(point.Color.R, point.Color.G, point.Color.B, NormalizeColorChannel(val, type));
                    break;
                case "nx": point.Normal.X = val; break;
                case "ny": point.Normal.Y = val; break;
                case "nz": point.Normal.Z = val; break;
                case "intensity" or "scalar_intensity":
                    point.Intensity = NormalizeIntensity(val, type);
                    break;
                default:
                    // Store as scalar
                    point.SetScalar(name, val);
                    break;
            }
        }

        return point;
    }

    /// <summary>
    /// Normalizes a color channel value to 0-1 range based on the actual PLY data type.
    /// UInt8 (0-255), UInt16 (0-65535), Float (assumed 0-1 or uses heuristic).
    /// </summary>
    private static float NormalizeColorChannel(float rawValue, PlyDataType type)
    {
        return type switch
        {
            PlyDataType.UInt8 or PlyDataType.Int8 => rawValue / 255f,
            PlyDataType.UInt16 or PlyDataType.Int16 => rawValue / 65535f,
            PlyDataType.UInt32 or PlyDataType.Int32 => rawValue / 255f,
            // Float types: use heuristic (could be 0-1 already, or 0-255 from some exporters)
            _ => rawValue > 1f ? rawValue / 255f : rawValue
        };
    }

    /// <summary>
    /// Normalizes an intensity value to 0-1 range based on the actual PLY data type.
    /// </summary>
    private static float NormalizeIntensity(float rawValue, PlyDataType type)
    {
        return type switch
        {
            PlyDataType.UInt8 or PlyDataType.Int8 => rawValue / 255f,
            PlyDataType.UInt16 or PlyDataType.Int16 => rawValue / 65535f,
            PlyDataType.UInt32 or PlyDataType.Int32 => rawValue / 65535f,
            // Float types: use heuristic
            _ => rawValue > 1f ? rawValue / 65535f : rawValue
        };
    }

    private void EnsureInitialized()
    {
        if (!_isInitialized)
            throw new InvalidOperationException("Reader not initialized. Call Initialize() first.");
    }

    public void Dispose()
    {
        _cache?.Dispose();
        _sectorManager?.Dispose();
        _plyIndex?.Dispose();
    }
}

/// <summary>
/// Complete result of a frame update.
/// Contains traversal info, cache status, and buffer data to upload.
/// </summary>
public class FrameUpdateResult
{
    /// <summary>
    /// Traversal result.
    /// </summary>
    public TraversalResult Traversal { get; set; } = new();

    /// <summary>
    /// Cache loading result.
    /// </summary>
    public CacheLoadResult CacheResult { get; set; } = new();

    /// <summary>
    /// Buffer update result with data to upload.
    /// </summary>
    public BufferUpdateResult BufferUpdate { get; set; } = new();

    /// <summary>
    /// Total time for the entire update.
    /// </summary>
    public long TotalTimeMs { get; set; }

    /// <summary>
    /// Quick access to new sector data to upload.
    /// Mutable list of sectors, each containing a Features dictionary.
    /// Upload these to your DynamicBufferAdvanced buffers.
    /// </summary>
    public List<SectorData> NewSectors => BufferUpdate.NewSectors;

    /// <summary>
    /// Quick access to active sectors for rendering.
    /// Use this for your rendering dispatch.
    /// </summary>
    public SectorInfo[] ActiveSectors => BufferUpdate.ActiveSectors;

    /// <summary>
    /// Total points ready for rendering.
    /// </summary>
    public int TotalPointsInBuffer => BufferUpdate.TotalPointsInBuffer;

    /// <summary>
    /// Whether there's new data to upload this frame.
    /// </summary>
    public bool HasNewData => BufferUpdate.HasNewData;
}
