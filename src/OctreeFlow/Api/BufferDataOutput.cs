using Stride.Core.Mathematics;
using OctreeFlow.Data;

namespace OctreeFlow.Api;

/// <summary>
/// Data for a single sector ready to be uploaded to vvvv gamma buffers.
/// Contains two dictionaries: one for Vector4 features and one for Float32 features.
/// Each dictionary maps feature names to their data arrays.
/// </summary>
public class SectorData
{
    /// <summary>
    /// Index of this sector in the buffer.
    /// </summary>
    public int SectorIndex { get; set; }

    /// <summary>
    /// Node ID this sector contains.
    /// </summary>
    public string NodeId { get; set; } = "";

    /// <summary>
    /// Number of points in this sector.
    /// </summary>
    public int PointCount { get; set; }

    /// <summary>
    /// Octree level of this node.
    /// </summary>
    public int Level { get; set; }

    /// <summary>
    /// Byte offset in the buffer for Vector4 features.
    /// Use this with DynamicBufferAdvanced SetData offset for Vector4 buffers.
    /// </summary>
    public int ByteOffsetVector4 { get; set; }

    /// <summary>
    /// Element offset in the buffer for Vector4 features.
    /// Use this if SetData expects element index instead of byte offset.
    /// </summary>
    public int ElementOffsetVector4 => ByteOffsetVector4 / 16;

    /// <summary>
    /// Byte offset in the buffer for Float32 features.
    /// Use this with DynamicBufferAdvanced SetData offset for float buffers.
    /// </summary>
    public int ByteOffsetFloat32 { get; set; }

    /// <summary>
    /// Element offset in the buffer for Float32 features.
    /// Use this if SetData expects element index instead of byte offset.
    /// </summary>
    public int ElementOffsetFloat32 => ByteOffsetFloat32 / 4;

    /// <summary>
    /// Byte offset in the buffer for Int32 features.
    /// Use this with DynamicBufferAdvanced SetData offset for int buffers.
    /// </summary>
    public int ByteOffsetInt32 { get; set; }

    /// <summary>
    /// Element offset in the buffer for Int32 features.
    /// Use this if SetData expects element index instead of byte offset.
    /// </summary>
    public int ElementOffsetInt32 => ByteOffsetInt32 / 4;

    // Internal storage for Vector4 features (Position, Colors, Normals)
    private Dictionary<string, Vector4[]> _vectorFeatures = new();

    // Internal storage for scalar features (Intensity and custom scalars)
    private Dictionary<string, float[]> _scalarFeatures = new();

    // Internal storage for integer features (Id and custom integers)
    private Dictionary<string, int[]> _integerFeatures = new();

    /// <summary>
    /// Vector4 feature data as sequence of key-value pairs.
    /// Keys: "Position", "Colors", "Normals".
    /// Values: Vector4[] arrays ready for upload to GPU buffers.
    /// </summary>
    public IEnumerable<KeyValuePair<string, Vector4[]>> VectorFeatures => _vectorFeatures;

    /// <summary>
    /// Scalar feature data as sequence of key-value pairs.
    /// Keys are scalar property names from the PLY file (e.g., "Intensity", custom scalars).
    /// Values: float[] arrays ready for upload to GPU buffers.
    /// </summary>
    public IEnumerable<KeyValuePair<string, float[]>> ScalarFeatures => _scalarFeatures;

    /// <summary>
    /// Checks if this sector has data for a specific Vector4 feature.
    /// </summary>
    /// <param name="name">Feature name (e.g., "Position", "Colors", "Normals").</param>
    /// <returns>True if the vector data is available.</returns>
    public bool HasVectorData(string name) => _vectorFeatures.ContainsKey(name);

    /// <summary>
    /// Gets the data for a specific Vector4 feature.
    /// </summary>
    /// <param name="name">Feature name (e.g., "Position", "Colors", "Normals").</param>
    /// <returns>The Vector4 array, or null if not available.</returns>
    public Vector4[]? GetVectorData(string name) => 
        _vectorFeatures.TryGetValue(name, out var arr) ? arr : null;

    /// <summary>
    /// Gets the names of all available Vector4 features in this sector.
    /// </summary>
    public IEnumerable<string> VectorFeatureNames => _vectorFeatures.Keys;

    /// <summary>
    /// Checks if this sector has data for a specific scalar feature.
    /// </summary>
    /// <param name="name">Scalar property name.</param>
    /// <returns>True if the scalar data is available.</returns>
    public bool HasScalarData(string name) => _scalarFeatures.ContainsKey(name);

    /// <summary>
    /// Gets the data for a specific scalar feature.
    /// </summary>
    /// <param name="name">Scalar property name.</param>
    /// <returns>The float array, or null if not available.</returns>
    public float[]? GetScalarData(string name) => 
        _scalarFeatures.TryGetValue(name, out var arr) ? arr : null;

    /// <summary>
    /// Gets the names of all available scalar features in this sector.
    /// </summary>
    public IEnumerable<string> ScalarFeatureNames => _scalarFeatures.Keys;

    /// <summary>
    /// Integer feature data as sequence of key-value pairs.
    /// Keys are integer property names (e.g., "Id").
    /// Values: int[] arrays ready for upload to GPU buffers.
    /// </summary>
    public IEnumerable<KeyValuePair<string, int[]>> IntegerFeatures => _integerFeatures;

    /// <summary>
    /// Checks if this sector has data for a specific integer feature.
    /// </summary>
    /// <param name="name">Integer property name (e.g., "Id").</param>
    /// <returns>True if the integer data is available.</returns>
    public bool HasIntegerData(string name) => _integerFeatures.ContainsKey(name);

    /// <summary>
    /// Gets the data for a specific integer feature.
    /// </summary>
    /// <param name="name">Integer property name (e.g., "Id").</param>
    /// <returns>The int array, or null if not available.</returns>
    public int[]? GetIntegerData(string name) => 
        _integerFeatures.TryGetValue(name, out var arr) ? arr : null;

    /// <summary>
    /// Gets the names of all available integer features in this sector.
    /// </summary>
    public IEnumerable<string> IntegerFeatureNames => _integerFeatures.Keys;

    /// <summary>
    /// Internal: Sets vector data (used by FromPointData).
    /// </summary>
    internal void SetVectorData(string name, Vector4[] data)
    {
        _vectorFeatures[name] = data;
    }

    /// <summary>
    /// Internal: Sets scalar data (used by FromPointData).
    /// </summary>
    internal void SetScalarData(string name, float[] data)
    {
        _scalarFeatures[name] = data;
    }

    /// <summary>
    /// Internal: Sets integer data (used by FromPointData).
    /// </summary>
    internal void SetIntegerData(string name, int[] data)
    {
        _integerFeatures[name] = data;
    }

    /// <summary>
    /// Creates SectorData from PointData array.
    /// Feature keys use the Point_ prefix convention (e.g., "Point_Position", "Point_Color").
    /// </summary>
    /// <param name="sectorIndex">Index of the sector.</param>
    /// <param name="byteOffsetVector4">Byte offset for Vector4 data.</param>
    /// <param name="byteOffsetFloat">Byte offset for float data.</param>
    /// <param name="nodeId">String node ID.</param>
    /// <param name="nodeIntId">Integer node ID (NodeInfo.NodeId). Written to Point_NodeID.</param>
    /// <param name="level">Octree level.</param>
    /// <param name="points">Point data array.</param>
    /// <param name="availableVector4Features">Set of Vector4 feature names (e.g., "Point_Position").</param>
    /// <param name="availableFloat32Features">Set of Float32 feature names (e.g., "Point_Intensity").</param>
    public static SectorData FromPointData(
        int sectorIndex,
        int byteOffsetVector4,
        int byteOffsetFloat,
        string nodeId,
        int nodeIntId,
        int level,
        PointData[] points,
        ISet<string> availableVector4Features,
        ISet<string> availableFloat32Features)
    {
        return FromPointData(sectorIndex, byteOffsetVector4, byteOffsetFloat, 0, nodeId, nodeIntId, level,
            points, availableVector4Features, availableFloat32Features, new HashSet<string>());
    }

    /// <summary>
    /// Creates SectorData from PointData array with integer feature support.
    /// Feature keys use the Point_ prefix convention (e.g., "Point_Position", "Point_Level").
    /// Point_NodeID is automatically filled with nodeIntId for all points in this sector.
    /// </summary>
    /// <param name="sectorIndex">Index of the sector.</param>
    /// <param name="byteOffsetVector4">Byte offset for Vector4 data.</param>
    /// <param name="byteOffsetFloat">Byte offset for float data.</param>
    /// <param name="byteOffsetInt32">Byte offset for int32 data.</param>
    /// <param name="nodeId">String node ID.</param>
    /// <param name="nodeIntId">
    /// Integer node ID (NodeInfo.NodeId, DFS sequential 0…TotalNodes−1).
    /// Written into the Point_NodeID buffer so shaders can cross-reference BF buffers:
    ///   float density = BF_Density[Point_NodeID[pointId]];
    /// </param>
    /// <param name="level">Octree level.</param>
    /// <param name="points">Point data array.</param>
    /// <param name="availableVector4Features">Set of Vector4 feature names (e.g., "Point_Position", "Point_Color", "Point_Normal").</param>
    /// <param name="availableFloat32Features">Set of Float32 feature names (e.g., "Point_Intensity", custom "Point_*" scalars).</param>
    /// <param name="availableInt32Features">Set of Int32 feature names (e.g., "Point_Id", "Point_Level", "Point_NodeID").</param>
    public static SectorData FromPointData(
        int sectorIndex,
        int byteOffsetVector4,
        int byteOffsetFloat,
        int byteOffsetInt32,
        string nodeId,
        int nodeIntId,
        int level,
        PointData[] points,
        ISet<string> availableVector4Features,
        ISet<string> availableFloat32Features,
        ISet<string> availableInt32Features)
    {
        var result = new SectorData
        {
            SectorIndex = sectorIndex,
            NodeId = nodeId,
            Level = level,
            PointCount = points.Length,
            ByteOffsetVector4 = byteOffsetVector4,
            ByteOffsetFloat32 = byteOffsetFloat,
            ByteOffsetInt32 = byteOffsetInt32
        };

        // Allocate per-feature arrays
        foreach (var name in availableVector4Features)
            result._vectorFeatures[name] = new Vector4[points.Length];

        foreach (var name in availableFloat32Features)
            result._scalarFeatures[name] = new float[points.Length];

        foreach (var name in availableInt32Features)
            result._integerFeatures[name] = new int[points.Length];

        // Point_NodeID is constant for every point in a sector — fill it upfront.
        if (result._integerFeatures.TryGetValue("Point_NodeID", out var nodeIdArr))
            Array.Fill(nodeIdArr, nodeIntId);

        // Convert per-point data
        for (int i = 0; i < points.Length; i++)
        {
            var pt = points[i];

            if (result._vectorFeatures.TryGetValue("Point_Position", out var posArr))
                posArr[i] = new Vector4(pt.Position.X, pt.Position.Y, pt.Position.Z, 1f);

            if (result._vectorFeatures.TryGetValue("Point_Color", out var colArr))
                colArr[i] = new Vector4(pt.Color.R, pt.Color.G, pt.Color.B, pt.Color.A);

            if (result._vectorFeatures.TryGetValue("Point_Normal", out var normArr))
                normArr[i] = new Vector4(pt.Normal.X, pt.Normal.Y, pt.Normal.Z, 0f);

            if (result._scalarFeatures.TryGetValue("Point_Intensity", out var intensityArr))
                intensityArr[i] = pt.Intensity;

            if (result._integerFeatures.TryGetValue("Point_Id", out var idArr))
                idArr[i] = pt.Id;

            if (result._integerFeatures.TryGetValue("Point_Level", out var levelArr))
                levelArr[i] = level;

            // Custom PLY scalars — stored lowercase in PointData.Scalars,
            // registered as "Point_{name}" in the feature dictionaries.
            if (pt.Scalars != null)
            {
                foreach (var kvp in pt.Scalars)
                {
                    if (result._scalarFeatures.TryGetValue("Point_" + kvp.Key, out var scalarArr))
                        scalarArr[i] = kvp.Value;
                }
            }
        }

        return result;
    }
}

/// <summary>
/// Information about an active sector (for rendering dispatch).
/// </summary>
public struct SectorInfo
{
    /// <summary>
    /// Sector index.
    /// </summary>
    public int SectorIndex { get; set; }

    /// <summary>
    /// Byte offset in buffer (for Vector4 buffers).
    /// </summary>
    public int ByteOffsetVector4 { get; set; }

    /// <summary>
    /// Byte offset in buffer (for float buffers).
    /// </summary>
    public int ByteOffsetFloat { get; set; }

    /// <summary>
    /// Byte offset in buffer (for int32 buffers).
    /// </summary>
    public int ByteOffsetInt32 { get; set; }

    /// <summary>
    /// Start index in buffer (element index, not byte).
    /// </summary>
    public int StartIndex { get; set; }

    /// <summary>
    /// Number of points in this sector.
    /// </summary>
    public int PointCount { get; set; }

    /// <summary>
    /// Node ID in this sector.
    /// </summary>
    public string NodeId { get; set; }

    /// <summary>
    /// Octree level of this node.
    /// </summary>
    public int Level { get; set; }
}

/// <summary>
/// Combined buffer data for uploading all sectors at once (at offset 0).
/// Use this if your buffer implementation doesn't support non-zero offsets.
/// </summary>
public class CombinedBufferData
{
    // Internal storage for combined Vector4 features
    private Dictionary<string, Vector4[]> _vectorFeatures = new();

    // Internal storage for combined scalar features
    private Dictionary<string, float[]> _scalarFeatures = new();

    // Internal storage for combined integer features
    private Dictionary<string, int[]> _integerFeatures = new();

    /// <summary>
    /// Total number of points across all sectors.
    /// </summary>
    public int TotalPointCount { get; set; }

    /// <summary>
    /// Combined Vector4 feature data as sequence of key-value pairs.
    /// Keys: "Position", "Colors", "Normals".
    /// Values: Vector4[] arrays ready for upload to GPU buffers.
    /// </summary>
    public IEnumerable<KeyValuePair<string, Vector4[]>> VectorFeatures => _vectorFeatures;

    /// <summary>
    /// Combined scalar feature data as sequence of key-value pairs.
    /// Keys are scalar property names from the PLY file (e.g., "Intensity", custom scalars).
    /// Values: float[] arrays ready for upload to GPU buffers.
    /// </summary>
    public IEnumerable<KeyValuePair<string, float[]>> ScalarFeatures => _scalarFeatures;

    /// <summary>
    /// Combined integer feature data as sequence of key-value pairs.
    /// Keys are integer property names (e.g., "Id").
    /// Values: int[] arrays ready for upload to GPU buffers.
    /// </summary>
    public IEnumerable<KeyValuePair<string, int[]>> IntegerFeatures => _integerFeatures;

    /// <summary>
    /// Checks if this combined data has a specific Vector4 feature.
    /// </summary>
    /// <param name="name">Feature name (e.g., "Position", "Colors", "Normals").</param>
    /// <returns>True if the vector data is available.</returns>
    public bool HasVectorData(string name) => _vectorFeatures.ContainsKey(name) && TotalPointCount > 0;

    /// <summary>
    /// Gets the combined data for a specific Vector4 feature.
    /// </summary>
    /// <param name="name">Feature name (e.g., "Position", "Colors", "Normals").</param>
    /// <returns>The Vector4 array, or null if not available.</returns>
    public Vector4[]? GetVectorData(string name) => 
        _vectorFeatures.TryGetValue(name, out var arr) ? arr : null;

    /// <summary>
    /// Gets the names of all available Vector4 features.
    /// </summary>
    public IEnumerable<string> VectorFeatureNames => _vectorFeatures.Keys;

    /// <summary>
    /// Checks if this combined data has a specific scalar feature.
    /// </summary>
    /// <param name="name">Scalar property name.</param>
    /// <returns>True if the scalar data is available.</returns>
    public bool HasScalarData(string name) => _scalarFeatures.ContainsKey(name) && TotalPointCount > 0;

    /// <summary>
    /// Gets the combined data for a specific scalar feature.
    /// </summary>
    /// <param name="name">Scalar property name.</param>
    /// <returns>The float array, or null if not available.</returns>
    public float[]? GetScalarData(string name) => 
        _scalarFeatures.TryGetValue(name, out var arr) ? arr : null;

    /// <summary>
    /// Gets the names of all available scalar features.
    /// </summary>
    public IEnumerable<string> ScalarFeatureNames => _scalarFeatures.Keys;

    /// <summary>
    /// Checks if this combined data has a specific integer feature.
    /// </summary>
    /// <param name="name">Integer property name (e.g., "Id").</param>
    /// <returns>True if the integer data is available.</returns>
    public bool HasIntegerData(string name) => _integerFeatures.ContainsKey(name) && TotalPointCount > 0;

    /// <summary>
    /// Gets the combined data for a specific integer feature.
    /// </summary>
    /// <param name="name">Integer property name (e.g., "Id").</param>
    /// <returns>The int array, or null if not available.</returns>
    public int[]? GetIntegerData(string name) => 
        _integerFeatures.TryGetValue(name, out var arr) ? arr : null;

    /// <summary>
    /// Gets the names of all available integer features.
    /// </summary>
    public IEnumerable<string> IntegerFeatureNames => _integerFeatures.Keys;

    /// <summary>
    /// Internal: Sets vector data (used by CombineAllSectorData).
    /// </summary>
    internal void SetVectorData(string name, Vector4[] data)
    {
        _vectorFeatures[name] = data;
    }

    /// <summary>
    /// Internal: Sets scalar data (used by CombineAllSectorData).
    /// </summary>
    internal void SetScalarData(string name, float[] data)
    {
        _scalarFeatures[name] = data;
    }

    /// <summary>
    /// Internal: Sets integer data (used by CombineAllSectorData).
    /// </summary>
    internal void SetIntegerData(string name, int[] data)
    {
        _integerFeatures[name] = data;
    }
}

/// <summary>
/// Result of a buffer manager update.
/// Contains new data to upload and current sector states.
/// </summary>
public class BufferUpdateResult
{
    /// <summary>
    /// Version number (increments on any change).
    /// </summary>
    public int Version { get; set; }

    /// <summary>
    /// Time taken for the update in milliseconds.
    /// </summary>
    public long UpdateTimeMs { get; set; }

    /// <summary>
    /// Mutable list of sectors with new data to upload.
    /// Each sector contains a Features dictionary where key is feature name 
    /// (Position, Colors, Normals, Intensity, or scalar names) and value contains 
    /// the data array and byte offset for that feature.
    /// Upload these using DynamicBufferAdvanced SetData with the byte offsets.
    /// </summary>
    public List<SectorData> NewSectors { get; set; } = new();

    /// <summary>
    /// All active sectors' data (includes both new and previously loaded).
    /// Use this for GetCombinedAllActiveData() to get the complete buffer state.
    /// </summary>
    public List<SectorData> AllActiveSectors { get; set; } = new();

    /// <summary>
    /// Sector indices that were released (cleared).
    /// You may optionally zero these regions in your buffers.
    /// </summary>
    public int[] ReleasedSectors { get; set; } = Array.Empty<int>();

    /// <summary>
    /// All currently active sectors (for rendering).
    /// Use this to know which sectors to render and their point counts.
    /// </summary>
    public SectorInfo[] ActiveSectors { get; set; } = Array.Empty<SectorInfo>();

    /// <summary>
    /// Number of nodes newly loaded this frame.
    /// </summary>
    public int NodesLoaded { get; set; }

    /// <summary>
    /// Number of nodes already in buffer.
    /// </summary>
    public int NodesAlreadyLoaded { get; set; }

    /// <summary>
    /// Number of nodes released this frame.
    /// </summary>
    public int NodesReleased { get; set; }

    /// <summary>
    /// Number of nodes skipped (not in cache or no space).
    /// </summary>
    public int NodesSkipped { get; set; }

    /// <summary>
    /// Total points across all active sectors.
    /// </summary>
    public int TotalPointsInBuffer { get; set; }

    /// <summary>
    /// True if there are new sectors to upload.
    /// </summary>
    public bool HasNewData => NewSectors.Count > 0;

    #region Diagnostic Properties

    /// <summary>
    /// Number of nodes that had data truncated because they exceeded MaxPointsPerSector.
    /// If this is > 0, you may need to increase pointsPerNodeOverride.
    /// </summary>
    public int TruncatedNodes { get; set; }

    /// <summary>
    /// Total number of points lost due to truncation.
    /// </summary>
    public int TruncatedPoints { get; set; }

    /// <summary>
    /// Reason for the last skipped node (for debugging).
    /// </summary>
    public string? LastSkipReason { get; set; }

    /// <summary>
    /// Maximum points allowed per sector (= PointsPerNode setting).
    /// </summary>
    public int MaxPointsPerSector { get; set; }

    /// <summary>
    /// Total number of sectors available in the buffer.
    /// </summary>
    public int AvailableSectors { get; set; }

    /// <summary>
    /// Number of sectors currently in use.
    /// </summary>
    public int UsedSectors { get; set; }

    /// <summary>
    /// True if any data was truncated this frame.
    /// </summary>
    public bool HasTruncation => TruncatedNodes > 0;

    #endregion

    /// <summary>
    /// Gets combined buffer data from all NEW sectors only, for uploading at offset 0.
    /// WARNING: This only returns NEW data - previously loaded sectors are not included!
    /// For complete buffer state, use GetCombinedAllActiveData() instead.
    /// </summary>
    public CombinedBufferData GetCombinedNewData()
    {
        return CombineAllSectorData(NewSectors);
    }

    /// <summary>
    /// Gets combined buffer data from all NEW sectors, padded to <paramref name="maximumSize"/>.
    /// See <see cref="GetCombinedNewData()"/> for details on the "new only" caveat.
    /// Slots [TotalPointCount..maximumSize-1] are zero-padded.
    /// </summary>
    public CombinedBufferData GetCombinedNewData(int maximumSize)
    {
        return CombineAllSectorData(NewSectors, maximumSize);
    }

    /// <summary>
    /// Gets combined buffer data from ALL ACTIVE sectors (both new and previously loaded).
    /// Use this to get the complete buffer state for uploading at offset 0.
    /// This is what you should use for simple vvvv gamma setups where you upload
    /// the entire buffer each frame.
    /// </summary>
    public CombinedBufferData GetCombinedAllActiveData()
    {
        return CombineAllSectorData(AllActiveSectors);
    }

    /// <summary>
    /// Gets combined buffer data from ALL ACTIVE sectors, with all arrays padded to
    /// <paramref name="maximumSize"/> so every buffer in your VL.Fuse patch shares the same
    /// element count and ShaderNode reads stay index-synchronized.
    ///
    /// Only Point_* features are written — BF_* and Vertex_* buffers are managed separately
    /// via BuildStaticNodeData / BuildStaticVertexData and are never touched here.
    ///
    /// Slots [TotalPointCount..maximumSize-1] are zero-padded.
    /// </summary>
    /// <param name="maximumSize">
    /// Total number of elements in every output array.
    /// Use the same value passed to BuildStaticNodeData / BuildStaticVertexData.
    /// </param>
    public CombinedBufferData GetCombinedAllActiveData(int maximumSize)
    {
        return CombineAllSectorData(AllActiveSectors, maximumSize);
    }

    /// <summary>
    /// Combines data from multiple sectors into contiguous arrays.
    /// Only Point_* features are written — BF_* and Vertex_* data are never included.
    /// </summary>
    private static CombinedBufferData CombineAllSectorData(
        List<SectorData> sectors,
        int maximumSize = 0)
    {
        var result = new CombinedBufferData();

        if (sectors.Count == 0) return result;

        int totalPoints = sectors.Sum(s => s.PointCount);
        result.TotalPointCount = totalPoints;

        if (totalPoints == 0) return result;

        int bufferSize = maximumSize > 0 ? Math.Max(totalPoints, maximumSize) : totalPoints;

        // Collect all feature names across sectors
        var vectorNames  = new HashSet<string>();
        var scalarNames  = new HashSet<string>();
        var integerNames = new HashSet<string>();

        foreach (var sector in sectors)
        {
            foreach (var name in sector.VectorFeatureNames)  vectorNames.Add(name);
            foreach (var name in sector.ScalarFeatureNames)  scalarNames.Add(name);
            foreach (var name in sector.IntegerFeatureNames) integerNames.Add(name);
        }

        // Allocate arrays at bufferSize (zero-padded beyond totalPoints)
        var vectorArrays  = new Dictionary<string, Vector4[]>();
        var scalarArrays  = new Dictionary<string, float[]>();
        var integerArrays = new Dictionary<string, int[]>();

        foreach (var name in vectorNames)  vectorArrays[name]  = new Vector4[bufferSize];
        foreach (var name in scalarNames)  scalarArrays[name]  = new float[bufferSize];
        foreach (var name in integerNames) integerArrays[name] = new int[bufferSize];

        // Copy sector data into the flat arrays
        int offset = 0;
        foreach (var sector in sectors)
        {
            int count = sector.PointCount;

            foreach (var name in vectorNames)
            {
                var src = sector.GetVectorData(name);
                if (src != null) Array.Copy(src, 0, vectorArrays[name], offset, count);
            }

            foreach (var name in scalarNames)
            {
                var src = sector.GetScalarData(name);
                if (src != null) Array.Copy(src, 0, scalarArrays[name], offset, count);
            }

            foreach (var name in integerNames)
            {
                var src = sector.GetIntegerData(name);
                if (src != null) Array.Copy(src, 0, integerArrays[name], offset, count);
            }

            offset += count;
        }

        foreach (var kvp in vectorArrays)  result.SetVectorData(kvp.Key,  kvp.Value);
        foreach (var kvp in scalarArrays)  result.SetScalarData(kvp.Key,  kvp.Value);
        foreach (var kvp in integerArrays) result.SetIntegerData(kvp.Key, kvp.Value);

        return result;
    }
}

/// <summary>
/// Configuration for buffer sizes.
/// </summary>
public class BufferConfiguration
{
    /// <summary>
    /// Maximum points per sector.
    /// </summary>
    public int MaxPointsPerSector { get; set; } = 65536;

    /// <summary>
    /// Number of sectors in the buffer.
    /// </summary>
    public int SectorCount { get; set; } = 64;

    /// <summary>
    /// Total buffer capacity in points.
    /// </summary>
    public int TotalCapacity => MaxPointsPerSector * SectorCount;

    /// <summary>
    /// Bytes per point for Vector4 buffers (16 bytes per Vector4).
    /// </summary>
    public const int BytesPerVector4 = 16;

    /// <summary>
    /// Bytes per point for float buffers (4 bytes per float).
    /// </summary>
    public const int BytesPerFloat = 4;

    /// <summary>
    /// Bytes per point for int32 buffers (4 bytes per int).
    /// </summary>
    public const int BytesPerInt32 = 4;

    /// <summary>
    /// Total size in bytes for a Vector4 buffer.
    /// </summary>
    public long TotalBytesVector4 => (long)TotalCapacity * BytesPerVector4;

    /// <summary>
    /// Total size in bytes for a float buffer.
    /// </summary>
    public long TotalBytesFloat => (long)TotalCapacity * BytesPerFloat;

    /// <summary>
    /// Total size in bytes for an int32 buffer.
    /// </summary>
    public long TotalBytesInt32 => (long)TotalCapacity * BytesPerInt32;

    /// <summary>
    /// Byte offset for a given sector in Vector4 buffers.
    /// </summary>
    public int GetByteOffsetVector4(int sectorIndex) => sectorIndex * MaxPointsPerSector * BytesPerVector4;

    /// <summary>
    /// Byte offset for a given sector in float buffers.
    /// </summary>
    public int GetByteOffsetFloat(int sectorIndex) => sectorIndex * MaxPointsPerSector * BytesPerFloat;

    /// <summary>
    /// Byte offset for a given sector in int32 buffers.
    /// </summary>
    public int GetByteOffsetInt32(int sectorIndex) => sectorIndex * MaxPointsPerSector * BytesPerInt32;

    /// <summary>
    /// Creates configuration from buffer size in bytes.
    /// </summary>
    /// <param name="bufferSizeBytes">Desired buffer size in bytes (per buffer type).</param>
    /// <param name="maxPointsPerSector">Maximum points per sector.</param>
    public static BufferConfiguration FromBufferSize(long bufferSizeBytes, int maxPointsPerSector = 65536)
    {
        // Calculate how many sectors fit in the given size
        // Using Vector4 size (16 bytes) as reference since it's the largest per-element
        long bytesPerSector = (long)maxPointsPerSector * BytesPerVector4;
        int sectorCount = Math.Max(1, (int)(bufferSizeBytes / bytesPerSector));

        return new BufferConfiguration
        {
            MaxPointsPerSector = maxPointsPerSector,
            SectorCount = sectorCount
        };
    }
}
