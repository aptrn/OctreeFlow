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

    // Internal storage for Vector4 features (Position, Colors, Normals)
    private Dictionary<string, Vector4[]> _vectorFeatures = new();

    // Internal storage for scalar features (Intensity and custom scalars)
    private Dictionary<string, float[]> _scalarFeatures = new();

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
    /// Creates SectorData from PointData array.
    /// </summary>
    /// <param name="sectorIndex">Index of the sector.</param>
    /// <param name="byteOffsetVector4">Byte offset for Vector4 data.</param>
    /// <param name="byteOffsetFloat">Byte offset for float data.</param>
    /// <param name="nodeId">Node ID.</param>
    /// <param name="level">Octree level.</param>
    /// <param name="points">Point data array.</param>
    /// <param name="availableVector4Features">Set of Vector4 feature names that should be included (e.g., "Position", "Colors", "Normals"). Only features in this set will be added.</param>
    /// <param name="availableFloat32Features">Set of Float32 feature names that should be included (e.g., "Intensity", scalar names). Only features in this set will be added.</param>
    public static SectorData FromPointData(
        int sectorIndex, 
        int byteOffsetVector4, 
        int byteOffsetFloat, 
        string nodeId, 
        int level, 
        PointData[] points,
        ISet<string> availableVector4Features,
        ISet<string> availableFloat32Features)
    {
        var result = new SectorData
        {
            SectorIndex = sectorIndex,
            NodeId = nodeId,
            Level = level,
            PointCount = points.Length,
            ByteOffsetVector4 = byteOffsetVector4,
            ByteOffsetFloat32 = byteOffsetFloat
        };

        // Create arrays only for available Vector4 features
        foreach (var vectorName in availableVector4Features)
        {
            result._vectorFeatures[vectorName] = new Vector4[points.Length];
        }

        // Create arrays only for available scalar features
        foreach (var scalarName in availableFloat32Features)
        {
            result._scalarFeatures[scalarName] = new float[points.Length];
        }

        // Convert point data
        for (int i = 0; i < points.Length; i++)
        {
            var pt = points[i];
            
            if (result._vectorFeatures.TryGetValue("Position", out var posArr))
                posArr[i] = new Vector4(pt.Position.X, pt.Position.Y, pt.Position.Z, 1f);
            
            if (result._vectorFeatures.TryGetValue("Colors", out var colArr))
                colArr[i] = new Vector4(pt.Color.R, pt.Color.G, pt.Color.B, pt.Color.A);
            
            if (result._vectorFeatures.TryGetValue("Normals", out var normArr))
                normArr[i] = new Vector4(pt.Normal.X, pt.Normal.Y, pt.Normal.Z, 0f);
            
            if (result._scalarFeatures.TryGetValue("Intensity", out var intArr))
                intArr[i] = pt.Intensity;

            // Copy scalars from PointData
            if (pt.Scalars != null)
            {
                foreach (var kvp in pt.Scalars)
                {
                    if (result._scalarFeatures.TryGetValue(kvp.Key, out var arr))
                    {
                        arr[i] = kvp.Value;
                    }
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
    /// Combines data from multiple sectors into contiguous arrays.
    /// </summary>
    private static CombinedBufferData CombineAllSectorData(List<SectorData> sectors)
    {
        var result = new CombinedBufferData();
        
        if (sectors.Count == 0) return result;

        // Calculate total points
        int totalPoints = sectors.Sum(s => s.PointCount);
        result.TotalPointCount = totalPoints;

        if (totalPoints == 0) return result;

        // Collect all Vector4 feature names from all sectors
        var vectorNames = new HashSet<string>();
        foreach (var sector in sectors)
        {
            foreach (var name in sector.VectorFeatureNames)
            {
                vectorNames.Add(name);
            }
        }

        // Collect all scalar feature names from all sectors
        var scalarNames = new HashSet<string>();
        foreach (var sector in sectors)
        {
            foreach (var name in sector.ScalarFeatureNames)
            {
                scalarNames.Add(name);
            }
        }

        // Allocate combined Vector4 arrays
        var vectorArrays = new Dictionary<string, Vector4[]>();
        foreach (var name in vectorNames)
        {
            vectorArrays[name] = new Vector4[totalPoints];
        }

        // Allocate combined scalar arrays
        var scalarArrays = new Dictionary<string, float[]>();
        foreach (var name in scalarNames)
        {
            scalarArrays[name] = new float[totalPoints];
        }

        // Copy data from each sector
        int offset = 0;
        foreach (var sector in sectors)
        {
            int count = sector.PointCount;

            // Copy Vector4 data
            foreach (var name in vectorNames)
            {
                var sectorVector = sector.GetVectorData(name);
                if (sectorVector != null)
                {
                    Array.Copy(sectorVector, 0, vectorArrays[name], offset, count);
                }
                // If sector doesn't have this vector, the array remains zeroed
            }

            // Copy scalar data
            foreach (var name in scalarNames)
            {
                var sectorScalar = sector.GetScalarData(name);
                if (sectorScalar != null)
                {
                    Array.Copy(sectorScalar, 0, scalarArrays[name], offset, count);
                }
                // If sector doesn't have this scalar, the array remains zeroed
            }

            offset += count;
        }

        // Add Vector4 arrays to result
        foreach (var kvp in vectorArrays)
        {
            result.SetVectorData(kvp.Key, kvp.Value);
        }

        // Add scalar arrays to result
        foreach (var kvp in scalarArrays)
        {
            result.SetScalarData(kvp.Key, kvp.Value);
        }

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
    /// Total size in bytes for a Vector4 buffer.
    /// </summary>
    public long TotalBytesVector4 => (long)TotalCapacity * BytesPerVector4;

    /// <summary>
    /// Total size in bytes for a float buffer.
    /// </summary>
    public long TotalBytesFloat => (long)TotalCapacity * BytesPerFloat;

    /// <summary>
    /// Byte offset for a given sector in Vector4 buffers.
    /// </summary>
    public int GetByteOffsetVector4(int sectorIndex) => sectorIndex * MaxPointsPerSector * BytesPerVector4;

    /// <summary>
    /// Byte offset for a given sector in float buffers.
    /// </summary>
    public int GetByteOffsetFloat(int sectorIndex) => sectorIndex * MaxPointsPerSector * BytesPerFloat;

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
