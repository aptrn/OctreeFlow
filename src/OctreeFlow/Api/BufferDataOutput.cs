using Stride.Core.Mathematics;
using OctreeFlow.Data;

namespace OctreeFlow.Api;

/// <summary>
/// Data for a single sector ready to be uploaded to vvvv gamma buffers.
/// Contains Vector4 arrays for positions, colors, normals, and float arrays for scalars.
/// Use ByteOffset to know where to write in the DynamicBufferAdvanced.
/// </summary>
public class SectorData
{
    /// <summary>
    /// Index of this sector in the buffer.
    /// </summary>
    public int SectorIndex { get; set; }

    /// <summary>
    /// Byte offset in the buffer where this sector starts.
    /// Use this with DynamicBufferAdvanced SetData offset.
    /// </summary>
    public int ByteOffset { get; set; }

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
    /// Position data as Vector4 (xyz + w padding).
    /// Ready for MutableArray&lt;Vector4&gt; in vvvv gamma.
    /// </summary>
    public Vector4[] Positions { get; set; } = Array.Empty<Vector4>();

    /// <summary>
    /// Color data as Vector4 (rgba).
    /// Ready for MutableArray&lt;Vector4&gt; in vvvv gamma.
    /// </summary>
    public Vector4[] Colors { get; set; } = Array.Empty<Vector4>();

    /// <summary>
    /// Normal data as Vector4 (xyz + w padding).
    /// Ready for MutableArray&lt;Vector4&gt; in vvvv gamma.
    /// </summary>
    public Vector4[] Normals { get; set; } = Array.Empty<Vector4>();

    /// <summary>
    /// Intensity values as float array.
    /// Ready for MutableArray&lt;Float32&gt; in vvvv gamma.
    /// </summary>
    public float[] Intensities { get; set; } = Array.Empty<float>();

    /// <summary>
    /// Additional scalar properties (e.g., classification, etc.).
    /// Key is property name, value is float array.
    /// </summary>
    public Dictionary<string, float[]> Scalars { get; set; } = new();

    /// <summary>
    /// Creates SectorData from PointData array.
    /// </summary>
    public static SectorData FromPointData(int sectorIndex, int byteOffset, string nodeId, int level, PointData[] points)
    {
        var result = new SectorData
        {
            SectorIndex = sectorIndex,
            ByteOffset = byteOffset,
            NodeId = nodeId,
            Level = level,
            PointCount = points.Length,
            Positions = new Vector4[points.Length],
            Colors = new Vector4[points.Length],
            Normals = new Vector4[points.Length],
            Intensities = new float[points.Length]
        };

        // Collect scalar property names
        var scalarNames = new HashSet<string>();
        foreach (var pt in points)
        {
            if (pt.Scalars != null)
            {
                foreach (var key in pt.Scalars.Keys)
                    scalarNames.Add(key);
            }
        }

        // Initialize scalar arrays
        foreach (var name in scalarNames)
        {
            result.Scalars[name] = new float[points.Length];
        }

        // Convert point data
        for (int i = 0; i < points.Length; i++)
        {
            var pt = points[i];
            result.Positions[i] = new Vector4(pt.Position.X, pt.Position.Y, pt.Position.Z, 1f);
            result.Colors[i] = new Vector4(pt.Color.R, pt.Color.G, pt.Color.B, pt.Color.A);
            result.Normals[i] = new Vector4(pt.Normal.X, pt.Normal.Y, pt.Normal.Z, 0f);
            result.Intensities[i] = pt.Intensity;

            // Copy scalars
            if (pt.Scalars != null)
            {
                foreach (var kvp in pt.Scalars)
                {
                    result.Scalars[kvp.Key][i] = kvp.Value;
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
    /// Sectors with new data to upload.
    /// Upload these using DynamicBufferAdvanced SetData with the byte offsets.
    /// </summary>
    public SectorData[] NewSectors { get; set; } = Array.Empty<SectorData>();

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
    public bool HasNewData => NewSectors.Length > 0;
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
    public int TotalBytesVector4 => TotalCapacity * BytesPerVector4;

    /// <summary>
    /// Total size in bytes for a float buffer.
    /// </summary>
    public int TotalBytesFloat => TotalCapacity * BytesPerFloat;

    /// <summary>
    /// Byte offset for a given sector in Vector4 buffers.
    /// </summary>
    public int GetByteOffsetVector4(int sectorIndex) => sectorIndex * MaxPointsPerSector * BytesPerVector4;

    /// <summary>
    /// Byte offset for a given sector in float buffers.
    /// </summary>
    public int GetByteOffsetFloat(int sectorIndex) => sectorIndex * MaxPointsPerSector * BytesPerFloat;

    /// <summary>
    /// Creates configuration from buffer size in MB.
    /// </summary>
    /// <param name="bufferSizeMB">Desired buffer size in MB (per buffer type).</param>
    /// <param name="maxPointsPerSector">Maximum points per sector.</param>
    public static BufferConfiguration FromBufferSize(int bufferSizeMB, int maxPointsPerSector = 65536)
    {
        // Calculate how many sectors fit in the given size
        // Using Vector4 size (16 bytes) as reference since it's the largest per-element
        int bytesPerSector = maxPointsPerSector * BytesPerVector4;
        int sectorCount = Math.Max(1, (bufferSizeMB * 1024 * 1024) / bytesPerSector);

        return new BufferConfiguration
        {
            MaxPointsPerSector = maxPointsPerSector,
            SectorCount = sectorCount
        };
    }
}
