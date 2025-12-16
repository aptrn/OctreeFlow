using Stride.Core.Mathematics;
using System.Runtime.InteropServices;

namespace OctreeFlow.Api;

/// <summary>
/// GPU-friendly point data structure.
/// Packed struct for efficient GPU buffer uploads.
/// </summary>
[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct GpuPointData
{
    /// <summary>
    /// Position (12 bytes).
    /// </summary>
    public Vector3 Position;

    /// <summary>
    /// Color RGBA (16 bytes).
    /// </summary>
    public Color4 Color;

    /// <summary>
    /// Normal (12 bytes).
    /// </summary>
    public Vector3 Normal;

    /// <summary>
    /// Intensity (4 bytes).
    /// </summary>
    public float Intensity;

    // Total: 44 bytes per point

    /// <summary>
    /// Size of this struct in bytes.
    /// </summary>
    public static readonly int SizeInBytes = Marshal.SizeOf<GpuPointData>();

    public GpuPointData(Vector3 position, Color4 color, Vector3 normal, float intensity)
    {
        Position = position;
        Color = color;
        Normal = normal;
        Intensity = intensity;
    }

    /// <summary>
    /// Converts from PointData to GPU-friendly format.
    /// </summary>
    public static GpuPointData FromPointData(Data.PointData point)
    {
        return new GpuPointData(point.Position, point.Color, point.Normal, point.Intensity);
    }

    /// <summary>
    /// Converts an array of PointData to GPU format.
    /// </summary>
    public static GpuPointData[] FromPointDataArray(Data.PointData[] points)
    {
        var result = new GpuPointData[points.Length];
        for (int i = 0; i < points.Length; i++)
        {
            result[i] = FromPointData(points[i]);
        }
        return result;
    }

    /// <summary>
    /// Converts an array of PointData to a byte array for GPU upload.
    /// </summary>
    public static byte[] ToByteArray(Data.PointData[] points)
    {
        int stride = SizeInBytes;
        byte[] result = new byte[points.Length * stride];

        using var ms = new System.IO.MemoryStream(result);
        using var writer = new System.IO.BinaryWriter(ms);

        for (int i = 0; i < points.Length; i++)
        {
            var p = points[i];
            // Position
            writer.Write(p.Position.X);
            writer.Write(p.Position.Y);
            writer.Write(p.Position.Z);
            // Color
            writer.Write(p.Color.R);
            writer.Write(p.Color.G);
            writer.Write(p.Color.B);
            writer.Write(p.Color.A);
            // Normal
            writer.Write(p.Normal.X);
            writer.Write(p.Normal.Y);
            writer.Write(p.Normal.Z);
            // Intensity
            writer.Write(p.Intensity);
        }

        return result;
    }
}

/// <summary>
/// Provides sector data ready for GPU upload.
/// </summary>
public class GpuSectorData
{
    /// <summary>
    /// Sector index in the GPU buffer.
    /// </summary>
    public int SectorIndex { get; set; }

    /// <summary>
    /// Node ID this sector contains.
    /// </summary>
    public string NodeId { get; set; } = "";

    /// <summary>
    /// Point data in GPU-ready format.
    /// </summary>
    public GpuPointData[] Points { get; set; } = Array.Empty<GpuPointData>();

    /// <summary>
    /// Raw byte data for GPU upload.
    /// </summary>
    public byte[] RawData { get; set; } = Array.Empty<byte>();

    /// <summary>
    /// Number of points.
    /// </summary>
    public int PointCount => Points.Length;

    /// <summary>
    /// Size in bytes.
    /// </summary>
    public int SizeInBytes => RawData.Length;
}

