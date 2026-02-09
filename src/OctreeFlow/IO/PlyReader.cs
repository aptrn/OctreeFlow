using Stride.Core.Mathematics;
using OctreeFlow.Data;
using System.Globalization;
using System.Text;

namespace OctreeFlow.IO;

/// <summary>
/// Reads PLY (Polygon File Format) files with support for various point attributes.
/// Handles ASCII and binary formats, and various headers from different sources (COPC.laz, etc.).
/// </summary>
public class PlyReader
{
    /// <summary>
    /// PLY property types.
    /// </summary>
    private enum PlyType
    {
        Char, UChar, Short, UShort, Int, UInt, Float, Double,
        Int8, UInt8, Int16, UInt16, Int32, UInt32, Float32, Float64
    }

    /// <summary>
    /// PLY format type.
    /// </summary>
    private enum PlyFormat
    {
        Ascii,
        BinaryLittleEndian,
        BinaryBigEndian
    }

    /// <summary>
    /// Property definition from header.
    /// </summary>
    private class PlyProperty
    {
        public string Name { get; set; } = "";
        public PlyType Type { get; set; }
        public bool IsList { get; set; }
        public PlyType ListCountType { get; set; }
    }

    /// <summary>
    /// Reads a PLY file and returns a PointCloud.
    /// </summary>
    public PointCloud Read(string filePath)
    {
        using var stream = File.OpenRead(filePath);
        return Read(stream);
    }

    /// <summary>
    /// Reads a PLY file from a stream and returns a PointCloud.
    /// </summary>
    public PointCloud Read(Stream stream)
    {
        using var reader = new BinaryReader(stream, Encoding.ASCII, leaveOpen: true);

        // Parse header
        var (format, vertexCount, properties, headerEndPosition) = ParseHeader(stream);

        // Seek to data start
        stream.Position = headerEndPosition;

        // Read points
        var cloud = new PointCloud();
        cloud.PropertyNames.AddRange(properties.Select(p => p.Name));

        if (format == PlyFormat.Ascii)
        {
            ReadAsciiData(stream, cloud, vertexCount, properties);
        }
        else
        {
            bool bigEndian = format == PlyFormat.BinaryBigEndian;
            ReadBinaryData(reader, cloud, vertexCount, properties, bigEndian);
        }

        return cloud;
    }

    private (PlyFormat format, int vertexCount, List<PlyProperty> properties, long headerEnd) ParseHeader(Stream stream)
    {
        var format = PlyFormat.Ascii;
        int vertexCount = 0;
        var properties = new List<PlyProperty>();
        bool inVertexElement = false;

        using var headerReader = new StreamReader(stream, Encoding.ASCII, leaveOpen: true);
        string? line;

        // Read "ply" magic
        line = headerReader.ReadLine();
        if (line?.Trim().ToLower() != "ply")
            throw new FormatException("Not a valid PLY file");

        while ((line = headerReader.ReadLine()) != null)
        {
            line = line.Trim();

            if (line.StartsWith("end_header", StringComparison.OrdinalIgnoreCase))
                break;

            if (line.StartsWith("format", StringComparison.OrdinalIgnoreCase))
            {
                var parts = line.Split(' ', StringSplitOptions.RemoveEmptyEntries);
                if (parts.Length >= 2)
                {
                    format = parts[1].ToLower() switch
                    {
                        "ascii" => PlyFormat.Ascii,
                        "binary_little_endian" => PlyFormat.BinaryLittleEndian,
                        "binary_big_endian" => PlyFormat.BinaryBigEndian,
                        _ => throw new FormatException($"Unknown PLY format: {parts[1]}")
                    };
                }
            }
            else if (line.StartsWith("element", StringComparison.OrdinalIgnoreCase))
            {
                var parts = line.Split(' ', StringSplitOptions.RemoveEmptyEntries);
                if (parts.Length >= 3 && parts[1].Equals("vertex", StringComparison.OrdinalIgnoreCase))
                {
                    vertexCount = int.Parse(parts[2], CultureInfo.InvariantCulture);
                    inVertexElement = true;
                }
                else
                {
                    inVertexElement = false;
                }
            }
            else if (line.StartsWith("property", StringComparison.OrdinalIgnoreCase) && inVertexElement)
            {
                var prop = ParseProperty(line);
                if (prop != null && !prop.IsList)
                {
                    properties.Add(prop);
                }
            }
            // Ignore comments and other elements
        }

        // Find end of header - need to re-scan for exact position
        stream.Position = 0;
        var headerBytes = new List<byte>();
        var endMarker = Encoding.ASCII.GetBytes("end_header");
        int matchIndex = 0;

        while (true)
        {
            int b = stream.ReadByte();
            if (b == -1) throw new FormatException("Unexpected end of file in header");

            headerBytes.Add((byte)b);

            if (b == endMarker[matchIndex])
            {
                matchIndex++;
                if (matchIndex == endMarker.Length)
                {
                    // Skip to end of line
                    while (true)
                    {
                        b = stream.ReadByte();
                        if (b == -1 || b == '\n') break;
                    }
                    break;
                }
            }
            else
            {
                matchIndex = 0;
            }
        }

        return (format, vertexCount, properties, stream.Position);
    }

    private PlyProperty? ParseProperty(string line)
    {
        var parts = line.Split(' ', StringSplitOptions.RemoveEmptyEntries);

        if (parts.Length < 3) return null;

        if (parts[1].Equals("list", StringComparison.OrdinalIgnoreCase))
        {
            // List properties (not used for vertex data typically)
            return parts.Length >= 5 ? new PlyProperty
            {
                IsList = true,
                ListCountType = ParseType(parts[2]),
                Type = ParseType(parts[3]),
                Name = parts[4]
            } : null;
        }

        return new PlyProperty
        {
            IsList = false,
            Type = ParseType(parts[1]),
            Name = parts[2]
        };
    }

    private PlyType ParseType(string typeName)
    {
        return typeName.ToLower() switch
        {
            "char" or "int8" => PlyType.Int8,
            "uchar" or "uint8" => PlyType.UInt8,
            "short" or "int16" => PlyType.Int16,
            "ushort" or "uint16" => PlyType.UInt16,
            "int" or "int32" => PlyType.Int32,
            "uint" or "uint32" => PlyType.UInt32,
            "float" or "float32" => PlyType.Float32,
            "double" or "float64" => PlyType.Float64,
            _ => PlyType.Float32
        };
    }

    private int GetTypeSize(PlyType type)
    {
        return type switch
        {
            PlyType.Int8 or PlyType.UInt8 or PlyType.Char or PlyType.UChar => 1,
            PlyType.Int16 or PlyType.UInt16 or PlyType.Short or PlyType.UShort => 2,
            PlyType.Int32 or PlyType.UInt32 or PlyType.Int or PlyType.UInt or PlyType.Float32 or PlyType.Float => 4,
            PlyType.Float64 or PlyType.Double => 8,
            _ => 4
        };
    }

    private void ReadAsciiData(Stream stream, PointCloud cloud, int count, List<PlyProperty> properties)
    {
        using var reader = new StreamReader(stream, Encoding.ASCII);

        // Build property index map
        var propMap = new Dictionary<string, int>();
        for (int i = 0; i < properties.Count; i++)
        {
            propMap[properties[i].Name.ToLower()] = i;
        }

        for (int i = 0; i < count; i++)
        {
            var line = reader.ReadLine();
            if (line == null) break;

            var values = line.Split(' ', StringSplitOptions.RemoveEmptyEntries);
            if (values.Length < properties.Count) continue;

            var point = ParsePointFromValues(values, propMap, properties);
            cloud.AddPoint(point);
        }
    }

    private void ReadBinaryData(BinaryReader reader, PointCloud cloud, int count, List<PlyProperty> properties, bool bigEndian)
    {
        // Build property index map
        var propMap = new Dictionary<string, int>();
        for (int i = 0; i < properties.Count; i++)
        {
            propMap[properties[i].Name.ToLower()] = i;
        }

        for (int i = 0; i < count; i++)
        {
            var values = new float[properties.Count];

            for (int j = 0; j < properties.Count; j++)
            {
                values[j] = ReadBinaryValue(reader, properties[j].Type, bigEndian);
            }

            var point = ParsePointFromFloatValues(values, propMap, properties);
            cloud.AddPoint(point);
        }
    }

    private float ReadBinaryValue(BinaryReader reader, PlyType type, bool bigEndian)
    {
        switch (type)
        {
            case PlyType.Int8 or PlyType.Char:
                return reader.ReadSByte();

            case PlyType.UInt8 or PlyType.UChar:
                return reader.ReadByte();

            case PlyType.Int16 or PlyType.Short:
                var s = reader.ReadInt16();
                return bigEndian ? ReverseEndianness(s) : s;

            case PlyType.UInt16 or PlyType.UShort:
                var us = reader.ReadUInt16();
                return bigEndian ? ReverseEndianness(us) : us;

            case PlyType.Int32 or PlyType.Int:
                var i = reader.ReadInt32();
                return bigEndian ? ReverseEndianness(i) : i;

            case PlyType.UInt32 or PlyType.UInt:
                var ui = reader.ReadUInt32();
                return bigEndian ? ReverseEndianness(ui) : ui;

            case PlyType.Float32 or PlyType.Float:
                if (bigEndian)
                {
                    var bytes = reader.ReadBytes(4);
                    Array.Reverse(bytes);
                    return BitConverter.ToSingle(bytes);
                }
                return reader.ReadSingle();

            case PlyType.Float64 or PlyType.Double:
                if (bigEndian)
                {
                    var bytes = reader.ReadBytes(8);
                    Array.Reverse(bytes);
                    return (float)BitConverter.ToDouble(bytes);
                }
                return (float)reader.ReadDouble();

            default:
                return reader.ReadSingle();
        }
    }

    private static short ReverseEndianness(short value) =>
        (short)((value & 0xFF) << 8 | (value >> 8) & 0xFF);

    private static ushort ReverseEndianness(ushort value) =>
        (ushort)((value & 0xFF) << 8 | (value >> 8) & 0xFF);

    private static int ReverseEndianness(int value) =>
        (int)(((uint)value & 0xFF) << 24 |
              ((uint)value & 0xFF00) << 8 |
              ((uint)value & 0xFF0000) >> 8 |
              ((uint)value >> 24) & 0xFF);

    private static uint ReverseEndianness(uint value) =>
        (value & 0xFF) << 24 |
        (value & 0xFF00) << 8 |
        (value & 0xFF0000) >> 8 |
        (value >> 24) & 0xFF;

    private PointData ParsePointFromValues(string[] values, Dictionary<string, int> propMap, List<PlyProperty> properties)
    {
        var floatValues = new float[values.Length];
        for (int i = 0; i < values.Length && i < properties.Count; i++)
        {
            if (float.TryParse(values[i], NumberStyles.Float, CultureInfo.InvariantCulture, out float f))
            {
                floatValues[i] = f;
            }
        }
        return ParsePointFromFloatValues(floatValues, propMap, properties);
    }

    private PointData ParsePointFromFloatValues(float[] values, Dictionary<string, int> propMap, List<PlyProperty> properties)
    {
        var point = new PointData();

        // Position (x, y, z)
        if (propMap.TryGetValue("x", out int xi)) point.Position.X = values[xi];
        if (propMap.TryGetValue("y", out int yi)) point.Position.Y = values[yi];
        if (propMap.TryGetValue("z", out int zi)) point.Position.Z = values[zi];

        // Color (red/r, green/g, blue/b, alpha/a)
        // Use actual PLY data type for correct normalization (uchar=0-255, ushort=0-65535, float=0-1)
        float r = 1f, g = 1f, b = 1f, a = 1f;

        if (propMap.TryGetValue("red", out int ri))
            r = NormalizeColorChannel(values[ri], properties[ri].Type);
        else if (propMap.TryGetValue("r", out ri))
            r = NormalizeColorChannel(values[ri], properties[ri].Type);

        if (propMap.TryGetValue("green", out int gi))
            g = NormalizeColorChannel(values[gi], properties[gi].Type);
        else if (propMap.TryGetValue("g", out gi))
            g = NormalizeColorChannel(values[gi], properties[gi].Type);

        if (propMap.TryGetValue("blue", out int bi))
            b = NormalizeColorChannel(values[bi], properties[bi].Type);
        else if (propMap.TryGetValue("b", out bi))
            b = NormalizeColorChannel(values[bi], properties[bi].Type);

        if (propMap.TryGetValue("alpha", out int ai))
            a = NormalizeColorChannel(values[ai], properties[ai].Type);
        else if (propMap.TryGetValue("a", out ai))
            a = NormalizeColorChannel(values[ai], properties[ai].Type);

        point.Color = new Color4(r, g, b, a);

        // Normals (nx, ny, nz)
        if (propMap.TryGetValue("nx", out int nxi)) point.Normal.X = values[nxi];
        if (propMap.TryGetValue("ny", out int nyi)) point.Normal.Y = values[nyi];
        if (propMap.TryGetValue("nz", out int nzi)) point.Normal.Z = values[nzi];

        // Intensity - use type-aware normalization
        if (propMap.TryGetValue("intensity", out int ii))
        {
            point.Intensity = NormalizeIntensity(values[ii], properties[ii].Type);
        }
        else if (propMap.TryGetValue("scalar_intensity", out ii))
        {
            point.Intensity = NormalizeIntensity(values[ii], properties[ii].Type);
        }

        // Additional scalars
        var knownProps = new HashSet<string>
        {
            "x", "y", "z",
            "red", "green", "blue", "alpha", "r", "g", "b", "a",
            "nx", "ny", "nz",
            "intensity", "scalar_intensity"
        };

        foreach (var kvp in propMap)
        {
            if (!knownProps.Contains(kvp.Key))
            {
                point.SetScalar(kvp.Key, values[kvp.Value]);
            }
        }

        return point;
    }

    /// <summary>
    /// Normalizes a color channel value to 0-1 range based on the actual PLY data type.
    /// </summary>
    private static float NormalizeColorChannel(float rawValue, PlyType type)
    {
        return type switch
        {
            PlyType.UInt8 or PlyType.UChar or PlyType.Int8 or PlyType.Char => rawValue / 255f,
            PlyType.UInt16 or PlyType.UShort or PlyType.Int16 or PlyType.Short => rawValue / 65535f,
            PlyType.UInt32 or PlyType.UInt or PlyType.Int32 or PlyType.Int => rawValue / 255f,
            // Float types: use heuristic (could be 0-1 already, or 0-255 from some exporters)
            _ => rawValue > 1f ? rawValue / 255f : rawValue
        };
    }

    /// <summary>
    /// Normalizes an intensity value to 0-1 range based on the actual PLY data type.
    /// </summary>
    private static float NormalizeIntensity(float rawValue, PlyType type)
    {
        return type switch
        {
            PlyType.UInt8 or PlyType.UChar or PlyType.Int8 or PlyType.Char => rawValue / 255f,
            PlyType.UInt16 or PlyType.UShort or PlyType.Int16 or PlyType.Short => rawValue / 65535f,
            PlyType.UInt32 or PlyType.UInt or PlyType.Int32 or PlyType.Int => rawValue / 65535f,
            // Float types: use heuristic
            _ => rawValue > 1f ? rawValue / 65535f : rawValue
        };
    }
}

