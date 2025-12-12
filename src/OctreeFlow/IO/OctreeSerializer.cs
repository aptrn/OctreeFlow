using System.Runtime.InteropServices;
using System.Text.Json;
using System.Text.Json.Serialization;
using Stride.Core.Mathematics;
using OctreeFlow.Core;
using OctreeFlow.Data;

namespace OctreeFlow.IO;

/// <summary>
/// Serializes and deserializes octree structures to JSON and binary .octree formats.
/// </summary>
public class OctreeSerializer
{
    // Magic number for .octree files: "OCTR"
    private static readonly byte[] Magic = { 0x4F, 0x43, 0x54, 0x52 };
    private const int Version = 1;

    /// <summary>
    /// Point data structure for binary serialization.
    /// </summary>
    [StructLayout(System.Runtime.InteropServices.LayoutKind.Sequential, Pack = 1)]
    private struct BinaryPointData
    {
        public float X, Y, Z;           // Position (12 bytes)
        public float R, G, B, A;        // Color (16 bytes)
        public float NX, NY, NZ;        // Normal (12 bytes)
        public float Intensity;         // Intensity (4 bytes)
        // Total: 44 bytes per point
    }

    private readonly JsonSerializerOptions _jsonOptions;

    public OctreeSerializer()
    {
        _jsonOptions = new JsonSerializerOptions
        {
            WriteIndented = true,
            PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
            DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull
        };
    }

    /// <summary>
    /// Saves the octree structure as JSON.
    /// </summary>
    public void SaveStructureJson(OctreeNode root, string filePath)
    {
        var json = JsonSerializer.Serialize(root, _jsonOptions);
        File.WriteAllText(filePath, json);
    }

    /// <summary>
    /// Loads an octree structure from JSON.
    /// </summary>
    public OctreeNode? LoadStructureJson(string filePath)
    {
        var json = File.ReadAllText(filePath);
        return JsonSerializer.Deserialize<OctreeNode>(json, _jsonOptions);
    }

    /// <summary>
    /// Saves the complete octree with point data to a .octree binary file.
    /// </summary>
    public void SaveOctreeFile(OctreeNode root, PointCloud cloud, string filePath)
    {
        using var stream = File.Create(filePath);
        using var writer = new BinaryWriter(stream);

        // Write header
        writer.Write(Magic);
        writer.Write(Version);

        // Write point count
        writer.Write(cloud.TotalCount);

        // Write property names
        writer.Write(cloud.PropertyNames.Count);
        foreach (var name in cloud.PropertyNames)
        {
            writer.Write(name);
        }

        // Write bounding box
        var bounds = cloud.Bounds;
        writer.Write(bounds.Minimum.X);
        writer.Write(bounds.Minimum.Y);
        writer.Write(bounds.Minimum.Z);
        writer.Write(bounds.Maximum.X);
        writer.Write(bounds.Maximum.Y);
        writer.Write(bounds.Maximum.Z);

        // Placeholder for data offset
        long dataOffsetPosition = stream.Position;
        writer.Write((long)0);

        // Serialize structure
        var structureJson = JsonSerializer.Serialize(root, _jsonOptions);
        var structureBytes = System.Text.Encoding.UTF8.GetBytes(structureJson);
        writer.Write(structureBytes.Length);
        writer.Write(structureBytes);

        // Record data start position
        long dataStartPosition = stream.Position;

        // Go back and write the data offset
        stream.Position = dataOffsetPosition;
        writer.Write(dataStartPosition);
        stream.Position = dataStartPosition;

        // Write all point data sequentially
        WritePointData(writer, cloud, root);
    }

    /// <summary>
    /// Writes point data for the tree in depth-first order, recording offsets.
    /// </summary>
    private void WritePointData(BinaryWriter writer, PointCloud cloud, OctreeNode node)
    {
        // Record offset
        node.DataOffset = writer.BaseStream.Position;

        // Write points for this node
        foreach (var idx in node.PointIndices)
        {
            var point = cloud.GetPoint(idx);
            WritePoint(writer, point);
        }

        node.DataSize = (int)(writer.BaseStream.Position - node.DataOffset);

        // Recursively write children
        foreach (var child in node.Children)
        {
            WritePointData(writer, cloud, child);
        }
    }

    private void WritePoint(BinaryWriter writer, PointData point)
    {
        // Position
        writer.Write(point.Position.X);
        writer.Write(point.Position.Y);
        writer.Write(point.Position.Z);

        // Color
        writer.Write(point.Color.R);
        writer.Write(point.Color.G);
        writer.Write(point.Color.B);
        writer.Write(point.Color.A);

        // Normal
        writer.Write(point.Normal.X);
        writer.Write(point.Normal.Y);
        writer.Write(point.Normal.Z);

        // Intensity
        writer.Write(point.Intensity);

        // Scalars count and data
        int scalarCount = point.Scalars?.Count ?? 0;
        writer.Write(scalarCount);

        if (point.Scalars != null)
        {
            foreach (var kvp in point.Scalars)
            {
                writer.Write(kvp.Key);
                writer.Write(kvp.Value);
            }
        }
    }

    /// <summary>
    /// Loads a .octree file header and structure (without loading all point data).
    /// </summary>
    public (OctreeNode? root, OctreeFileHeader header) LoadOctreeFileHeader(string filePath)
    {
        using var stream = File.OpenRead(filePath);
        using var reader = new BinaryReader(stream);

        var header = ReadHeader(reader);

        // Read structure JSON length and content
        int structureLength = reader.ReadInt32();
        var structureBytes = reader.ReadBytes(structureLength);
        var structureJson = System.Text.Encoding.UTF8.GetString(structureBytes);

        var root = JsonSerializer.Deserialize<OctreeNode>(structureJson, _jsonOptions);

        return (root, header);
    }

    /// <summary>
    /// Loads point data for a specific node from the .octree file.
    /// </summary>
    public List<PointData> LoadNodePoints(string filePath, OctreeNode node)
    {
        using var stream = File.OpenRead(filePath);
        using var reader = new BinaryReader(stream);

        // Seek to node's data
        stream.Position = node.DataOffset;

        var points = new List<PointData>();
        for (int i = 0; i < node.PointIndices.Count; i++)
        {
            points.Add(ReadPoint(reader));
        }

        return points;
    }

    private OctreeFileHeader ReadHeader(BinaryReader reader)
    {
        var header = new OctreeFileHeader();

        // Verify magic
        var magic = reader.ReadBytes(4);
        if (!magic.SequenceEqual(Magic))
            throw new FormatException("Invalid .octree file format");

        header.Version = reader.ReadInt32();
        header.PointCount = reader.ReadInt32();

        // Property names
        int propCount = reader.ReadInt32();
        for (int i = 0; i < propCount; i++)
        {
            header.PropertyNames.Add(reader.ReadString());
        }

        // Bounding box
        header.Bounds = new BoundingBox(
            new Vector3(reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle()),
            new Vector3(reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle())
        );

        header.DataOffset = reader.ReadInt64();

        return header;
    }

    private PointData ReadPoint(BinaryReader reader)
    {
        var point = new PointData
        {
            Position = new Vector3(
                reader.ReadSingle(),
                reader.ReadSingle(),
                reader.ReadSingle()),
            Color = new Color4(
                reader.ReadSingle(),
                reader.ReadSingle(),
                reader.ReadSingle(),
                reader.ReadSingle()),
            Normal = new Vector3(
                reader.ReadSingle(),
                reader.ReadSingle(),
                reader.ReadSingle()),
            Intensity = reader.ReadSingle()
        };

        // Read scalars
        int scalarCount = reader.ReadInt32();
        if (scalarCount > 0)
        {
            point.Scalars = new Dictionary<string, float>();
            for (int i = 0; i < scalarCount; i++)
            {
                string key = reader.ReadString();
                float value = reader.ReadSingle();
                point.Scalars[key] = value;
            }
        }

        return point;
    }
}

/// <summary>
/// Header information from an .octree file.
/// </summary>
public class OctreeFileHeader
{
    public int Version { get; set; }
    public int PointCount { get; set; }
    public List<string> PropertyNames { get; set; } = new();
    public BoundingBox Bounds { get; set; }
    public long DataOffset { get; set; }
}

