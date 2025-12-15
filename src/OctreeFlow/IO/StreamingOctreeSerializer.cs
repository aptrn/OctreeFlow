using System.Text.Json;
using System.Text.Json.Serialization;
using Stride.Core.Mathematics;
using OctreeFlow.Core;

namespace OctreeFlow.IO;

/// <summary>
/// Serializes octree structure to .octree file.
/// The .octree file contains only the structure with point indices - no point data.
/// Point indices reference the original PLY file, keeping the .octree file small.
/// 
/// File format (Version 4):
/// - Magic: "OCTR" (4 bytes)
/// - Version: int32
/// - Total points in PLY: int32
/// - PLY path length + path: int32 + UTF8 bytes
/// - Property count: int32
///   - For each property: length + name (int32 + UTF8 bytes)
/// - Bounds: 6 x float32 (min.xyz, max.xyz)
/// - Node count: int32
/// - Nodes (written recursively, depth-first):
///   - ID length + ID: int32 + UTF8 bytes
///   - Level: int32
///   - BoundingBox: 6 x float32
///   - Point count: int32
///   - Point indices: N x int32 (raw binary)
///   - Child count: int32
///   - Children: recursively written
/// </summary>
public class StreamingOctreeSerializer
{
    private static readonly byte[] Magic = { 0x4F, 0x43, 0x54, 0x52 }; // "OCTR"
    private const int Version = 4; // Version 4: compact binary format with streaming writes

    private readonly JsonSerializerOptions _jsonOptions;

    public Action<int, int>? OnProgress { get; set; }

    public StreamingOctreeSerializer()
    {
        _jsonOptions = new JsonSerializerOptions
        {
            WriteIndented = true,
            PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
            DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull
        };
    }

    /// <summary>
    /// Saves the octree structure as JSON (lightweight - no point indices).
    /// Point indices are stored only in the .octree binary file.
    /// </summary>
    public void SaveStructureJson(OctreeNode root, string filePath)
    {
        var json = JsonSerializer.Serialize(root, _jsonOptions);
        File.WriteAllText(filePath, json);
    }

    /// <summary>
    /// Saves the octree structure to .octree file using compact binary format.
    /// Only stores the structure with point indices - no point data.
    /// Point indices reference the original PLY file.
    /// Uses streaming writes to avoid memory issues with large octrees.
    /// </summary>
    public void SaveOctreeFile(OctreeNode root, PlyIndex plyIndex, string plyPath, string outputPath)
    {
        int totalPoints = plyIndex.VertexCount;
        int nodeCount = root.GetTotalNodeCount();
        int nodesWritten = 0;

        using var outputStream = new FileStream(outputPath, FileMode.Create, FileAccess.Write, FileShare.None, 65536);
        using var writer = new BinaryWriter(outputStream);

        // Write header
        writer.Write(Magic);
        writer.Write(Version);
        writer.Write(totalPoints);

        // Write PLY file path (relative or absolute) for reference
        WriteString(writer, plyPath);

        // Write property names from PLY file
        var propNames = plyIndex.Properties.Select(p => p.Name).ToList();
        writer.Write(propNames.Count);
        foreach (var name in propNames)
        {
            WriteString(writer, name);
        }

        // Write bounding box
        var bounds = plyIndex.Bounds;
        writer.Write(bounds.Minimum.X);
        writer.Write(bounds.Minimum.Y);
        writer.Write(bounds.Minimum.Z);
        writer.Write(bounds.Maximum.X);
        writer.Write(bounds.Maximum.Y);
        writer.Write(bounds.Maximum.Z);

        // Write node count
        writer.Write(nodeCount);

        // Write nodes recursively (depth-first) - streams directly to disk
        WriteNode(writer, root, ref nodesWritten, nodeCount);

        // Ensure everything is flushed
        writer.Flush();
        outputStream.Flush();
    }

    /// <summary>
    /// Writes a single node and its children recursively.
    /// </summary>
    private void WriteNode(BinaryWriter writer, OctreeNode node, ref int nodesWritten, int totalNodes)
    {
        // Write node ID
        WriteString(writer, node.Id);

        // Write level
        writer.Write(node.Level);

        // Write bounding box
        writer.Write(node.BoundingBox.Minimum.X);
        writer.Write(node.BoundingBox.Minimum.Y);
        writer.Write(node.BoundingBox.Minimum.Z);
        writer.Write(node.BoundingBox.Maximum.X);
        writer.Write(node.BoundingBox.Maximum.Y);
        writer.Write(node.BoundingBox.Maximum.Z);

        // Write point indices count
        int pointCount = node.PointIndices?.Count ?? 0;
        writer.Write(pointCount);

        // Write point indices as raw binary (4 bytes each)
        if (node.PointIndices != null && pointCount > 0)
        {
            foreach (int idx in node.PointIndices)
            {
                writer.Write(idx);
            }
        }

        // Write children count
        int childCount = node.Children?.Count ?? 0;
        writer.Write(childCount);

        // Update progress
        nodesWritten++;
        if (nodesWritten % 1000 == 0 || nodesWritten == totalNodes)
        {
            OnProgress?.Invoke(nodesWritten, totalNodes);
        }

        // Write children recursively
        if (node.Children != null)
        {
            foreach (var child in node.Children)
            {
                WriteNode(writer, child, ref nodesWritten, totalNodes);
            }
        }
    }

    /// <summary>
    /// Writes a string with length prefix.
    /// </summary>
    private static void WriteString(BinaryWriter writer, string value)
    {
        var bytes = System.Text.Encoding.UTF8.GetBytes(value);
        writer.Write(bytes.Length);
        writer.Write(bytes);
    }

    /// <summary>
    /// Reads a string with length prefix.
    /// </summary>
    private static string ReadString(BinaryReader reader)
    {
        int length = reader.ReadInt32();
        var bytes = reader.ReadBytes(length);
        return System.Text.Encoding.UTF8.GetString(bytes);
    }

    /// <summary>
    /// Loads octree structure from JSON (does not include point indices).
    /// Use LoadOctreeFile to load the complete structure with indices.
    /// </summary>
    public OctreeNode? LoadStructureJson(string filePath)
    {
        var json = File.ReadAllText(filePath);
        return JsonSerializer.Deserialize<OctreeNode>(json, _jsonOptions);
    }

    /// <summary>
    /// Loads the complete octree from a .octree file including point indices.
    /// </summary>
    public (OctreeNode? Root, OctreeFileInfo Info) LoadOctreeFile(string filePath)
    {
        using var inputStream = File.OpenRead(filePath);
        using var reader = new BinaryReader(inputStream);

        // Read and verify magic
        var magic = reader.ReadBytes(4);
        if (!magic.SequenceEqual(Magic))
            throw new FormatException("Invalid .octree file format - magic number mismatch");

        // Read version
        int version = reader.ReadInt32();
        if (version != Version)
            throw new FormatException($"Unsupported .octree file version: {version} (expected {Version})");

        // Read header info
        int totalPoints = reader.ReadInt32();
        string plyPath = ReadString(reader);

        // Read property names
        int propCount = reader.ReadInt32();
        var propertyNames = new List<string>();
        for (int i = 0; i < propCount; i++)
        {
            propertyNames.Add(ReadString(reader));
        }

        // Read bounds
        var bounds = new BoundingBox(
            new Vector3(reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle()),
            new Vector3(reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle())
        );

        // Read node count
        int nodeCount = reader.ReadInt32();

        // Read root node (recursively reads all nodes)
        var root = ReadNode(reader);

        var info = new OctreeFileInfo
        {
            Version = version,
            TotalPoints = totalPoints,
            PlyPath = plyPath,
            PropertyNames = propertyNames,
            Bounds = bounds,
            NodeCount = nodeCount
        };

        return (root, info);
    }

    /// <summary>
    /// Reads a single node and its children recursively.
    /// </summary>
    private OctreeNode ReadNode(BinaryReader reader)
    {
        // Read node ID
        string id = ReadString(reader);

        // Read level
        int level = reader.ReadInt32();

        // Read bounding box
        var boundingBox = new BoundingBox(
            new Vector3(reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle()),
            new Vector3(reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle())
        );

        // Create node
        var node = new OctreeNode(id, boundingBox, level);

        // Read point indices count
        int pointCount = reader.ReadInt32();

        // Read point indices
        for (int i = 0; i < pointCount; i++)
        {
            node.AddPointIndex(reader.ReadInt32());
        }

        // Read children count
        int childCount = reader.ReadInt32();

        // Read children recursively
        for (int i = 0; i < childCount; i++)
        {
            node.AddChild(ReadNode(reader));
        }

        return node;
    }
}

/// <summary>
/// Information about an .octree file.
/// </summary>
public class OctreeFileInfo
{
    public int Version { get; set; }
    public int TotalPoints { get; set; }
    public string PlyPath { get; set; } = "";
    public List<string> PropertyNames { get; set; } = new();
    public BoundingBox Bounds { get; set; }
    public int NodeCount { get; set; }
}
