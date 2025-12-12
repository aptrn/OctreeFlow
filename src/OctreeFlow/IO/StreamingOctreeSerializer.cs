using System.Text.Json;
using System.Text.Json.Serialization;
using Stride.Core.Mathematics;
using OctreeFlow.Core;

namespace OctreeFlow.IO;

/// <summary>
/// Memory-efficient serializer that streams point data directly from PLY to .octree file.
/// Points are written in PLY order; node structure stores indices for lookup.
/// </summary>
public class StreamingOctreeSerializer
{
    private static readonly byte[] Magic = { 0x4F, 0x43, 0x54, 0x52 }; // "OCTR"
    private const int Version = 2; // Version 2: points in PLY order with index lookup

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
    /// Saves the octree structure as JSON.
    /// </summary>
    public void SaveStructureJson(OctreeNode root, string filePath)
    {
        var json = JsonSerializer.Serialize(root, _jsonOptions);
        File.WriteAllText(filePath, json);
    }

    /// <summary>
    /// Saves the complete octree by streaming from PLY file.
    /// Points are stored in PLY order (sequential write).
    /// Memory efficient - constant memory usage regardless of point count.
    /// </summary>
    public void SaveOctreeFile(OctreeNode root, PlyIndex plyIndex, string plyPath, string outputPath)
    {
        int totalPoints = plyIndex.VertexCount;

        using var outputStream = File.Create(outputPath);
        using var writer = new BinaryWriter(outputStream);

        // Write header
        writer.Write(Magic);
        writer.Write(Version);
        writer.Write(totalPoints);

        // Write property names
        var propNames = plyIndex.Properties.Select(p => p.Name).ToList();
        writer.Write(propNames.Count);
        foreach (var name in propNames)
        {
            writer.Write(name);
        }

        // Write bounding box
        var bounds = plyIndex.Bounds;
        writer.Write(bounds.Minimum.X);
        writer.Write(bounds.Minimum.Y);
        writer.Write(bounds.Minimum.Z);
        writer.Write(bounds.Maximum.X);
        writer.Write(bounds.Maximum.Y);
        writer.Write(bounds.Maximum.Z);

        // Calculate fixed point size (we'll use fixed-size format for efficient seeking)
        int scalarCount = Math.Max(0, propNames.Count - 10); // Excluding x,y,z,r,g,b,a,nx,ny,nz,intensity
        int pointSize = CalculateFixedPointSize(scalarCount);
        writer.Write(pointSize);

        // Write data offset placeholder
        long dataOffsetPosition = outputStream.Position;
        writer.Write((long)0);

        // Write structure
        var structureJson = JsonSerializer.Serialize(root, _jsonOptions);
        var structureBytes = System.Text.Encoding.UTF8.GetBytes(structureJson);
        writer.Write(structureBytes.Length);
        writer.Write(structureBytes);

        // Record data start and update offset
        long dataStartPosition = outputStream.Position;
        outputStream.Position = dataOffsetPosition;
        writer.Write(dataStartPosition);
        outputStream.Position = dataStartPosition;

        // Stream all points from PLY in order (single sequential pass)
        int writtenPoints = 0;
        int reportInterval = Math.Max(1, totalPoints / 100);

        // Build scalar name list (excluding standard properties)
        var scalarNames = propNames
            .Where(n => !IsStandardProperty(n.ToLower()))
            .ToList();

        using var plyReader = new PlyIndex(plyPath);
        plyReader.BuildIndex();

        plyReader.StreamVertices((index, position, values) =>
        {
            WritePointDataFixed(writer, position, values, plyReader.Properties, scalarNames, scalarCount);
            writtenPoints++;

            if (writtenPoints % reportInterval == 0)
                OnProgress?.Invoke(writtenPoints, totalPoints);
        });

        OnProgress?.Invoke(totalPoints, totalPoints);
    }

    private bool IsStandardProperty(string name)
    {
        return name switch
        {
            "x" or "y" or "z" => true,
            "red" or "green" or "blue" or "alpha" => true,
            "r" or "g" or "b" or "a" => true,
            "nx" or "ny" or "nz" => true,
            "intensity" or "scalar_intensity" => true,
            _ => false
        };
    }

    private int CalculateFixedPointSize(int scalarCount)
    {
        // Fixed layout:
        // Position: 12 bytes (3 * float)
        // Color: 16 bytes (4 * float)
        // Normal: 12 bytes (3 * float)
        // Intensity: 4 bytes (float)
        // Scalars: scalarCount * 4 bytes (floats only, names stored in header)
        return 12 + 16 + 12 + 4 + (scalarCount * 4);
    }

    private void WritePointDataFixed(BinaryWriter writer, Vector3 position, float[] values, 
        List<PlyProperty> properties, List<string> scalarNames, int scalarCount)
    {
        // Position
        writer.Write(position.X);
        writer.Write(position.Y);
        writer.Write(position.Z);

        // Extract standard properties with defaults
        float r = 1f, g = 1f, b = 1f, a = 1f;
        float nx = 0, ny = 0, nz = 0;
        float intensity = 1f;

        var scalarValues = new float[scalarCount];
        int scalarIdx = 0;

        for (int i = 0; i < properties.Count && i < values.Length; i++)
        {
            var prop = properties[i];
            var val = values[i];
            var name = prop.Name.ToLower();

            switch (name)
            {
                case "x" or "y" or "z":
                    break;
                case "red" or "r":
                    r = val > 1 ? val / 255f : val;
                    break;
                case "green" or "g":
                    g = val > 1 ? val / 255f : val;
                    break;
                case "blue" or "b":
                    b = val > 1 ? val / 255f : val;
                    break;
                case "alpha" or "a":
                    a = val > 1 ? val / 255f : val;
                    break;
                case "nx":
                    nx = val;
                    break;
                case "ny":
                    ny = val;
                    break;
                case "nz":
                    nz = val;
                    break;
                case "intensity" or "scalar_intensity":
                    intensity = val > 1 ? val / 65535f : val;
                    break;
                default:
                    if (scalarIdx < scalarCount)
                        scalarValues[scalarIdx++] = val;
                    break;
            }
        }

        // Write color
        writer.Write(r);
        writer.Write(g);
        writer.Write(b);
        writer.Write(a);

        // Write normal
        writer.Write(nx);
        writer.Write(ny);
        writer.Write(nz);

        // Write intensity
        writer.Write(intensity);

        // Write scalars (fixed count, stored as raw floats)
        for (int i = 0; i < scalarCount; i++)
        {
            writer.Write(scalarValues[i]);
        }
    }

    /// <summary>
    /// Loads octree structure from JSON.
    /// </summary>
    public OctreeNode? LoadStructureJson(string filePath)
    {
        var json = File.ReadAllText(filePath);
        return JsonSerializer.Deserialize<OctreeNode>(json, _jsonOptions);
    }
}
