using Stride.Core.Mathematics;

namespace OctreeFlow.Data;

/// <summary>
/// Represents a single point with position, color, and optional attributes.
/// Designed to handle PLY files with varying properties (normals, intensity, scalars).
/// </summary>
public struct PointData
{
    /// <summary>
    /// Unique identifier for this point.
    /// </summary>
    public int Id;

    /// <summary>
    /// 3D position of the point.
    /// </summary>
    public Vector3 Position;

    /// <summary>
    /// RGBA color of the point (0-255 per channel).
    /// </summary>
    public Color4 Color;

    /// <summary>
    /// Normal vector (optional, may be zero if not present in source).
    /// </summary>
    public Vector3 Normal;

    /// <summary>
    /// Intensity value (common in LiDAR data, 0-1 normalized).
    /// </summary>
    public float Intensity;

    /// <summary>
    /// Additional scalar values from PLY file.
    /// Keys are property names from the PLY header.
    /// </summary>
    public Dictionary<string, float>? Scalars;

    public PointData(Vector3 position)
    {
        Id = 0;
        Position = position;
        Color = new Color4(1f, 1f, 1f, 1f);
        Normal = Vector3.Zero;
        Intensity = 1f;
        Scalars = null;
    }

    public PointData(Vector3 position, Color4 color)
    {
        Id = 0;
        Position = position;
        Color = color;
        Normal = Vector3.Zero;
        Intensity = 1f;
        Scalars = null;
    }

    public PointData(Vector3 position, Color4 color, Vector3 normal)
    {
        Id = 0;
        Position = position;
        Color = color;
        Normal = normal;
        Intensity = 1f;
        Scalars = null;
    }

    public PointData(int id, Vector3 position, Color4 color)
    {
        Id = id;
        Position = position;
        Color = color;
        Normal = Vector3.Zero;
        Intensity = 1f;
        Scalars = null;
    }

    /// <summary>
    /// Gets or sets a scalar value by name.
    /// </summary>
    public float GetScalar(string name, float defaultValue = 0f)
    {
        if (Scalars == null || !Scalars.TryGetValue(name, out var value))
            return defaultValue;
        return value;
    }

    /// <summary>
    /// Sets a scalar value by name.
    /// </summary>
    public void SetScalar(string name, float value)
    {
        Scalars ??= new Dictionary<string, float>();
        Scalars[name] = value;
    }
}

