namespace OctreeFlow.Core;

/// <summary>
/// Configuration settings for octree generation.
/// </summary>
public class OctreeConfiguration
{
    /// <summary>
    /// Number of points per node (N).
    /// Default: 1000
    /// </summary>
    public int PointsPerNode { get; set; } = 1000;

    /// <summary>
    /// Starting minimum distance between points at level 0.
    /// Points closer than this threshold will be discarded during selection.
    /// </summary>
    public float StartingMinDistance { get; set; } = 1.0f;

    /// <summary>
    /// Ratio to multiply the distance threshold for each subsequent level.
    /// Default: 0.5 (halves the distance at each level).
    /// </summary>
    public float LevelThresholdRatio { get; set; } = 0.5f;

    /// <summary>
    /// Random seed for reproducible point selection.
    /// Set to null for non-deterministic behavior.
    /// </summary>
    public int? RandomSeed { get; set; } = null;

    /// <summary>
    /// Maximum depth of the octree. Set to 0 for unlimited.
    /// </summary>
    public int MaxDepth { get; set; } = 0;

    /// <summary>
    /// Minimum number of points required to create a node.
    /// Nodes with fewer available points will not be subdivided.
    /// </summary>
    public int MinPointsForNode { get; set; } = 1;

    /// <summary>
    /// Maximum attempts to find a valid point before giving up on filling the node.
    /// Prevents infinite loops when points are sparse.
    /// </summary>
    public int MaxSelectionAttempts { get; set; } = 10000;

    /// <summary>
    /// Gets the distance threshold for a specific level.
    /// </summary>
    public float GetDistanceThreshold(int level)
    {
        return StartingMinDistance * MathF.Pow(LevelThresholdRatio, level);
    }

    /// <summary>
    /// Creates a default configuration.
    /// </summary>
    public static OctreeConfiguration Default => new();

    /// <summary>
    /// Creates a configuration optimized for large point clouds.
    /// </summary>
    public static OctreeConfiguration LargeCloud => new()
    {
        PointsPerNode = 5000,
        StartingMinDistance = 2.0f,
        LevelThresholdRatio = 0.5f,
        MaxSelectionAttempts = 50000
    };

    /// <summary>
    /// Creates a configuration optimized for dense/detailed point clouds.
    /// </summary>
    public static OctreeConfiguration HighDetail => new()
    {
        PointsPerNode = 500,
        StartingMinDistance = 0.1f,
        LevelThresholdRatio = 0.6f,
        MaxSelectionAttempts = 20000
    };

    /// <summary>
    /// Validates the configuration and throws if invalid.
    /// </summary>
    public void Validate()
    {
        if (PointsPerNode <= 0)
            throw new ArgumentException("PointsPerNode must be greater than 0");

        if (StartingMinDistance <= 0)
            throw new ArgumentException("StartingMinDistance must be greater than 0");

        if (LevelThresholdRatio <= 0 || LevelThresholdRatio >= 1)
            throw new ArgumentException("LevelThresholdRatio must be between 0 and 1 (exclusive)");

        if (MaxDepth < 0)
            throw new ArgumentException("MaxDepth must be 0 (unlimited) or greater");

        if (MinPointsForNode <= 0)
            throw new ArgumentException("MinPointsForNode must be greater than 0");

        if (MaxSelectionAttempts <= 0)
            throw new ArgumentException("MaxSelectionAttempts must be greater than 0");
    }

    /// <summary>
    /// Returns a copy of this configuration.
    /// </summary>
    public OctreeConfiguration Clone()
    {
        return new OctreeConfiguration
        {
            PointsPerNode = PointsPerNode,
            StartingMinDistance = StartingMinDistance,
            LevelThresholdRatio = LevelThresholdRatio,
            RandomSeed = RandomSeed,
            MaxDepth = MaxDepth,
            MinPointsForNode = MinPointsForNode,
            MaxSelectionAttempts = MaxSelectionAttempts
        };
    }
}

