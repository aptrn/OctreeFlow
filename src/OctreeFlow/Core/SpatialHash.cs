using Stride.Core.Mathematics;
using System.Runtime.CompilerServices;

namespace OctreeFlow.Core;

/// <summary>
/// Spatial hash grid for O(1) distance proximity queries.
/// Used to efficiently check if a point is within distance threshold of any existing point.
/// </summary>
public class SpatialHash
{
    private readonly Dictionary<long, List<Vector3>> _cells;
    private readonly float _cellSize;
    private readonly float _distanceThreshold;
    private readonly float _distanceThresholdSq;
    private readonly float _inverseCellSize;

    /// <summary>
    /// Number of points currently in the hash.
    /// </summary>
    public int Count { get; private set; }

    /// <summary>
    /// Creates a spatial hash with the given distance threshold.
    /// Cell size is set to the threshold for optimal query performance.
    /// </summary>
    public SpatialHash(float distanceThreshold)
    {
        _distanceThreshold = distanceThreshold;
        _distanceThresholdSq = distanceThreshold * distanceThreshold;
        _cellSize = distanceThreshold; // Cell size = threshold for optimal queries
        _inverseCellSize = 1.0f / distanceThreshold; // Pre-compute for faster division
        _cells = new Dictionary<long, List<Vector3>>(256); // Initial capacity
        Count = 0;
    }

    /// <summary>
    /// Creates a spatial hash with the given distance threshold and expected point count.
    /// </summary>
    public SpatialHash(float distanceThreshold, int expectedPointCount)
    {
        _distanceThreshold = distanceThreshold;
        _distanceThresholdSq = distanceThreshold * distanceThreshold;
        _cellSize = distanceThreshold;
        _inverseCellSize = 1.0f / distanceThreshold;
        // Estimate number of cells based on expected points (assume ~2 points per cell on average)
        int estimatedCells = Math.Max(256, expectedPointCount / 2);
        _cells = new Dictionary<long, List<Vector3>>(estimatedCells);
        Count = 0;
    }

    /// <summary>
    /// Computes the cell key for a position.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private long GetCellKey(Vector3 pos)
    {
        int x = (int)MathF.Floor(pos.X * _inverseCellSize);
        int y = (int)MathF.Floor(pos.Y * _inverseCellSize);
        int z = (int)MathF.Floor(pos.Z * _inverseCellSize);

        // Pack into 64-bit key (21 bits each, allows ~2M range per axis)
        const long mask = 0x1FFFFF; // 21 bits
        return ((long)(x & mask) << 42) | ((long)(y & mask) << 21) | (long)(z & mask);
    }

    /// <summary>
    /// Gets cell coordinates for a position.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private (int x, int y, int z) GetCellCoords(Vector3 pos)
    {
        return (
            (int)MathF.Floor(pos.X * _inverseCellSize),
            (int)MathF.Floor(pos.Y * _inverseCellSize),
            (int)MathF.Floor(pos.Z * _inverseCellSize)
        );
    }

    /// <summary>
    /// Gets cell key from coordinates.
    /// </summary>
    private long GetCellKeyFromCoords(int x, int y, int z)
    {
        const long mask = 0x1FFFFF;
        return ((long)(x & mask) << 42) | ((long)(y & mask) << 21) | (long)(z & mask);
    }

    /// <summary>
    /// Adds a point to the spatial hash.
    /// </summary>
    public void Add(Vector3 position)
    {
        long key = GetCellKey(position);

        if (!_cells.TryGetValue(key, out var cell))
        {
            cell = new List<Vector3>();
            _cells[key] = cell;
        }

        cell.Add(position);
        Count++;
    }

    /// <summary>
    /// Checks if a point is within the distance threshold of any existing point.
    /// Returns true if the point is TOO CLOSE to an existing point.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool HasNearbyPoint(Vector3 position)
    {
        var (cx, cy, cz) = GetCellCoords(position);

        // Check 3x3x3 neighborhood of cells (27 cells total)
        // Unrolled inner loops for better performance
        for (int dx = -1; dx <= 1; dx++)
        {
            int nx = cx + dx;
            for (int dy = -1; dy <= 1; dy++)
            {
                int ny = cy + dy;
                for (int dz = -1; dz <= 1; dz++)
                {
                    long key = GetCellKeyFromCoords(nx, ny, cz + dz);

                    if (_cells.TryGetValue(key, out var cell))
                    {
                        // Use index-based iteration for better performance
                        for (int i = 0; i < cell.Count; i++)
                        {
                            var existing = cell[i];
                            float dx2 = position.X - existing.X;
                            float dy2 = position.Y - existing.Y;
                            float dz2 = position.Z - existing.Z;
                            float distSq = dx2 * dx2 + dy2 * dy2 + dz2 * dz2;

                            if (distSq < _distanceThresholdSq)
                                return true;
                        }
                    }
                }
            }
        }

        return false;
    }

    /// <summary>
    /// Checks if a point is distant enough from all existing points AND adds it if so.
    /// Returns true if the point was added (was distant enough).
    /// This is an atomic operation - more efficient than separate check + add.
    /// </summary>
    public bool TryAdd(Vector3 position)
    {
        if (HasNearbyPoint(position))
            return false;

        Add(position);
        return true;
    }

    /// <summary>
    /// Clears all points from the hash.
    /// </summary>
    public void Clear()
    {
        _cells.Clear();
        Count = 0;
    }

    /// <summary>
    /// Gets approximate memory usage in bytes.
    /// </summary>
    public long GetMemoryUsage()
    {
        long usage = 0;
        foreach (var cell in _cells.Values)
        {
            usage += cell.Count * 12; // 3 floats per Vector3
            usage += 32; // List overhead
        }
        usage += _cells.Count * 24; // Dictionary entry overhead
        return usage;
    }
}

