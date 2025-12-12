using Stride.Core.Mathematics;

namespace OctreeFlow.Data;

/// <summary>
/// Represents a collection of points loaded from a PLY file.
/// Acts as the "ply buffer" during octree construction.
/// </summary>
public class PointCloud
{
    private readonly List<PointData> _points;
    private readonly HashSet<int> _availableIndices;
    private readonly Random _random;

    /// <summary>
    /// All points in the cloud.
    /// </summary>
    public IReadOnlyList<PointData> Points => _points;

    /// <summary>
    /// Number of points still available (not yet assigned to nodes).
    /// </summary>
    public int AvailableCount => _availableIndices.Count;

    /// <summary>
    /// Total number of points in the cloud.
    /// </summary>
    public int TotalCount => _points.Count;

    /// <summary>
    /// Bounding box containing all points.
    /// </summary>
    public BoundingBox Bounds { get; private set; }

    /// <summary>
    /// Property names found in the PLY file header.
    /// </summary>
    public List<string> PropertyNames { get; } = new();

    public PointCloud()
    {
        _points = new List<PointData>();
        _availableIndices = new HashSet<int>();
        _random = new Random();
        Bounds = new BoundingBox();
    }

    public PointCloud(int seed)
    {
        _points = new List<PointData>();
        _availableIndices = new HashSet<int>();
        _random = new Random(seed);
        Bounds = new BoundingBox();
    }

    /// <summary>
    /// Adds a point to the cloud.
    /// </summary>
    public void AddPoint(PointData point)
    {
        int index = _points.Count;
        _points.Add(point);
        _availableIndices.Add(index);
        UpdateBounds(point.Position);
    }

    /// <summary>
    /// Gets a point by its index.
    /// </summary>
    public PointData GetPoint(int index) => _points[index];

    /// <summary>
    /// Gets a random available point index. Returns -1 if no points available.
    /// </summary>
    public int GetRandomAvailableIndex()
    {
        if (_availableIndices.Count == 0)
            return -1;

        int skip = _random.Next(_availableIndices.Count);
        return _availableIndices.Skip(skip).First();
    }

    /// <summary>
    /// Marks a point as used (removes from available pool).
    /// </summary>
    public void MarkUsed(int index)
    {
        _availableIndices.Remove(index);
    }

    /// <summary>
    /// Checks if a point index is still available.
    /// </summary>
    public bool IsAvailable(int index) => _availableIndices.Contains(index);

    /// <summary>
    /// Gets all available indices as a list.
    /// </summary>
    public List<int> GetAvailableIndices() => _availableIndices.ToList();

    /// <summary>
    /// Gets available indices that fall within the specified bounding box.
    /// </summary>
    public List<int> GetAvailableIndicesInBounds(BoundingBox bounds)
    {
        var result = new List<int>();
        foreach (var index in _availableIndices)
        {
            var pos = _points[index].Position;
            if (bounds.Contains(ref pos) != ContainmentType.Disjoint)
            {
                result.Add(index);
            }
        }
        return result;
    }

    /// <summary>
    /// Removes a set of indices from the available pool.
    /// </summary>
    public void MarkUsedBatch(IEnumerable<int> indices)
    {
        foreach (var index in indices)
        {
            _availableIndices.Remove(index);
        }
    }

    /// <summary>
    /// Resets all points to available state.
    /// </summary>
    public void ResetAvailability()
    {
        _availableIndices.Clear();
        for (int i = 0; i < _points.Count; i++)
        {
            _availableIndices.Add(i);
        }
    }

    /// <summary>
    /// Computes and sets the bounding box from all points.
    /// </summary>
    public void ComputeBounds()
    {
        if (_points.Count == 0)
        {
            Bounds = new BoundingBox();
            return;
        }

        var min = _points[0].Position;
        var max = _points[0].Position;

        foreach (var point in _points)
        {
            min = Vector3.Min(min, point.Position);
            max = Vector3.Max(max, point.Position);
        }

        Bounds = new BoundingBox(min, max);
    }

    private void UpdateBounds(Vector3 position)
    {
        if (_points.Count == 1)
        {
            Bounds = new BoundingBox(position, position);
        }
        else
        {
            Bounds = new BoundingBox(
                Vector3.Min(Bounds.Minimum, position),
                Vector3.Max(Bounds.Maximum, position)
            );
        }
    }
}

