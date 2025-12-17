using OctreeFlow.Data;
using OctreeFlow.IO;
using System.Collections.Concurrent;
using System.Diagnostics;

namespace OctreeFlow.Api;

/// <summary>
/// Manages RAM cache for point data with LRU eviction policy.
/// Stores point data loaded from PLY file, keyed by node ID.
/// </summary>
public class CacheManager : IDisposable
{
    private readonly ConcurrentDictionary<string, CacheEntry> _cache = new();
    private readonly LinkedList<string> _lruList = new();
    private readonly object _lruLock = new();
    private readonly long _maxSizeBytes;
    private long _currentSizeBytes;
    private int _version;

    /// <summary>
    /// Current cache version (increments on each change).
    /// </summary>
    public int Version => _version;

    /// <summary>
    /// Maximum cache size in bytes.
    /// </summary>
    public long MaxSizeBytes => _maxSizeBytes;

    /// <summary>
    /// Current used size in bytes.
    /// </summary>
    public long CurrentSizeBytes => _currentSizeBytes;

    /// <summary>
    /// Number of entries (nodes) currently in cache.
    /// </summary>
    public int EntryCount => _cache.Count;

    /// <summary>
    /// Total number of points across all cached nodes.
    /// </summary>
    public int TotalPointsCached => _cache.Values.Sum(e => e.PointIndices.Length);

    /// <summary>
    /// Creates a new cache manager with specified size in MB.
    /// </summary>
    /// <param name="maxSizeMB">Maximum cache size in megabytes.</param>
    public CacheManager(int maxSizeMB)
    {
        _maxSizeBytes = (long)maxSizeMB * 1024 * 1024;
    }

    /// <summary>
    /// Checks if a node is in the cache.
    /// </summary>
    public bool Contains(string nodeId) => _cache.ContainsKey(nodeId);

    /// <summary>
    /// Gets cached point indices for a node, or null if not cached.
    /// </summary>
    public int[]? GetPointIndices(string nodeId)
    {
        if (_cache.TryGetValue(nodeId, out var entry))
        {
            TouchEntry(nodeId);
            return entry.PointIndices;
        }
        return null;
    }

    /// <summary>
    /// Gets cached point data for a node, or null if not cached.
    /// </summary>
    public PointData[]? GetPointData(string nodeId)
    {
        if (_cache.TryGetValue(nodeId, out var entry))
        {
            TouchEntry(nodeId);
            return entry.PointData;
        }
        return null;
    }

    /// <summary>
    /// Adds point data to the cache.
    /// </summary>
    public void Add(string nodeId, int[] pointIndices, PointData[]? pointData = null)
    {
        // Calculate entry size (indices + optional point data)
        long entrySize = pointIndices.Length * sizeof(int);
        if (pointData != null)
        {
            // Approximate size: position (12) + color (16) + normal (12) + intensity (4) = 44 bytes base
            entrySize += pointData.Length * 48; // Add overhead for struct
        }

        // Evict if necessary
        while (_currentSizeBytes + entrySize > _maxSizeBytes && _cache.Count > 0)
        {
            EvictOldest();
        }

        var entry = new CacheEntry
        {
            NodeId = nodeId,
            PointIndices = pointIndices,
            PointData = pointData,
            SizeBytes = entrySize
        };

        if (_cache.TryAdd(nodeId, entry))
        {
            Interlocked.Add(ref _currentSizeBytes, entrySize);
            AddToLru(nodeId);
            Interlocked.Increment(ref _version);
        }
    }

    /// <summary>
    /// Removes a node from the cache.
    /// </summary>
    public bool Remove(string nodeId)
    {
        if (_cache.TryRemove(nodeId, out var entry))
        {
            Interlocked.Add(ref _currentSizeBytes, -entry.SizeBytes);
            RemoveFromLru(nodeId);
            Interlocked.Increment(ref _version);
            return true;
        }
        return false;
    }

    /// <summary>
    /// Clears the entire cache.
    /// </summary>
    public void Clear()
    {
        _cache.Clear();
        lock (_lruLock)
        {
            _lruList.Clear();
        }
        _currentSizeBytes = 0;
        Interlocked.Increment(ref _version);
    }

    /// <summary>
    /// Gets all cached node IDs.
    /// </summary>
    public IEnumerable<string> GetCachedNodeIds() => _cache.Keys;

    private void TouchEntry(string nodeId)
    {
        lock (_lruLock)
        {
            var node = _lruList.Find(nodeId);
            if (node != null)
            {
                _lruList.Remove(node);
                _lruList.AddLast(nodeId);
            }
        }
    }

    private void AddToLru(string nodeId)
    {
        lock (_lruLock)
        {
            _lruList.AddLast(nodeId);
        }
    }

    private void RemoveFromLru(string nodeId)
    {
        lock (_lruLock)
        {
            _lruList.Remove(nodeId);
        }
    }

    private void EvictOldest()
    {
        string? oldestId;
        lock (_lruLock)
        {
            if (_lruList.Count == 0) return;
            oldestId = _lruList.First?.Value;
            if (oldestId != null)
                _lruList.RemoveFirst();
        }

        if (oldestId != null && _cache.TryRemove(oldestId, out var entry))
        {
            Interlocked.Add(ref _currentSizeBytes, -entry.SizeBytes);
        }
    }

    public void Dispose()
    {
        Clear();
    }

    private class CacheEntry
    {
        public required string NodeId { get; init; }
        public required int[] PointIndices { get; init; }
        public PointData[]? PointData { get; init; }
        public required long SizeBytes { get; init; }
    }
}

/// <summary>
/// Result of a cache loading operation.
/// </summary>
public class CacheLoadResult
{
    /// <summary>
    /// Incremental version number of the cache.
    /// </summary>
    public int Version { get; set; }

    /// <summary>
    /// Total loading time in milliseconds.
    /// </summary>
    public long LoadTimeMs { get; set; }

    /// <summary>
    /// Whether the loading process is complete.
    /// </summary>
    public bool IsComplete { get; set; }

    /// <summary>
    /// Dictionary mapping node IDs to their point indices.
    /// </summary>
    public Dictionary<string, int[]> LoadedNodes { get; set; } = new();

    /// <summary>
    /// Number of nodes successfully loaded.
    /// </summary>
    public int NodesLoaded => LoadedNodes.Count;

    /// <summary>
    /// Total points loaded.
    /// </summary>
    public int TotalPointsLoaded => LoadedNodes.Values.Sum(indices => indices.Length);

    /// <summary>
    /// Error message if loading failed.
    /// </summary>
    public string? Error { get; set; }
}

