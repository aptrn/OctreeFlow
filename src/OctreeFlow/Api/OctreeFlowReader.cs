using OctreeFlow.Core;
using OctreeFlow.Data;
using OctreeFlow.IO;
using Stride.Core.Mathematics;
using System.Diagnostics;

namespace OctreeFlow.Api;

/// <summary>
/// Main API for reading and traversing octree point cloud data.
/// Handles loading from .octree and .ply files, traversal, RAM caching, and buffer data output.
/// Designed for use with VVVV Gamma - outputs data ready for DynamicBufferAdvanced.
/// 
/// Workflow:
/// 1. Create reader with paths and settings
/// 2. Call Initialize() once
/// 3. Each frame: Call UpdateFrame() with your traversal delegate
/// 4. Use the BufferUpdateResult to upload new sectors to your DynamicBufferAdvanced buffers
/// 5. Use ActiveSectors for rendering dispatch
/// </summary>
public class OctreeFlowReader : IDisposable
{
    private readonly string _octreePath;
    private readonly string _plyPath;
    private readonly int _cacheSizeMB;
    private readonly int _maxBufferSizeMB;
    private readonly int? _pointsPerNodeOverride;

    private CacheManager? _cache;
    private SectorManager? _sectorManager;
    private OctreeNode? _root;
    private OctreeFileInfo? _fileInfo;
    private PlyIndex? _plyIndex;
    private readonly Dictionary<string, NodeInfo> _nodeInfoCache = new();
    private Dictionary<string, Vector4[]>? _featuresVector4;
    private Dictionary<string, float[]>? _featuresFloat32;
    
    private int _pointsPerNode; // Resolved from file or override
    private int _maxNodes; // Calculated from buffer size and points per node

    private int _traversalVersion;
    private bool _isInitialized;
    
    // Cached traversal result - return same object if content unchanged
    private TraversalResult? _cachedTraversalResult;
    private HashSet<string> _lastViewingNodeIds = new();
    
    // Cached frame update result - return same object if buffer state unchanged
    private FrameUpdateResult? _cachedFrameUpdateResult;
    private int _lastBufferVersion;

    #region Public Properties

    /// <summary>
    /// Root node of the octree.
    /// </summary>
    public OctreeNode? Root => _root;

    /// <summary>
    /// File information from the octree file.
    /// </summary>
    public OctreeFileInfo? FileInfo => _fileInfo;

    /// <summary>
    /// The RAM cache manager.
    /// </summary>
    public CacheManager? Cache => _cache;

    /// <summary>
    /// The sector manager for buffer data.
    /// </summary>
    public SectorManager? SectorManager => _sectorManager;

    /// <summary>
    /// Buffer configuration.
    /// </summary>
    public BufferConfiguration? BufferConfig => _sectorManager?.Configuration;

    /// <summary>
    /// Whether the reader has been initialized.
    /// </summary>
    public bool IsInitialized => _isInitialized;

    /// <summary>
    /// Bounding box of the entire point cloud.
    /// </summary>
    public BoundingBox Bounds => _fileInfo?.Bounds ?? new BoundingBox();

    /// <summary>
    /// Total number of points in the point cloud.
    /// </summary>
    public int TotalPoints => _fileInfo?.TotalPoints ?? 0;

    /// <summary>
    /// Total number of nodes in the octree.
    /// </summary>
    public int TotalNodes => _fileInfo?.NodeCount ?? 0;

    /// <summary>
    /// Points per node (from octree file or override).
    /// Each sector/node slot holds this many points.
    /// </summary>
    public int PointsPerNode => _pointsPerNode;

    /// <summary>
    /// Maximum buffer size per buffer in megabytes (as specified).
    /// </summary>
    public int MaxBufferSizeMB => _maxBufferSizeMB;

    /// <summary>
    /// Maximum number of nodes that can be active in the buffer simultaneously.
    /// Calculated as: MaxBufferSizeMB × 1024 × 1024 / (PointsPerNode × 16).
    /// </summary>
    public int MaxNodes => _maxNodes;

    /// <summary>
    /// Number of sectors in buffer (same as MaxNodes - 1 sector = 1 node).
    /// </summary>
    public int SectorCount => _maxNodes;

    /// <summary>
    /// Total buffer capacity in points.
    /// </summary>
    public int BufferCapacity => _maxNodes * _pointsPerNode;

    /// <summary>
    /// Total buffer size in bytes for Vector4 buffers (Position, Colors, Normals).
    /// Use this to create your DynamicBufferAdvanced&lt;Vector4&gt; with the correct size.
    /// Calculated as: MaxNodes × PointsPerNode × 16 bytes.
    /// </summary>
    public long BufferSizeBytesVector4 => (long)_maxNodes * _pointsPerNode * 16;

    /// <summary>
    /// Total buffer size in bytes for Float32 buffers (Intensity, scalars).
    /// Use this to create your DynamicBufferAdvanced&lt;float&gt; with the correct size.
    /// Calculated as: MaxNodes × PointsPerNode × 4 bytes.
    /// </summary>
    public long BufferSizeBytesFloat32 => (long)_maxNodes * _pointsPerNode * 4;

    /// <summary>
    /// Available properties from the PLY file.
    /// </summary>
    public IReadOnlyList<PlyProperty> PlyProperties => _plyIndex?.Properties ?? Array.Empty<PlyProperty>().ToList();

    /// <summary>
    /// Sequence of Vector4 feature names mapped to typed arrays for buffer initialization.
    /// Keys: "Position", "Colors", "Normals".
    /// Values: Vector4[] arrays sized to buffer capacity.
    /// Use this to create your DynamicBufferAdvanced instances in a ForEach.
    /// </summary>
    public IEnumerable<KeyValuePair<string, Vector4[]>> FeaturesVector4 => 
        _featuresVector4 ?? Enumerable.Empty<KeyValuePair<string, Vector4[]>>();

    /// <summary>
    /// Sequence of Float32 feature names mapped to typed arrays for buffer initialization.
    /// Keys: "Intensity" and any scalar dimension names.
    /// Values: float[] arrays sized to buffer capacity.
    /// Use this to create your DynamicBufferAdvanced instances in a ForEach.
    /// </summary>
    public IEnumerable<KeyValuePair<string, float[]>> FeaturesFloat32 => 
        _featuresFloat32 ?? Enumerable.Empty<KeyValuePair<string, float[]>>();

    /// <summary>
    /// Checks if a Vector4 feature exists (Position, Colors, Normals).
    /// </summary>
    /// <param name="name">Feature name (e.g., "Position", "Colors", "Normals").</param>
    /// <returns>True if the feature is available.</returns>
    public bool HasFeatureVector4(string name) => _featuresVector4?.ContainsKey(name) ?? false;

    /// <summary>
    /// Checks if a Float32 feature exists (Intensity or any scalar).
    /// </summary>
    /// <param name="name">Feature name (e.g., "Intensity" or scalar property name).</param>
    /// <returns>True if the feature is available.</returns>
    public bool HasFeatureFloat32(string name) => _featuresFloat32?.ContainsKey(name) ?? false;

    /// <summary>
    /// Checks if a scalar feature exists by name.
    /// This checks the Float32 features excluding Intensity.
    /// </summary>
    /// <param name="name">Scalar property name from the PLY file.</param>
    /// <returns>True if the scalar feature is available.</returns>
    public bool HasScalarFeature(string name)
    {
        if (_featuresFloat32 == null) return false;
        // Check if it exists and is not "Intensity" (which is a standard feature)
        return name != "Intensity" && _featuresFloat32.ContainsKey(name);
    }

    /// <summary>
    /// Gets the names of all available scalar features (excluding standard features like Intensity).
    /// </summary>
    public IEnumerable<string> ScalarFeatureNames => 
        _featuresFloat32?.Keys.Where(k => k != "Intensity") ?? Enumerable.Empty<string>();

    /// <summary>
    /// Gets the buffer array for a specific Vector4 feature.
    /// Use this to get the correctly-sized array for a specific feature.
    /// </summary>
    /// <param name="name">Feature name (e.g., "Position", "Colors", "Normals").</param>
    /// <returns>The Vector4 array, or null if not available.</returns>
    public Vector4[]? GetFeatureVector4(string name)
    {
        if (_featuresVector4 == null) return null;
        return _featuresVector4.TryGetValue(name, out var arr) ? arr : null;
    }

    /// <summary>
    /// Gets the buffer array for a specific Float32 feature.
    /// Use this to get the correctly-sized array for a specific feature (Intensity or scalar).
    /// </summary>
    /// <param name="name">Feature name (e.g., "Intensity" or scalar property name).</param>
    /// <returns>The float array, or null if not available.</returns>
    public float[]? GetFeatureFloat32(string name)
    {
        if (_featuresFloat32 == null) return null;
        return _featuresFloat32.TryGetValue(name, out var arr) ? arr : null;
    }

    /// <summary>
    /// Gets the buffer array for a specific scalar feature.
    /// Alias for GetFeatureFloat32 but more semantic for scalar usage.
    /// </summary>
    /// <param name="name">Scalar property name from the PLY file.</param>
    /// <returns>The float array, or null if not available.</returns>
    public float[]? GetScalarData(string name) => GetFeatureFloat32(name);

    #endregion

    /// <summary>
    /// Creates a new OctreeFlowReader.
    /// </summary>
    /// <param name="octreePath">Path to the .octree file.</param>
    /// <param name="plyPath">Path to the .ply file.</param>
    /// <param name="cacheSizeMB">RAM cache size in megabytes.</param>
    /// <param name="maxBufferSizeMB">Maximum size per buffer in megabytes (e.g., 512, 1024, 2048). Determines how many nodes can be active simultaneously.</param>
    /// <param name="pointsPerNodeOverride">Optional override for points per node. If null, reads from octree file (or uses default 1000 for legacy files).</param>
    public OctreeFlowReader(
        string octreePath,
        string plyPath,
        int cacheSizeMB = 512,
        int maxBufferSizeMB = 512,
        int? pointsPerNodeOverride = null)
    {
        _octreePath = octreePath;
        _plyPath = plyPath;
        _cacheSizeMB = cacheSizeMB;
        _maxBufferSizeMB = maxBufferSizeMB;
        _pointsPerNodeOverride = pointsPerNodeOverride;
    }

    /// <summary>
    /// Creates and initializes a new OctreeFlowReader.
    /// </summary>
    public static OctreeFlowReader Create(
        string octreePath,
        string plyPath,
        int cacheSizeMB = 512,
        int maxBufferSizeMB = 512,
        int? pointsPerNodeOverride = null)
    {
        var reader = new OctreeFlowReader(octreePath, plyPath, cacheSizeMB, maxBufferSizeMB, pointsPerNodeOverride);
        reader.Initialize();
        return reader;
    }

    /// <summary>
    /// Initializes the reader by loading the octree structure and building the PLY index.
    /// </summary>
    public void Initialize()
    {
        if (_isInitialized) return;

        // Load octree structure
        var serializer = new StreamingOctreeSerializer();
        var (root, info) = serializer.LoadOctreeFile(_octreePath);

        _root = root ?? throw new InvalidOperationException("Failed to load octree file");
        _fileInfo = info;

        // Resolve points per node: override > file > default
        if (_pointsPerNodeOverride.HasValue)
        {
            _pointsPerNode = _pointsPerNodeOverride.Value;
        }
        else if (info.PointsPerNode > 0)
        {
            _pointsPerNode = info.PointsPerNode;
        }
        else
        {
            // Legacy file without PointsPerNode - use default
            _pointsPerNode = 1000;
        }

        // Calculate maxNodes from buffer size and points per node
        // Using Vector4 size (16 bytes) as the constraint since it's the largest element type
        long bufferSizeBytes = (long)_maxBufferSizeMB * 1024 * 1024;
        long bytesPerNode = (long)_pointsPerNode * 16; // 16 bytes per Vector4
        _maxNodes = Math.Max(1, (int)(bufferSizeBytes / bytesPerNode));

        // Build PLY index (lightweight - just parse header)
        _plyIndex = new PlyIndex(_plyPath);
        _plyIndex.BuildIndexHeaderOnly();

        // Build node info cache
        BuildNodeInfoCache(_root);

        // Create RAM cache
        _cache = new CacheManager(_cacheSizeMB);

        // Create sector manager with calculated config
        var config = new BufferConfiguration
        {
            SectorCount = _maxNodes,
            MaxPointsPerSector = _pointsPerNode
        };
        _sectorManager = new SectorManager(_cache, config);

        // Build feature info dictionaries with correctly sized arrays
        BuildFeatureInfo(config.TotalCapacity);

        // Tell SectorManager which features are available (so it only creates matching data)
        _sectorManager.SetAvailableFeatures(
            _featuresVector4?.Keys ?? Enumerable.Empty<string>(),
            _featuresFloat32?.Keys ?? Enumerable.Empty<string>());

        _isInitialized = true;
    }

    /// <summary>
    /// Builds the feature info dictionaries from PLY properties.
    /// Populates FeaturesVector4 and FeaturesFloat32 with correctly-sized arrays for buffer initialization.
    /// </summary>
    /// <param name="bufferCapacity">Total buffer capacity in points (used to size the arrays).</param>
    private void BuildFeatureInfo(int bufferCapacity)
    {
        _featuresVector4 = new Dictionary<string, Vector4[]>();
        _featuresFloat32 = new Dictionary<string, float[]>();

        if (_plyIndex == null) return;

        // Standard Vector4 features (always present if PLY has x,y,z)
        bool hasPosition = false;
        bool hasColors = false;
        bool hasNormals = false;
        bool hasIntensity = false;

        foreach (var prop in _plyIndex.Properties)
        {
            var name = prop.Name.ToLower();

            // Check for position components
            if (name == "x" || name == "y" || name == "z")
            {
                hasPosition = true;
            }
            // Check for color components
            else if (name == "red" || name == "r" || name == "green" || name == "g" || 
                     name == "blue" || name == "b" || name == "alpha" || name == "a")
            {
                hasColors = true;
            }
            // Check for normal components
            else if (name == "nx" || name == "ny" || name == "nz")
            {
                hasNormals = true;
            }
            // Check for intensity
            else if (name == "intensity" || name == "scalar_intensity")
            {
                hasIntensity = true;
            }
            // Everything else is a scalar
            else
            {
                // Add as scalar feature with correctly-sized float array
                _featuresFloat32[prop.Name] = new float[bufferCapacity];
            }
        }

        // Add standard features based on what was found (with correctly-sized arrays)
        if (hasPosition)
        {
            _featuresVector4["Position"] = new Vector4[bufferCapacity];
        }
        if (hasColors)
        {
            _featuresVector4["Colors"] = new Vector4[bufferCapacity];
        }
        if (hasNormals)
        {
            _featuresVector4["Normals"] = new Vector4[bufferCapacity];
        }
        if (hasIntensity)
        {
            _featuresFloat32["Intensity"] = new float[bufferCapacity];
        }
    }

    /// <summary>
    /// Initializes the reader asynchronously.
    /// </summary>
    public async Task InitializeAsync(Action<string, int, int>? onProgress = null)
    {
        if (_isInitialized) return;

        await Task.Run(() =>
        {
            onProgress?.Invoke("Loading octree...", 0, 100);
            Initialize();
            onProgress?.Invoke("Ready", 100, 100);
        });
    }

    private void BuildNodeInfoCache(OctreeNode node)
    {
        var info = new NodeInfo(node);
        _nodeInfoCache[node.Id] = info;

        foreach (var child in node.Children)
        {
            BuildNodeInfoCache(child);
        }
    }

    /// <summary>
    /// Gets the NodeInfo for a node by ID.
    /// </summary>
    public NodeInfo? GetNodeInfo(string nodeId)
    {
        return _nodeInfoCache.TryGetValue(nodeId, out var info) ? info : null;
    }

    /// <summary>
    /// Gets statistics about the octree structure per level.
    /// Useful for debugging to see how points are distributed.
    /// </summary>
    /// <returns>Dictionary where key is level and value is (nodeCount, totalPoints).</returns>
    public Dictionary<int, (int NodeCount, int TotalPoints)> GetLevelStats()
    {
        EnsureInitialized();
        
        var stats = new Dictionary<int, (int NodeCount, int TotalPoints)>();
        
        foreach (var nodeInfo in _nodeInfoCache.Values)
        {
            if (!stats.ContainsKey(nodeInfo.Level))
            {
                stats[nodeInfo.Level] = (0, 0);
            }
            
            var current = stats[nodeInfo.Level];
            stats[nodeInfo.Level] = (current.NodeCount + 1, current.TotalPoints + nodeInfo.PointCount);
        }
        
        return stats;
    }

    /// <summary>
    /// Gets all NodeInfo objects at a specific level.
    /// </summary>
    public IEnumerable<NodeInfo> GetNodesAtLevel(int level)
    {
        EnsureInitialized();
        return _nodeInfoCache.Values.Where(n => n.Level == level);
    }

    /// <summary>
    /// Gets the maximum depth of the octree.
    /// </summary>
    public int MaxDepth => _nodeInfoCache.Values.Max(n => n.Level);

    #region Simple Traversal Methods (No Delegate Required)

    /// <summary>
    /// Traverses and selects all nodes at exactly the specified level.
    /// Easy to use without regions - just pass the level number.
    /// </summary>
    /// <param name="targetLevel">The level to select (0 = root, 1 = first children, etc.)</param>
    /// <returns>Result containing nodes at the target level.</returns>
    public TraversalResult TraverseToLevel(int targetLevel)
    {
        return Traverse(node =>
        {
            if (node.Level == targetLevel)
                return TraversalDecision.DisplayAndStop;
            else if (node.Level < targetLevel)
                return TraversalDecision.SkipButContinue;
            else
                return TraversalDecision.Reject;
        });
    }

    /// <summary>
    /// Traverses and selects all nodes from root up to and including the specified level.
    /// </summary>
    /// <param name="maxLevel">The maximum level to include (0 = root only, 1 = root + first children, etc.)</param>
    /// <returns>Result containing nodes up to maxLevel.</returns>
    public TraversalResult TraverseUpToLevel(int maxLevel)
    {
        return Traverse(node =>
        {
            if (node.Level <= maxLevel)
            {
                bool continueDeeper = node.Level < maxLevel && !node.IsLeaf;
                return new TraversalDecision(true, true, continueDeeper);
            }
            return TraversalDecision.Reject;
        });
    }

    /// <summary>
    /// Traverses and selects nodes until reaching approximately the target point count.
    /// Prioritizes coarser levels (lower detail) first.
    /// </summary>
    /// <param name="targetPointCount">Approximate maximum number of points to select.</param>
    /// <returns>Result containing nodes up to the point budget.</returns>
    public TraversalResult TraverseByPointBudget(int targetPointCount)
    {
        int currentPoints = 0;
        
        return Traverse(node =>
        {
            if (currentPoints >= targetPointCount)
                return TraversalDecision.Reject;

            currentPoints += node.PointCount;
            
            bool continueDeeper = currentPoints < targetPointCount && !node.IsLeaf;
            return new TraversalDecision(true, true, continueDeeper);
        });
    }

    /// <summary>
    /// Traverses and selects all leaf nodes (nodes with no children).
    /// This gives the highest detail available.
    /// </summary>
    /// <returns>Result containing all leaf nodes.</returns>
    public TraversalResult TraverseLeaves()
    {
        return Traverse(node =>
        {
            if (node.IsLeaf)
                return TraversalDecision.DisplayAndStop;
            else
                return TraversalDecision.SkipButContinue;
        });
    }

    /// <summary>
    /// Traverses and selects nodes that intersect with the given bounding box.
    /// Only includes nodes whose bounds overlap with the view bounds.
    /// </summary>
    /// <param name="viewBounds">The bounding box to test intersection with.</param>
    /// <param name="maxLevel">Maximum level to traverse to (-1 for unlimited).</param>
    /// <returns>Result containing nodes that intersect the view bounds.</returns>
    public TraversalResult TraverseByBounds(BoundingBox viewBounds, int maxLevel = -1)
    {
        return Traverse(node =>
        {
            // Check if node intersects view bounds
            bool intersects = node.BoundingBox.Intersects(ref viewBounds);
            
            if (!intersects)
                return TraversalDecision.Reject;

            // Check level limit
            if (maxLevel >= 0 && node.Level >= maxLevel)
                return TraversalDecision.DisplayAndStop;

            // Node intersects - include it and continue to children
            if (node.IsLeaf)
                return TraversalDecision.DisplayAndStop;
            else
                return TraversalDecision.DisplayAndContinue;
        });
    }

    /// <summary>
    /// Traverses and selects nodes based on distance from a camera position.
    /// Closer nodes get more detail (deeper levels), farther nodes get less detail.
    /// </summary>
    /// <param name="cameraPosition">The camera/viewer position.</param>
    /// <param name="detailBias">Higher values = more detail. Default 1.0. Range: 0.1 to 10.0</param>
    /// <param name="maxPoints">Maximum points to select (0 = unlimited).</param>
    /// <returns>Result containing nodes selected by distance-based LOD.</returns>
    public TraversalResult TraverseByDistance(Vector3 cameraPosition, float detailBias = 1.0f, int maxPoints = 0)
    {
        int currentPoints = 0;
        
        return Traverse(node =>
        {
            // Check point budget
            if (maxPoints > 0 && currentPoints >= maxPoints)
                return TraversalDecision.Reject;

            // Calculate distance from camera to node center
            float distance = Vector3.Distance(cameraPosition, node.Center);
            
            // Calculate node size (use largest dimension)
            float nodeSize = Math.Max(Math.Max(node.Size.X, node.Size.Y), node.Size.Z);
            
            // Screen-space size heuristic: larger nodes or closer nodes should subdivide
            // screenSize approximates how big the node appears on screen
            float screenSize = (nodeSize / Math.Max(distance, 0.001f)) * detailBias;
            
            // Threshold for subdivision (tune this value)
            float subdivideThreshold = 0.1f;
            
            currentPoints += node.PointCount;

            if (node.IsLeaf || screenSize < subdivideThreshold)
                return TraversalDecision.DisplayAndStop;
            else
                return TraversalDecision.DisplayAndContinue;
        });
    }

    #endregion

    /// <summary>
    /// Traverses the octree using the provided function.
    /// This appears as a regular node input in vvvv gamma (not a region).
    /// Pass a Func&lt;NodeInfo, TraversalDecision&gt; that decides how to handle each node.
    /// 
    /// IMPORTANT: Returns the SAME TraversalResult object if the viewing nodes haven't changed.
    /// This prevents unnecessary downstream updates in vvvv gamma.
    /// </summary>
    /// <param name="traversalFunction">Function that takes NodeInfo and returns TraversalDecision.</param>
    /// <returns>Result containing caching and viewing node lists. Same object if unchanged.</returns>
    public TraversalResult Traverse(Func<NodeInfo, TraversalDecision> traversalFunction)
    {
        EnsureInitialized();

        var sw = Stopwatch.StartNew();
        var result = new TraversalResult();

        if (_root == null)
        {
            result.IsComplete = true;
            result.TraversalTimeMs = sw.ElapsedMilliseconds;
            result.Version = _traversalVersion;
            return _cachedTraversalResult ?? result;
        }

        TraverseNode(_root, traversalFunction, result);

        sw.Stop();
        result.TraversalTimeMs = sw.ElapsedMilliseconds;
        result.IsComplete = true;

        // Check if viewing nodes changed
        var currentNodeIds = new HashSet<string>(result.ViewingNodes.Select(n => n.Id));
        
        if (_cachedTraversalResult != null && currentNodeIds.SetEquals(_lastViewingNodeIds))
        {
            // Content unchanged - return the SAME object (no "Changed" trigger in vvvv)
            return _cachedTraversalResult;
        }

        // Content changed - increment version and cache new result
        result.Version = Interlocked.Increment(ref _traversalVersion);
        _lastViewingNodeIds = currentNodeIds;
        _cachedTraversalResult = result;

        return result;
    }

    private void TraverseNode(OctreeNode node, Func<NodeInfo, TraversalDecision> func, TraversalResult result)
    {
        result.NodesVisited++;

        // Get or create NodeInfo
        if (!_nodeInfoCache.TryGetValue(node.Id, out var nodeInfo))
        {
            nodeInfo = new NodeInfo(node);
            _nodeInfoCache[node.Id] = nodeInfo;
        }

        // Update status from cache and sector manager
        nodeInfo.UpdateStatus(
            _cache?.Contains(node.Id) ?? false,
            _sectorManager?.Contains(node.Id) ?? false,
            _sectorManager?.GetSectorFor(node.Id) ?? -1
        );

        // Call function
        var decision = func(nodeInfo);

        if (decision.IsAccepted)
        {
            result.NodesAccepted++;
            result.CachingNodes.Add(nodeInfo);

            if (decision.IsForDisplay)
            {
                result.ViewingNodes.Add(nodeInfo);
            }
        }

        // Continue to children if requested
        if (decision.ContinueToChildren)
        {
            foreach (var child in node.Children)
            {
                TraverseNode(child, func, result);
            }
        }
    }

    /// <summary>
    /// Loads specified nodes into RAM cache.
    /// </summary>
    /// <param name="nodes">Nodes to load into cache.</param>
    /// <returns>Result of the cache loading operation.</returns>
    public CacheLoadResult LoadToCache(IEnumerable<NodeInfo> nodes)
    {
        EnsureInitialized();

        var sw = Stopwatch.StartNew();
        var result = new CacheLoadResult();
        var nodesToLoad = nodes.Where(n => !_cache!.Contains(n.Id)).ToList();

        try
        {
            foreach (var nodeInfo in nodesToLoad)
            {
                var indices = nodeInfo.Node.PointIndices?.ToArray();
                if (indices != null && indices.Length > 0)
                {
                    var pointData = ReadPointData(indices);
                    _cache!.Add(nodeInfo.Id, indices, pointData);
                    result.LoadedNodes[nodeInfo.Id] = indices;
                }
            }

            result.Version = _cache!.Version;
            result.IsComplete = true;
        }
        catch (Exception ex)
        {
            result.Error = ex.Message;
            result.IsComplete = false;
        }

        sw.Stop();
        result.LoadTimeMs = sw.ElapsedMilliseconds;
        return result;
    }

    /// <summary>
    /// Loads specified nodes into RAM cache asynchronously.
    /// </summary>
    public async Task<CacheLoadResult> LoadToCacheAsync(
        IEnumerable<NodeInfo> nodes,
        CancellationToken cancellationToken = default)
    {
        EnsureInitialized();

        var sw = Stopwatch.StartNew();
        var result = new CacheLoadResult();
        var nodesToLoad = nodes.Where(n => !_cache!.Contains(n.Id)).ToList();

        try
        {
            await Task.Run(() =>
            {
                foreach (var nodeInfo in nodesToLoad)
                {
                    if (cancellationToken.IsCancellationRequested) break;

                    var indices = nodeInfo.Node.PointIndices?.ToArray();
                    if (indices != null && indices.Length > 0)
                    {
                        var pointData = ReadPointData(indices);
                        _cache!.Add(nodeInfo.Id, indices, pointData);
                        result.LoadedNodes[nodeInfo.Id] = indices;
                    }
                }
            }, cancellationToken);

            result.Version = _cache!.Version;
            result.IsComplete = !cancellationToken.IsCancellationRequested;
        }
        catch (OperationCanceledException)
        {
            result.IsComplete = false;
        }
        catch (Exception ex)
        {
            result.Error = ex.Message;
            result.IsComplete = false;
        }

        sw.Stop();
        result.LoadTimeMs = sw.ElapsedMilliseconds;
        return result;
    }

    /// <summary>
    /// Updates buffer with the given viewing nodes.
    /// Loads nodes to cache if needed, then updates sector manager.
    /// Use this after calling Traverse() separately.
    /// </summary>
    /// <param name="viewingNodes">Nodes to display (from TraversalResult.ViewingNodes).</param>
    /// <returns>Buffer update result with new sectors and active sectors.</returns>
    public BufferUpdateResult UpdateBuffer(IEnumerable<NodeInfo> viewingNodes)
    {
        EnsureInitialized();

        var nodeList = viewingNodes.ToList();

        // Load viewing nodes to cache (sync)
        var notInCache = nodeList.Where(n => !_cache!.Contains(n.Id)).ToList();

        if (notInCache.Count > 0)
        {
            LoadToCache(notInCache);
        }

        // Update sector manager - get buffer data to upload
        return _sectorManager!.Update(nodeList);
    }

    /// <summary>
    /// Updates buffer with the given viewing nodes asynchronously.
    /// Loads nodes to cache if needed, then updates sector manager.
    /// Use this after calling Traverse() separately.
    /// </summary>
    /// <param name="viewingNodes">Nodes to display (from TraversalResult.ViewingNodes).</param>
    /// <param name="cancellationToken">Cancellation token.</param>
    /// <returns>Buffer update result with new sectors and active sectors.</returns>
    public async Task<BufferUpdateResult> UpdateBufferAsync(
        IEnumerable<NodeInfo> viewingNodes,
        CancellationToken cancellationToken = default)
    {
        EnsureInitialized();

        var nodeList = viewingNodes.ToList();

        // Load viewing nodes to cache (async)
        var notInCache = nodeList.Where(n => !_cache!.Contains(n.Id)).ToList();

        if (notInCache.Count > 0)
        {
            await LoadToCacheAsync(notInCache, cancellationToken);
        }

        // Update sector manager - get buffer data to upload
        return _sectorManager!.Update(nodeList);
    }

    /// <summary>
    /// Performs a complete frame update: Traverse → Cache → Buffer data output.
    /// Convenience method that combines Traverse() and UpdateBuffer().
    /// 
    /// IMPORTANT: Returns the SAME FrameUpdateResult object if nothing changed.
    /// This prevents unnecessary downstream updates in vvvv gamma.
    /// </summary>
    /// <param name="traversalResult">Result from a previous Traverse() call.</param>
    /// <returns>Complete frame result with buffer data to upload. Same object if unchanged.</returns>
    public FrameUpdateResult UpdateFrame(TraversalResult traversalResult)
    {
        EnsureInitialized();

        var sw = Stopwatch.StartNew();
        var result = new FrameUpdateResult
        {
            Traversal = traversalResult
        };

        // Load viewing nodes to cache (sync)
        var notInCache = traversalResult.ViewingNodes
            .Where(n => !_cache!.Contains(n.Id))
            .ToList();

        if (notInCache.Count > 0)
        {
            result.CacheResult = LoadToCache(notInCache);
        }
        else
        {
            result.CacheResult = new CacheLoadResult { IsComplete = true, Version = _cache!.Version };
        }

        // Update sector manager - get buffer data to upload
        result.BufferUpdate = _sectorManager!.Update(traversalResult.ViewingNodes);

        sw.Stop();
        result.TotalTimeMs = sw.ElapsedMilliseconds;

        // Check if buffer state changed
        int currentBufferVersion = result.BufferUpdate.Version;
        
        if (_cachedFrameUpdateResult != null && 
            currentBufferVersion == _lastBufferVersion && 
            !result.BufferUpdate.HasNewData)
        {
            // Buffer state unchanged - return the SAME object (no "Changed" trigger in vvvv)
            return _cachedFrameUpdateResult;
        }

        // Buffer state changed - cache new result
        _lastBufferVersion = currentBufferVersion;
        _cachedFrameUpdateResult = result;

        return result;
    }

    /// <summary>
    /// Async version of UpdateFrame.
    /// </summary>
    /// <param name="traversalResult">Result from a previous Traverse() call.</param>
    /// <param name="cancellationToken">Cancellation token.</param>
    /// <returns>Complete frame result with buffer data to upload.</returns>
    public async Task<FrameUpdateResult> UpdateFrameAsync(
        TraversalResult traversalResult,
        CancellationToken cancellationToken = default)
    {
        EnsureInitialized();

        var sw = Stopwatch.StartNew();
        var result = new FrameUpdateResult
        {
            Traversal = traversalResult
        };

        // Load viewing nodes to cache (async)
        var notInCache = traversalResult.ViewingNodes
            .Where(n => !_cache!.Contains(n.Id))
            .ToList();

        if (notInCache.Count > 0)
        {
            result.CacheResult = await LoadToCacheAsync(notInCache, cancellationToken);
        }
        else
        {
            result.CacheResult = new CacheLoadResult { IsComplete = true, Version = _cache!.Version };
        }

        // Update sector manager - get buffer data to upload
        result.BufferUpdate = _sectorManager!.Update(traversalResult.ViewingNodes);

        sw.Stop();
        result.TotalTimeMs = sw.ElapsedMilliseconds;

        return result;
    }

    /// <summary>
    /// Reads point data for the given indices from the PLY file.
    /// </summary>
    private PointData[] ReadPointData(int[] indices)
    {
        if (_plyIndex == null) return Array.Empty<PointData>();

        var result = new PointData[indices.Length];

        // For binary PLY, we can do random access
        if (_plyIndex.Format != PlyFormat.Ascii)
        {
            for (int i = 0; i < indices.Length; i++)
            {
                var values = _plyIndex.ReadVertex(indices[i]);
                result[i] = ConvertToPointData(values);
            }
        }
        else
        {
            // For ASCII, we need to stream (less efficient for random access)
            var sortedIndices = indices.Select((idx, i) => (idx, i)).OrderBy(x => x.idx).ToArray();
            int currentIndex = 0;

            _plyIndex.StreamVertices((vertexIndex, pos, values) =>
            {
                while (currentIndex < sortedIndices.Length && sortedIndices[currentIndex].idx == vertexIndex)
                {
                    result[sortedIndices[currentIndex].i] = ConvertToPointData(values);
                    currentIndex++;
                }
            });
        }

        return result;
    }

    private PointData ConvertToPointData(float[] values)
    {
        if (_plyIndex == null) return new PointData();

        var point = new PointData();
        var props = _plyIndex.Properties;

        for (int i = 0; i < props.Count && i < values.Length; i++)
        {
            var name = props[i].Name.ToLower();
            var val = values[i];

            switch (name)
            {
                case "x": point.Position.X = val; break;
                case "y": point.Position.Y = val; break;
                case "z": point.Position.Z = val; break;
                case "red" or "r":
                    point.Color = new Color4(val > 1 ? val / 255f : val, point.Color.G, point.Color.B, point.Color.A);
                    break;
                case "green" or "g":
                    point.Color = new Color4(point.Color.R, val > 1 ? val / 255f : val, point.Color.B, point.Color.A);
                    break;
                case "blue" or "b":
                    point.Color = new Color4(point.Color.R, point.Color.G, val > 1 ? val / 255f : val, point.Color.A);
                    break;
                case "alpha" or "a":
                    point.Color = new Color4(point.Color.R, point.Color.G, point.Color.B, val > 1 ? val / 255f : val);
                    break;
                case "nx": point.Normal.X = val; break;
                case "ny": point.Normal.Y = val; break;
                case "nz": point.Normal.Z = val; break;
                case "intensity" or "scalar_intensity":
                    point.Intensity = val > 1 ? val / 65535f : val;
                    break;
                default:
                    // Store as scalar
                    point.SetScalar(name, val);
                    break;
            }
        }

        return point;
    }

    private void EnsureInitialized()
    {
        if (!_isInitialized)
            throw new InvalidOperationException("Reader not initialized. Call Initialize() first.");
    }

    public void Dispose()
    {
        _cache?.Dispose();
        _sectorManager?.Dispose();
        _plyIndex?.Dispose();
    }
}

/// <summary>
/// Complete result of a frame update.
/// Contains traversal info, cache status, and buffer data to upload.
/// </summary>
public class FrameUpdateResult
{
    /// <summary>
    /// Traversal result.
    /// </summary>
    public TraversalResult Traversal { get; set; } = new();

    /// <summary>
    /// Cache loading result.
    /// </summary>
    public CacheLoadResult CacheResult { get; set; } = new();

    /// <summary>
    /// Buffer update result with data to upload.
    /// </summary>
    public BufferUpdateResult BufferUpdate { get; set; } = new();

    /// <summary>
    /// Total time for the entire update.
    /// </summary>
    public long TotalTimeMs { get; set; }

    /// <summary>
    /// Quick access to new sector data to upload.
    /// Mutable list of sectors, each containing a Features dictionary.
    /// Upload these to your DynamicBufferAdvanced buffers.
    /// </summary>
    public List<SectorData> NewSectors => BufferUpdate.NewSectors;

    /// <summary>
    /// Quick access to active sectors for rendering.
    /// Use this for your rendering dispatch.
    /// </summary>
    public SectorInfo[] ActiveSectors => BufferUpdate.ActiveSectors;

    /// <summary>
    /// Total points ready for rendering.
    /// </summary>
    public int TotalPointsInBuffer => BufferUpdate.TotalPointsInBuffer;

    /// <summary>
    /// Whether there's new data to upload this frame.
    /// </summary>
    public bool HasNewData => BufferUpdate.HasNewData;
}
