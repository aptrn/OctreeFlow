namespace OctreeFlow.Api;

/// <summary>
/// Decision result from a traversal delegate.
/// </summary>
public struct TraversalDecision
{
    /// <summary>
    /// Whether the node is accepted for processing.
    /// </summary>
    public bool IsAccepted { get; set; }

    /// <summary>
    /// Whether the node should be marked for GPU display (viewing).
    /// </summary>
    public bool IsForDisplay { get; set; }

    /// <summary>
    /// Whether to continue traversing into children nodes.
    /// </summary>
    public bool ContinueToChildren { get; set; }

    public TraversalDecision(bool accepted, bool forDisplay, bool continueToChildren)
    {
        IsAccepted = accepted;
        IsForDisplay = forDisplay;
        ContinueToChildren = continueToChildren;
    }

    /// <summary>
    /// Reject this node and stop traversal here.
    /// </summary>
    public static TraversalDecision Reject => new(false, false, false);

    /// <summary>
    /// Accept for caching only, continue to children.
    /// </summary>
    public static TraversalDecision CacheOnly => new(true, false, true);

    /// <summary>
    /// Accept for display, continue to children.
    /// </summary>
    public static TraversalDecision DisplayAndContinue => new(true, true, true);

    /// <summary>
    /// Accept for display, stop at this level (leaf-like behavior).
    /// </summary>
    public static TraversalDecision DisplayAndStop => new(true, true, false);

    /// <summary>
    /// Skip this node but continue to children.
    /// </summary>
    public static TraversalDecision SkipButContinue => new(false, false, true);
}

/// <summary>
/// Delegate for traversing octree nodes.
/// </summary>
/// <param name="nodeInfo">Information about the current node.</param>
/// <returns>A decision about how to handle this node.</returns>
public delegate TraversalDecision TraversalDelegate(NodeInfo nodeInfo);

/// <summary>
/// Result of a traversal operation.
/// </summary>
public class TraversalResult
{
    /// <summary>
    /// Incremental version number of this traversal output.
    /// </summary>
    public int Version { get; set; }

    /// <summary>
    /// Total traversal time in milliseconds.
    /// </summary>
    public long TraversalTimeMs { get; set; }

    /// <summary>
    /// Whether the traversal is complete.
    /// </summary>
    public bool IsComplete { get; set; }

    /// <summary>
    /// Nodes marked for caching (accepted nodes).
    /// </summary>
    public List<NodeInfo> CachingNodes { get; set; } = new();

    /// <summary>
    /// Nodes marked for GPU display (viewing nodes).
    /// </summary>
    public List<NodeInfo> ViewingNodes { get; set; } = new();

    /// <summary>
    /// Total number of nodes visited during traversal.
    /// </summary>
    public int NodesVisited { get; set; }

    /// <summary>
    /// Total number of nodes accepted.
    /// </summary>
    public int NodesAccepted { get; set; }

    /// <summary>
    /// Total point count across all viewing nodes.
    /// </summary>
    public int TotalViewingPoints => ViewingNodes.Sum(n => n.PointCount);

    /// <summary>
    /// Total point count across all caching nodes.
    /// </summary>
    public int TotalCachingPoints => CachingNodes.Sum(n => n.PointCount);
}

