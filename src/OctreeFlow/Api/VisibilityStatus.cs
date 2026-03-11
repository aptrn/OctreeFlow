namespace OctreeFlow.Api;

/// <summary>
/// Describes a node's visibility relative to the camera frustum and potential occluders.
/// </summary>
public enum VisibilityStatus
{
    /// <summary>
    /// The node's bounding box is fully within the view frustum and not occluded.
    /// </summary>
    InView,

    /// <summary>
    /// The node's bounding box partially intersects the view frustum, or is partially
    /// occluded by another node's screen-space projection.
    /// </summary>
    HalfCovered,

    /// <summary>
    /// The node's bounding box is entirely outside the view frustum, or completely
    /// occluded by another node's screen-space projection.
    /// </summary>
    Hidden
}
