using Stride.Core.Mathematics;
using OctreeFlow.Core;

namespace OctreeFlow.IO;

/// <summary>
/// Computes per-node metadata (average color, point density) for an octree.
/// Requires a single streaming pass over the PLY file to gather color data.
/// Density is computed purely from point counts and bounding box volumes.
/// </summary>
public class NodeMetadataComputer
{
    /// <summary>
    /// Progress callback: (phase, current, total).
    /// Phase 0 = building point-to-node map, Phase 1 = streaming PLY for colors.
    /// </summary>
    public Action<int, int, int>? OnProgress { get; set; }

    /// <summary>
    /// Computes AverageColor, PointDensityRaw, and PointDensity for every node in the tree.
    /// Sets <see cref="OctreeNode.HasMetadata"/> to true on each node when done.
    /// </summary>
    /// <param name="root">Root node of the octree (with point indices populated).</param>
    /// <param name="plyIndex">PLY index (must have been built with BuildIndex or BuildIndexHeaderOnly).</param>
    public void Compute(OctreeNode root, PlyIndex plyIndex)
    {
        // --- Phase 0: collect all nodes in a flat list ---
        var nodes = new List<OctreeNode>();
        CollectAllNodes(root, nodes);
        int nodeCount = nodes.Count;

        // Build point-index → node-list-index reverse map.
        // This lets us accumulate color sums while streaming the PLY sequentially.
        int totalPoints = plyIndex.VertexCount;
        var pointToNode = new int[totalPoints];
        Array.Fill(pointToNode, -1);

        for (int n = 0; n < nodeCount; n++)
        {
            foreach (int pi in nodes[n].PointIndices)
            {
                if ((uint)pi < (uint)totalPoints)
                    pointToNode[pi] = n;
            }
            if (n % 10000 == 0)
                OnProgress?.Invoke(0, n, nodeCount);
        }
        OnProgress?.Invoke(0, nodeCount, nodeCount);

        // --- Compute density (no PLY read required) ---
        float maxDensityRaw = 0f;
        foreach (var node in nodes)
        {
            var s = node.BoundingBox.Maximum - node.BoundingBox.Minimum;
            float volume = s.X * s.Y * s.Z;
            node.PointDensityRaw = (volume > 0f && node.PointCount > 0)
                ? node.PointCount / volume
                : 0f;
            if (node.PointDensityRaw > maxDensityRaw)
                maxDensityRaw = node.PointDensityRaw;
        }

        if (maxDensityRaw > 0f)
        {
            foreach (var node in nodes)
                node.PointDensity = node.PointDensityRaw / maxDensityRaw;
        }

        // --- Phase 1: stream PLY for average color ---
        // Locate color property indices.
        int rIdx = -1, gIdx = -1, bIdx = -1, aIdx = -1;
        for (int i = 0; i < plyIndex.Properties.Count; i++)
        {
            switch (plyIndex.Properties[i].Name.ToLowerInvariant())
            {
                case "red" or "r":   rIdx = i; break;
                case "green" or "g": gIdx = i; break;
                case "blue" or "b":  bIdx = i; break;
                case "alpha" or "a": aIdx = i; break;
            }
        }

        bool hasColor = rIdx >= 0 && gIdx >= 0 && bIdx >= 0;

        if (!hasColor)
        {
            // No color data — mark all nodes with metadata (density is still valid).
            foreach (var node in nodes)
            {
                node.AverageColor = new Color4(1f, 1f, 1f, 1f);
                node.HasMetadata = true;
            }
            return;
        }

        // Determine normalization scale: UInt8 (uchar) → 0-255, Float32 → already 0-1.
        float colorScale = plyIndex.Properties[rIdx].Type == PlyDataType.UInt8
            ? 1f / 255f
            : 1f;

        // Accumulate color sums per node.
        var sumR = new double[nodeCount];
        var sumG = new double[nodeCount];
        var sumB = new double[nodeCount];
        var sumA = new double[nodeCount];
        var counts = new int[nodeCount];

        plyIndex.StreamVertices(
            (idx, _, values) =>
            {
                int ni = (uint)idx < (uint)pointToNode.Length ? pointToNode[idx] : -1;
                if (ni < 0) return;

                sumR[ni] += rIdx < values.Length ? values[rIdx] : 0;
                sumG[ni] += gIdx < values.Length ? values[gIdx] : 0;
                sumB[ni] += bIdx < values.Length ? values[bIdx] : 0;
                if (aIdx >= 0 && aIdx < values.Length) sumA[ni] += values[aIdx];
                else sumA[ni] += 255.0 / colorScale; // default alpha = 1 after scaling
                counts[ni]++;
            },
            (current, total) => OnProgress?.Invoke(1, current, total));

        // Finalize average colors.
        for (int n = 0; n < nodeCount; n++)
        {
            int c = counts[n];
            if (c > 0)
            {
                nodes[n].AverageColor = new Color4(
                    Math.Clamp((float)(sumR[n] / c) * colorScale, 0f, 1f),
                    Math.Clamp((float)(sumG[n] / c) * colorScale, 0f, 1f),
                    Math.Clamp((float)(sumB[n] / c) * colorScale, 0f, 1f),
                    Math.Clamp((float)(sumA[n] / c) * colorScale, 0f, 1f));
            }
            else
            {
                nodes[n].AverageColor = new Color4(1f, 1f, 1f, 1f);
            }
            nodes[n].HasMetadata = true;
        }
    }

    private static void CollectAllNodes(OctreeNode node, List<OctreeNode> result)
    {
        result.Add(node);
        foreach (var child in node.Children)
            CollectAllNodes(child, result);
    }
}
