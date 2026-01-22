# OctreeFlow

A C# library for octree-based point cloud processing, designed for real-time streaming and LOD (Level of Detail) rendering.

## What It Does

OctreeFlow takes large PLY point clouds and organizes them into an octree structure that enables:

- **Progressive loading** - Load only the points you need based on camera position
- **Level of detail** - Coarser points at distance, finer points up close
- **Efficient caching** - RAM cache with LRU eviction for smooth streaming

## How It Works

### 1. Build Phase (Offline)

The octree is built using distance-based spatial partitioning:

1. Start with all points in a root node
2. For each node, randomly select points that are spaced apart by a distance threshold
3. Remaining points are split into 8 child octants and the process repeats
4. Distance threshold decreases at each level (controlled by a ratio)

This creates a natural LOD structure where higher levels contain sparser, more spread-out points.

### 2. Runtime Phase

At runtime, traverse the octree based on your camera/view:

1. **Traverse** - Walk the tree, deciding which nodes to display based on distance, screen size, etc.
2. **Cache** - Load selected nodes into RAM cache
3. **Buffer** - Copy point data into GPU-ready arrays (`Vector4[]` for positions/colors/normals)

The library handles traversal and caching. You handle the GPU upload and rendering (e.g., via VVVV Gamma's `DynamicBufferAdvanced<T>`).

## Quick Start

```bash
# Build an octree from a PLY file
octreeflow build -i pointcloud.ply -o output

# View octree info
octreeflow info -i output.octree
```

## Dependencies

- **Stride.Core.Mathematics** - Vector types
- **System.CommandLine** - CLI parsing
