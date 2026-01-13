# OctreeFlow

A C# library for octree-based point cloud processing, designed for use with **VVVV Gamma** and as a standalone CLI tool.

## Features

- **PLY File Support**: Read PLY files with support for positions, colors, normals, intensity, and custom scalar values
- **Octree Generation**: Distance-based spatial partitioning with configurable parameters
- **Binary Format**: Efficient `.octree` format for streaming and real-time navigation
- **VVVV Gamma Compatible**: Outputs data as `Vector4[]` and `float[]` arrays ready for `DynamicBufferAdvanced<T>`
- **No GPU Dependencies**: Library handles traversal and caching only - you handle GPU upload in your patch

## VVVV Gamma API

The `OctreeFlow.Api` namespace provides a complete API for real-time octree traversal and RAM caching. GPU upload is handled by your vvvv gamma patch using `DynamicBufferAdvanced<T>`.

### Workflow Overview

```
┌────────────────┐    ┌────────────────┐    ┌────────────────┐
│   Traversal    │───>│    Caching     │───>│  Buffer Data   │
│   (Octree)     │    │    (RAM)       │    │  (Vector4[])   │
└────────────────┘    └────────────────┘    └────────────────┘
                                                     │
                                                     v
                                            ┌────────────────┐
                                            │ DynamicBuffer  │
                                            │ Advanced<T>    │
                                            │ (Your Patch)   │
                                            └────────────────┘
```

### Basic Usage with OctreeFlowReader

```csharp
using OctreeFlow.Api;

// 1. Create reader once
var reader = new OctreeFlowReader(
    octreePath: "pointcloud.octree",
    plyPath: "pointcloud.ply",
    cacheSizeMB: 512,      // RAM cache
    bufferSizeMB: 256,     // Buffer allocation
    maxPointsPerSector: 65536
);

// 2. Initialize (once)
reader.Initialize();

// 3. Each frame: UpdateFrame with your traversal logic
var result = reader.UpdateFrame(nodeInfo =>
{
    // Your LOD logic - e.g., based on camera distance
    bool accept = nodeInfo.Level <= targetDepth;
    bool display = accept && (nodeInfo.Level == targetDepth || nodeInfo.IsLeaf);
    bool continueChildren = nodeInfo.Level < targetDepth;
    
    return new TraversalDecision(accept, display, continueChildren);
});

// 4. Upload new sectors to your DynamicBufferAdvanced
foreach (var sector in result.NewSectors)
{
    // Upload to your buffers using byte offset
    positionBuffer.SetData(sector.Positions, sector.ByteOffset);
    colorBuffer.SetData(sector.Colors, sector.ByteOffset);
    normalBuffer.SetData(sector.Normals, sector.ByteOffset);
    intensityBuffer.SetData(sector.Intensities, sector.ByteOffset);
}

// 5. Render using active sectors
foreach (var sector in result.ActiveSectors)
{
    // sector.StartIndex - first point index in buffers
    // sector.PointCount - number of points to render
    DrawPoints(sector.StartIndex, sector.PointCount);
}
```

### Buffer Data Output

The library outputs data ready for vvvv gamma's `DynamicBufferAdvanced<T>`:

```csharp
// Each SectorData contains:
foreach (var sector in result.NewSectors)
{
    // For MutableArray<Vector4>
    Vector4[] positions = sector.Positions;   // xyz + w padding
    Vector4[] colors = sector.Colors;         // rgba
    Vector4[] normals = sector.Normals;       // xyz + w padding
    
    // For MutableArray<Float32>
    float[] intensities = sector.Intensities;
    
    // Additional scalars (e.g., classification)
    Dictionary<string, float[]> scalars = sector.Scalars;
    
    // Where to write in buffer
    int byteOffset = sector.ByteOffset;
    int sectorIndex = sector.SectorIndex;
    int pointCount = sector.PointCount;
}
```

### Buffer Configuration

```csharp
// Get buffer configuration from reader
var config = reader.BufferConfig;

// Total buffer size needed (for creating DynamicBufferAdvanced)
int totalCapacity = config.TotalCapacity;           // Points
int bytesForVector4 = config.TotalBytesVector4;     // For position/color/normal buffers
int bytesForFloat = config.TotalBytesFloat;         // For scalar buffers

// Or create custom configuration
var config = BufferConfiguration.FromBufferSize(
    bufferSizeMB: 256,
    maxPointsPerSector: 65536
);
```

### Sector-Based Rendering

The buffers are divided into **sectors** (one per node):

```
Buffer Layout:
┌──────────────┬──────────────┬──────────────┬───┐
│   Sector 0   │   Sector 1   │   Sector 2   │...│
│ Offset: 0    │ Offset: 1MB  │ Offset: 2MB  │   │
│ Points: 5000 │ Points: 3200 │ Points: 8000 │   │
│ Node: "2_0"  │ Node: "2_1"  │ Node: "2_2"  │   │
└──────────────┴──────────────┴──────────────┴───┘
```

- Each sector holds one node's worth of points
- Use `ByteOffset` when calling `SetData` on `DynamicBufferAdvanced`
- Use `StartIndex` and `PointCount` for rendering dispatch
- LRU eviction when buffer is full

### Active Sectors for Rendering

```csharp
// Active sectors to render this frame
foreach (var sector in result.ActiveSectors)
{
    sector.SectorIndex;       // Index in buffer
    sector.StartIndex;        // First point index (element)
    sector.ByteOffsetVector4; // Byte offset for Vector4 buffers
    sector.ByteOffsetFloat;   // Byte offset for float buffers
    sector.PointCount;        // Points to render
    sector.NodeId;            // Octree node ID
    sector.Level;             // Node depth level
}
```

### Traversal Delegate

The traversal delegate receives a `NodeInfo` object with:
- `Id`: Node identifier (e.g., "2_0_1_0")
- `BoundingBox`, `Center`, `Size`: Spatial information
- `PointCount`: Number of points in this node
- `Level`: Depth in tree (0 = root)
- `ChildCount`, `IsLeaf`: Structure information
- `IsInCache`, `IsOnGpu`, `GpuSectorIndex`: Loading status

Returns a `TraversalDecision`:
- `IsAccepted`: Include in caching nodes
- `IsForDisplay`: Include in viewing nodes (for buffer)
- `ContinueToChildren`: Recurse into children

**Predefined decisions:**
```csharp
TraversalDecision.Reject;           // Skip node, stop here
TraversalDecision.CacheOnly;        // Cache but don't display, continue to children
TraversalDecision.DisplayAndContinue; // Display and continue to children
TraversalDecision.DisplayAndStop;   // Display and stop (leaf-like)
TraversalDecision.SkipButContinue;  // Skip node but continue to children
```

### Async Operations

```csharp
// Async frame update (cache loading in background)
var result = await reader.UpdateFrameAsync(traversalDelegate, cancellationToken);

// Or separate async cache loading
var cacheResult = await reader.LoadToCacheAsync(nodes, cancellationToken);
```

### Result Information

```csharp
var result = reader.UpdateFrame(traversalDelegate);

// New data to upload this frame
result.HasNewData;              // True if NewSectors.Length > 0
result.NewSectors;              // SectorData[] to upload

// Active sectors for rendering
result.ActiveSectors;           // SectorInfo[] to render
result.TotalPointsInBuffer;     // Total points across all sectors

// Timing
result.TotalTimeMs;             // Total processing time

// Traversal stats
result.Traversal.NodesVisited;
result.Traversal.NodesAccepted;
result.Traversal.ViewingNodes;   // Nodes for display
result.Traversal.CachingNodes;   // Nodes to cache

// Cache stats  
result.CacheResult.LoadedNodes;
```

## VVVV Gamma Integration Example

In your vvvv gamma patch:

1. **Create Reader** (once in Create region):
   - Use `OctreeFlowReader` constructor
   - Call `Initialize()`

2. **Create Buffers** (once in Create region):
   - Create `DynamicBufferAdvanced<Vector4>` for positions, colors, normals
   - Create `DynamicBufferAdvanced<Float32>` for intensity/scalars
   - Use `BufferConfig.TotalBytesVector4` and `TotalBytesFloat` for sizes

3. **Update Loop** (per frame):
   - Call `UpdateFrame()` with your traversal delegate
   - For each `NewSector`: call `SetData` on buffers with `ByteOffset`
   - Use `ActiveSectors` for rendering dispatch

4. **Render**:
   - Use `ComputeStage` or `Sprite/Point` rendering
   - Index into buffers using `VertexID` or compute thread index
   - Filter by sector start/count for proper rendering

## CLI Usage

### Build an Octree

```bash
octreeflow build -i pointcloud.ply -o output

# With custom parameters
octreeflow build -i pointcloud.ply -o output -n 2000 -d 0.5 -r 0.5 -v
```

#### Build Options

| Option | Description | Default |
|--------|-------------|---------|
| `-i, --input` | Input PLY file (required) | - |
| `-o, --output` | Output path (without extension) | Same as input |
| `-n, --points-per-node` | Points per octree node | 1000 |
| `-d, --min-distance` | Minimum distance at level 0 | 1.0 |
| `-r, --level-ratio` | Distance ratio per level | 0.5 |
| `-s, --seed` | Random seed for reproducibility | - |
| `--max-depth` | Maximum tree depth (0 = unlimited) | 0 |
| `-v, --verbose` | Verbose output | false |

### View Octree Info

```bash
octreeflow info -i output.octree
```

## Octree Algorithm

1. **Configuration**: Set points per node (N), starting distance threshold, and level ratio
2. **Root Node**: Create root node encompassing all points
3. **Filling Cycle** (per node):
   - Randomly select points from available pool
   - Accept point if distance to all selected points > threshold
   - Repeat until node has N points or exhausted attempts
4. **Subdivision**: Split remaining points into 8 octants, recurse for each non-empty bin
5. **Termination**: Stop when no points remain

### Node ID Format

`depth_x_y_z` (e.g., `0_0_0_0` for root, `2_0_1_0_1_0_1` for deeper nodes)

## File Formats

### `.octree` Binary Format (v4)

| Section | Contents |
|---------|----------|
| Magic | "OCTR" (4 bytes) |
| Version | int32 |
| Point count | int32 |
| PLY path | length-prefixed string |
| Properties | count + length-prefixed strings |
| Bounds | 6 x float32 (min.xyz, max.xyz) |
| Node count | int32 |
| Nodes | Recursive depth-first with point indices |

## API Classes

| Class | Description |
|-------|-------------|
| `OctreeFlowReader` | Main reader - Traverse + Cache + Buffer data output |
| `SectorManager` | Sector allocation with LRU eviction |
| `CacheManager` | LRU RAM cache for point data |
| `BufferConfiguration` | Buffer size and offset calculations |
| `SectorData` | Data for a sector (Vector4[], float[], offsets) |
| `SectorInfo` | Info about active sector (for rendering) |
| `BufferUpdateResult` | Result with new sectors and active sectors |
| `FrameUpdateResult` | Complete frame update result |
| `TraversalResult` | Result of Traverse() with viewing/caching nodes |
| `NodeInfo` | Node metadata for traversal decisions |
| `TraversalDecision` | Return type from traversal delegate |

## Dependencies

- **Stride.Core.Mathematics** (4.2.0+): Vector3, Vector4, Color4, BoundingBox types
- **System.Text.Json**: JSON serialization
- **System.CommandLine** (CLI only): Command-line parsing

## Migration from Previous Version

If you were using the old GPU-managed API:

1. Replace `PointCloudLoader` / `PointCloudRenderer` with `OctreeFlowReader`
2. Remove `GraphicsDevice` from initialization - no longer needed
3. Create your own `DynamicBufferAdvanced<T>` buffers in vvvv gamma
4. Use `NewSectors` to upload data with `SetData` and byte offsets
5. Use `ActiveSectors` for rendering dispatch
