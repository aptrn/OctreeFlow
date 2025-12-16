# OctreeFlow

A C# library for octree-based point cloud processing, designed for use with **VVVV Gamma** and as a standalone CLI tool.

## Features

- **PLY File Support**: Read PLY files with support for positions, colors, normals, intensity, and custom scalar values
- **Octree Generation**: Distance-based spatial partitioning with configurable parameters
- **Binary Format**: Efficient `.octree` format for streaming and real-time navigation
- **VVVV Gamma Compatible**: Uses Stride.Core.Mathematics types (Vector3, Color4, BoundingBox)
- **Streaming API**: RAM cache and GPU sector management for real-time traversal

## VVVV Gamma API

The `OctreeFlow.Api` namespace provides a complete API for real-time octree traversal and point cloud rendering with full GPU buffer management.

### Workflow: PointCloudLoader

The recommended workflow using `PointCloudLoader`:

```csharp
using OctreeFlow.Api;
using Stride.Graphics;

// 1. Create loader once
var loader = new PointCloudLoader(
    octreePath: "pointcloud.octree",
    plyPath: "pointcloud.ply",
    cacheSizeMB: 512,
    gpuBufferSizeMB: 256,
    maxPointsPerNode: 65536
);

// 2. Initialize with GraphicsDevice (once)
loader.Initialize(graphicsDevice);

// 3. Each frame: Traverse
var traversal = loader.Traverse(nodeInfo =>
{
    // Your LOD logic - e.g., based on camera distance
    bool accept = nodeInfo.Level <= targetDepth;
    bool display = accept && (nodeInfo.Level == targetDepth || nodeInfo.IsLeaf);
    bool continueChildren = nodeInfo.Level < targetDepth;
    
    return new TraversalDecision(accept, display, continueChildren);
});

// 4. Cache and Upload (async or sync)
var result = await loader.CacheAndUploadAsync(commandList, traversal);
// OR: var result = loader.CacheAndUpload(commandList, traversal);

// 5. Render using active sectors
foreach (var sector in result.ActiveSectors)
{
    // sector.StartIndex - first point index in buffers
    // sector.PointCount - number of points to render
    DrawInstanced(sector.StartIndex, sector.PointCount);
}
```

### Separate Buffers Per Attribute

The loader creates separate buffers for each attribute:

```csharp
// Position buffer (Vector3 per point)
Buffer positionBuffer = loader.PositionBuffer;

// Color buffer (Vector3 RGB per point)
Buffer colorBuffer = loader.ColorBuffer;

// Normal buffer (Vector3 per point) - may be null
Buffer normalBuffer = loader.NormalBuffer;

// Scalar buffers (float per point)
Buffer intensityBuffer = loader.GetScalarBuffer("intensity");

// All scalar buffers
var allScalars = loader.ScalarBuffers; // Dictionary<string, Buffer>

// Check available properties
bool hasNormals = loader.HasNormals;
var scalarNames = loader.ScalarProperties; // e.g., ["intensity", "classification"]
```

### Shader Usage (SDSL)

```hlsl
// Declare separate buffers
StructuredBuffer<float3> PositionBuffer;
StructuredBuffer<float3> ColorBuffer;
StructuredBuffer<float3> NormalBuffer;
StructuredBuffer<float> IntensityBuffer;

// In vertex shader - use SV_VertexID
void VSMain(uint vertexId : SV_VertexID)
{
    float3 pos = PositionBuffer[vertexId];
    float3 color = ColorBuffer[vertexId];
    float3 normal = NormalBuffer[vertexId];
    float intensity = IntensityBuffer[vertexId];
}
```

### Sector-Based Rendering

The GPU buffers are divided into **sectors** (one per node):

```
Buffer Layout:
┌──────────────┬──────────────┬──────────────┬───┐
│   Sector 0   │   Sector 1   │   Sector 2   │...│
│ StartIdx: 0  │ StartIdx: N  │ StartIdx: 2N │   │
│ Points: 5000 │ Points: 3200 │ Points: 8000 │   │
│ Node: "2_0"  │ Node: "2_1"  │ Node: "2_2"  │   │
└──────────────┴──────────────┴──────────────┴───┘
```

- Each sector can hold up to `maxPointsPerNode` points
- Sectors are updated independently (no full buffer re-upload)
- LRU eviction when buffer is full

### Result Information

```csharp
var result = loader.CacheAndUpload(commandList, traversal);

// Active sectors for rendering
foreach (var sector in result.ActiveSectors)
{
    sector.SectorIndex;  // Sector index
    sector.StartIndex;   // First point in buffer
    sector.PointCount;   // Points to render
    sector.NodeId;       // Octree node ID
    sector.Level;        // Node depth level
}

// Statistics
result.TotalPointsOnGpu;  // Total points in GPU buffers
result.SectorsUploaded;   // Sectors uploaded this frame
result.NodesCached;       // Nodes loaded to RAM cache
result.TotalTimeMs;       // Total processing time
```

### Advanced: Direct Buffer Access

For custom rendering pipelines:

```csharp
// Access the buffer manager
var gpuBuffers = loader.GpuBuffers;

// Buffer info
int sectorCount = gpuBuffers.SectorCount;
int sectorSize = gpuBuffers.SectorSizePoints;
int totalCapacity = gpuBuffers.TotalCapacity;

// Check if specific node is loaded
bool isLoaded = gpuBuffers.Contains("2_0_1_0");
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
- `IsForDisplay`: Include in viewing nodes (for GPU)
- `ContinueToChildren`: Recurse into children

### GPU Sector System

The GPU buffer is divided into sectors, each holding one node's worth of points:

```csharp
// Sector info
var activeSectors = reader.GpuManager.GetActiveSectors();
var sectorActivations = reader.GpuManager.GetSectorActivations();

// Check if node is on GPU
bool onGpu = reader.GpuManager.Contains(nodeId);
int sectorIndex = reader.GpuManager.GetSectorForNode(nodeId);
```

### GPU-Ready Point Format

```csharp
// Convert to GPU format (44 bytes per point)
var gpuPoints = GpuPointData.FromPointDataArray(pointData);
byte[] rawData = GpuPointData.ToByteArray(pointData);
```

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

### Demo Traversal API

```bash
octreeflow traverse -o output.octree -p pointcloud.ply -d 3 -c 256 -g 128
```

| Option | Description | Default |
|--------|-------------|---------|
| `-o, --octree` | Input .octree file (required) | - |
| `-p, --ply` | Input .ply file (required) | - |
| `-d, --max-depth` | Maximum traversal depth | 3 |
| `-c, --cache-size` | RAM cache size in MB | 256 |
| `-g, --gpu-size` | GPU buffer size in MB | 128 |

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

### `.json` Structure

JSON representation of the octree hierarchy with bounding boxes, point counts, and metadata (without point indices for smaller file size).

## API Classes

| Class | Description |
|-------|-------------|
| `PointCloudLoader` | **Recommended** - main loader with Traverse + CacheAndUpload |
| `PointCloudBuffers` | Multi-buffer GPU manager (Position, Color, Normal, Scalars) |
| `PointBufferSector` | Info about a sector (StartIndex, PointCount) |
| `TraversalResult` | Result of Traverse() with viewing/caching nodes |
| `CacheAndUploadResult` | Result with active sectors for rendering |
| `NodeInfo` | Node metadata for traversal decisions |
| `TraversalDecision` | Return type from traversal delegate |
| `CacheManager` | LRU RAM cache for point data |

## Dependencies

- **Stride.Core.Mathematics** (4.2.0+): Vector3, Color4, BoundingBox types
- **System.Text.Json**: JSON serialization
- **System.CommandLine** (CLI only): Command-line parsing
