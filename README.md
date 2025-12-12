# OctreeFlow

A C# library for octree-based point cloud processing, designed for use with **VVVV Gamma** and as a standalone CLI tool.

## Features

- **PLY File Support**: Read PLY files with support for positions, colors, normals, intensity, and custom scalar values
- **Octree Generation**: Distance-based spatial partitioning with configurable parameters
- **Binary Format**: Efficient `.octree` format for streaming and real-time navigation
- **VVVV Gamma Compatible**: Uses Stride.Core.Mathematics types (Vector3, Color4, BoundingBox)


## CLI Usage

### Build an Octree

```bash
octreeflow build -i pointcloud.ply -o output

# With custom parameters
octreeflow build -i pointcloud.ply -o output -n 2000 -d 0.5 -r 0.5 -v
```

#### Options

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

### `.octree` Binary Format

| Section | Contents |
|---------|----------|
| Header | Magic "OCTR", version, point count, properties, bounds |
| Structure | JSON-serialized octree node hierarchy |
| Point Data | Binary point data (position, color, normal, intensity, scalars) |

### `.json` Structure

JSON representation of the octree hierarchy with bounding boxes, point indices, and metadata.

## Dependencies

- **Stride.Core.Mathematics** (4.2.0+): Vector3, Color4, BoundingBox types
- **System.Text.Json**: JSON serialization
- **System.CommandLine** (CLI only): Command-line parsing
