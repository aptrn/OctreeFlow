using System.CommandLine;
using OctreeFlow.Api;
using OctreeFlow.Core;
using OctreeFlow.IO;

namespace OctreeFlow.CLI;

class Program
{
    static async Task<int> Main(string[] args)
    {
        var rootCommand = new RootCommand("OctreeFlow - Point cloud octree processor for VVVV Gamma");

        // Build command
        var buildCommand = CreateBuildCommand();
        rootCommand.AddCommand(buildCommand);

        // Info command
        var infoCommand = CreateInfoCommand();
        rootCommand.AddCommand(infoCommand);

        // Traverse command (demo API)
        var traverseCommand = CreateTraverseCommand();
        rootCommand.AddCommand(traverseCommand);

        return await rootCommand.InvokeAsync(args);
    }

    static Command CreateBuildCommand()
    {
        var inputOption = new Option<FileInfo>(
            aliases: new[] { "--input", "-i" },
            description: "Input PLY file path")
        { IsRequired = true };

        var outputOption = new Option<string>(
            aliases: new[] { "--output", "-o" },
            description: "Output file path (without extension, will create .octree and .json)");

        var pointsPerNodeOption = new Option<int>(
            aliases: new[] { "--points-per-node", "-n" },
            getDefaultValue: () => 1000,
            description: "Number of points per node");

        var minDistanceOption = new Option<float>(
            aliases: new[] { "--min-distance", "-d" },
            getDefaultValue: () => 1.0f,
            description: "Starting minimum distance between points at level 0");

        var ratioOption = new Option<float>(
            aliases: new[] { "--level-ratio", "-r" },
            getDefaultValue: () => 0.5f,
            description: "Distance threshold ratio for each subsequent level");

        var seedOption = new Option<int?>(
            aliases: new[] { "--seed", "-s" },
            description: "Random seed for reproducible results");

        var maxDepthOption = new Option<int>(
            aliases: new[] { "--max-depth" },
            getDefaultValue: () => 0,
            description: "Maximum octree depth (0 = unlimited)");

        var verboseOption = new Option<bool>(
            aliases: new[] { "--verbose", "-v" },
            getDefaultValue: () => false,
            description: "Enable verbose output");

        var buildCommand = new Command("build", "Build an octree from a PLY file")
        {
            inputOption,
            outputOption,
            pointsPerNodeOption,
            minDistanceOption,
            ratioOption,
            seedOption,
            maxDepthOption,
            verboseOption
        };

        buildCommand.SetHandler(async (context) =>
        {
            var input = context.ParseResult.GetValueForOption(inputOption)!;
            var output = context.ParseResult.GetValueForOption(outputOption);
            var pointsPerNode = context.ParseResult.GetValueForOption(pointsPerNodeOption);
            var minDistance = context.ParseResult.GetValueForOption(minDistanceOption);
            var ratio = context.ParseResult.GetValueForOption(ratioOption);
            var seed = context.ParseResult.GetValueForOption(seedOption);
            var maxDepth = context.ParseResult.GetValueForOption(maxDepthOption);
            var verbose = context.ParseResult.GetValueForOption(verboseOption);

            await BuildOctree(input, output, pointsPerNode, minDistance, ratio, seed, maxDepth, verbose);
        });

        return buildCommand;
    }

    static Command CreateInfoCommand()
    {
        var inputOption = new Option<FileInfo>(
            aliases: new[] { "--input", "-i" },
            description: "Input .octree or .json file path")
        { IsRequired = true };

        var infoCommand = new Command("info", "Display information about an .octree or .json file")
        {
            inputOption
        };

        infoCommand.SetHandler((input) =>
        {
            DisplayInfo(input);
        }, inputOption);

        return infoCommand;
    }

    static Task BuildOctree(
        FileInfo input,
        string? output,
        int pointsPerNode,
        float minDistance,
        float ratio,
        int? seed,
        int maxDepth,
        bool verbose)
    {
        if (!input.Exists)
        {
            Console.ForegroundColor = ConsoleColor.Red;
            Console.WriteLine($"Error: Input file not found: {input.FullName}");
            Console.ResetColor();
            return Task.CompletedTask;
        }

        // Default output path
        output ??= Path.Combine(
            input.DirectoryName ?? ".",
            Path.GetFileNameWithoutExtension(input.Name));

        Console.WriteLine("╔══════════════════════════════════════════════════════════════╗");
        Console.WriteLine("║              OctreeFlow Builder (Streaming Mode)             ║");
        Console.WriteLine("╚══════════════════════════════════════════════════════════════╝");
        Console.WriteLine();

        Console.WriteLine($"Input:           {input.FullName}");
        Console.WriteLine($"Output:          {output}.octree / {output}.json");
        Console.WriteLine($"Points per node: {pointsPerNode}");
        Console.WriteLine($"Min distance:    {minDistance}");
        Console.WriteLine($"Level ratio:     {ratio}");
        Console.WriteLine($"Max depth:       {(maxDepth == 0 ? "unlimited" : maxDepth)}");
        if (seed.HasValue)
            Console.WriteLine($"Random seed:     {seed.Value}");
        Console.WriteLine();

        var config = new OctreeConfiguration
        {
            PointsPerNode = pointsPerNode,
            StartingMinDistance = minDistance,
            LevelThresholdRatio = ratio,
            RandomSeed = seed,
            MaxDepth = maxDepth
        };

        var stopwatch = System.Diagnostics.Stopwatch.StartNew();
        OctreeNode root;
        PlyIndex? plyIndex = null;
        int nodeCount = 0;
        int lastLevel = -1;
        int lastPercent = -1;

        try
        {
            using var builder = new StreamingOctreeBuilder(config, input.FullName);

            builder.OnProgress = (phase, current, total, depth) =>
            {
                int percent = total > 0 ? (int)(100.0 * current / total) : 0;
                
                // Always show progress (simplified for non-verbose, detailed for verbose)
                if (percent != lastPercent || verbose)
                {
                    lastPercent = percent;
                    string phaseName = phase switch
                    {
                        0 => "Indexing",
                        1 => "Building",
                        2 => "Writing",
                        _ => "Processing"
                    };

                    if (verbose)
                    {
                        Console.Write($"\r  {phaseName}: {current:N0}/{total:N0} ({percent}%)      ");
                    }
                    else
                    {
                        Console.Write($"\r  {phaseName}: {percent}%   ");
                    }
                }
            };

            builder.OnNodeCompleted = node =>
            {
                nodeCount++;
                if (verbose && node.Level != lastLevel)
                {
                    Console.WriteLine();
                    Console.Write($"  Level {node.Level}: ");
                    lastLevel = node.Level;
                }
                else if (verbose)
                {
                    Console.Write(".");
                }
            };

            Console.WriteLine("Building octree...");
            root = builder.Build();
            plyIndex = builder.GetPlyIndex();
            
            // Clear the progress line
            Console.Write("\r                                                    \r");
        }
        catch (Exception ex)
        {
            Console.ForegroundColor = ConsoleColor.Red;
            Console.WriteLine();
            Console.WriteLine($"Error during build: {ex.Message}");
            if (verbose)
                Console.WriteLine(ex.StackTrace);
            Console.ResetColor();
            return Task.CompletedTask;
        }

        stopwatch.Stop();

        if (verbose) Console.WriteLine();
        Console.WriteLine();
        Console.ForegroundColor = ConsoleColor.Green;
        Console.WriteLine($"Build complete! ({FormatTime(stopwatch.Elapsed)})");
        Console.ResetColor();

        Console.WriteLine($"  Total nodes:    {root.GetTotalNodeCount():N0}");
        Console.WriteLine($"  Total points:   {root.GetTotalPointCount():N0}");
        Console.WriteLine($"  Max depth:      {root.GetMaxDepth()}");

        if (plyIndex != null)
        {
            Console.WriteLine($"  Bounds:         {plyIndex.Bounds.Minimum} - {plyIndex.Bounds.Maximum}");
            Console.WriteLine($"  Properties:     {string.Join(", ", plyIndex.Properties.Select(p => p.Name))}");
        }
        Console.WriteLine();

        // Save files
        Console.Write("Saving .octree file... ");
        stopwatch.Restart();

        var serializer = new StreamingOctreeSerializer();
        serializer.OnProgress = (current, total) =>
        {
            if (verbose)
                Console.Write($"\r  Writing: {current:N0}/{total:N0} ({100.0 * current / total:F1}%)      ");
        };

        try
        {
            if (plyIndex != null)
            {
                serializer.SaveOctreeFile(root, plyIndex, input.FullName, output + ".octree");
            }
            stopwatch.Stop();
            Console.ForegroundColor = ConsoleColor.Green;
            Console.WriteLine($"Done! ({FormatTime(stopwatch.Elapsed)})");
            Console.ResetColor();
        }
        catch (Exception ex)
        {
            stopwatch.Stop();
            Console.ForegroundColor = ConsoleColor.Red;
            Console.WriteLine($"Failed!");
            
            // Show the full exception message (which should include our detailed info)
            Console.WriteLine($"Error: {ex.Message}");
            
            // If there's an inner exception, show it too
            if (ex.InnerException != null)
            {
                Console.WriteLine($"Inner exception type: {ex.InnerException.GetType().Name}");
                Console.WriteLine($"Inner exception: {ex.InnerException.Message}");
            }
            
            // Always show exception type for debugging
            Console.WriteLine($"Exception type: {ex.GetType().Name}");
            
            if (verbose)
            {
                Console.WriteLine($"Stack trace: {ex.StackTrace}");
                if (ex.InnerException != null)
                    Console.WriteLine($"Inner stack trace: {ex.InnerException.StackTrace}");
            }
            Console.ResetColor();
            
            // Check if file was created despite error
            var octreeFile = new FileInfo(output + ".octree");
            if (octreeFile.Exists)
            {
                Console.WriteLine($"Note: .octree file was created ({FormatFileSize(octreeFile.Length)}) but may be incomplete.");
            }
            
            return Task.CompletedTask;
        }

        Console.Write("Saving structure JSON... ");
        stopwatch.Restart();

        try
        {
            serializer.SaveStructureJson(root, output + ".json");
            stopwatch.Stop();
            Console.ForegroundColor = ConsoleColor.Green;
            Console.WriteLine($"Done! ({stopwatch.ElapsedMilliseconds}ms)");
            Console.ResetColor();
        }
        catch (Exception ex)
        {
            Console.ForegroundColor = ConsoleColor.Red;
            Console.WriteLine($"Failed!");
            Console.WriteLine($"Error: {ex.Message}");
            Console.ResetColor();
        }

        // File sizes
        var octreeFileInfo = new FileInfo(output + ".octree");
        var jsonFileInfo = new FileInfo(output + ".json");

        Console.WriteLine();
        Console.WriteLine($"Output files:");
        if (octreeFileInfo.Exists)
            Console.WriteLine($"  {octreeFileInfo.Name}: {FormatFileSize(octreeFileInfo.Length)}");
        if (jsonFileInfo.Exists)
            Console.WriteLine($"  {jsonFileInfo.Name}: {FormatFileSize(jsonFileInfo.Length)}");
        Console.WriteLine();
        Console.WriteLine("Done!");

        // Cleanup
        plyIndex?.Dispose();

        return Task.CompletedTask;
    }

    static void DisplayInfo(FileInfo input)
    {
        if (!input.Exists)
        {
            Console.ForegroundColor = ConsoleColor.Red;
            Console.WriteLine($"Error: File not found: {input.FullName}");
            Console.ResetColor();
            return;
        }

        Console.WriteLine($"File: {input.FullName}");
        Console.WriteLine($"Size: {FormatFileSize(input.Length)}");
        Console.WriteLine();

        try
        {
            var serializer = new StreamingOctreeSerializer();
            OctreeNode? root = null;

            if (input.Extension.Equals(".json", StringComparison.OrdinalIgnoreCase))
            {
                root = serializer.LoadStructureJson(input.FullName);
            }
            else
            {
                // For .octree files, we'd need the full deserializer
                // For now just load JSON structure if available
                var jsonPath = Path.ChangeExtension(input.FullName, ".json");
                if (File.Exists(jsonPath))
                {
                    Console.WriteLine($"(Reading structure from {Path.GetFileName(jsonPath)})");
                    Console.WriteLine();
                    root = serializer.LoadStructureJson(jsonPath);
                }
                else
                {
                    Console.WriteLine("Note: JSON structure file not found. Limited info available.");
                    Console.WriteLine();
                }
            }

            if (root != null)
            {
                Console.WriteLine("Structure:");
                Console.WriteLine($"  Root ID:      {root.Id}");
                Console.WriteLine($"  Total nodes:  {root.GetTotalNodeCount():N0}");
                Console.WriteLine($"  Total points: {root.GetTotalPointCount():N0}");
                Console.WriteLine($"  Max depth:    {root.GetMaxDepth()}");
                Console.WriteLine();

                // Level statistics
                Console.WriteLine("Nodes per level:");
                var levelCounts = new Dictionary<int, int>();
                var levelPoints = new Dictionary<int, int>();
                CountStatsByLevel(root, levelCounts, levelPoints);

                foreach (var kvp in levelCounts.OrderBy(x => x.Key))
                {
                    Console.WriteLine($"  Level {kvp.Key}: {kvp.Value:N0} nodes, {levelPoints[kvp.Key]:N0} points");
                }
            }
        }
        catch (Exception ex)
        {
            Console.ForegroundColor = ConsoleColor.Red;
            Console.WriteLine($"Error reading file: {ex.Message}");
            Console.ResetColor();
        }
    }

    static void CountStatsByLevel(OctreeNode node, Dictionary<int, int> nodeCounts, Dictionary<int, int> pointCounts)
    {
        if (!nodeCounts.ContainsKey(node.Level))
        {
            nodeCounts[node.Level] = 0;
            pointCounts[node.Level] = 0;
        }
        nodeCounts[node.Level]++;
        pointCounts[node.Level] += node.PointIndices.Count;

        foreach (var child in node.Children)
        {
            CountStatsByLevel(child, nodeCounts, pointCounts);
        }
    }

    static string FormatFileSize(long bytes)
    {
        string[] sizes = { "B", "KB", "MB", "GB", "TB" };
        int order = 0;
        double size = bytes;

        while (size >= 1024 && order < sizes.Length - 1)
        {
            order++;
            size /= 1024;
        }

        return $"{size:0.##} {sizes[order]}";
    }

    static string FormatTime(TimeSpan time)
    {
        if (time.TotalMinutes >= 1)
            return $"{time.TotalMinutes:F1} minutes";
        if (time.TotalSeconds >= 1)
            return $"{time.TotalSeconds:F1} seconds";
        return $"{time.TotalMilliseconds:F0}ms";
    }

    static Command CreateTraverseCommand()
    {
        var octreeOption = new Option<FileInfo>(
            aliases: new[] { "--octree", "-o" },
            description: "Input .octree file path")
        { IsRequired = true };

        var plyOption = new Option<FileInfo>(
            aliases: new[] { "--ply", "-p" },
            description: "Input .ply file path")
        { IsRequired = true };

        var maxDepthOption = new Option<int>(
            aliases: new[] { "--max-depth", "-d" },
            getDefaultValue: () => 3,
            description: "Maximum depth to traverse");

        var cacheSizeOption = new Option<int>(
            aliases: new[] { "--cache-size", "-c" },
            getDefaultValue: () => 256,
            description: "RAM cache size in MB");

        var gpuSizeOption = new Option<int>(
            aliases: new[] { "--gpu-size", "-g" },
            getDefaultValue: () => 128,
            description: "GPU buffer size in MB");

        var traverseCommand = new Command("traverse", "Traverse an octree and demonstrate the API")
        {
            octreeOption,
            plyOption,
            maxDepthOption,
            cacheSizeOption,
            gpuSizeOption
        };

        traverseCommand.SetHandler(async (context) =>
        {
            var octree = context.ParseResult.GetValueForOption(octreeOption)!;
            var ply = context.ParseResult.GetValueForOption(plyOption)!;
            var maxDepth = context.ParseResult.GetValueForOption(maxDepthOption);
            var cacheSize = context.ParseResult.GetValueForOption(cacheSizeOption);
            var gpuSize = context.ParseResult.GetValueForOption(gpuSizeOption);

            await DemoTraversal(octree, ply, maxDepth, cacheSize, gpuSize);
        });

        return traverseCommand;
    }

    static async Task DemoTraversal(
        FileInfo octreeFile,
        FileInfo plyFile,
        int maxDepth,
        int cacheSizeMB,
        int gpuSizeMB)
    {
        if (!octreeFile.Exists)
        {
            Console.ForegroundColor = ConsoleColor.Red;
            Console.WriteLine($"Error: Octree file not found: {octreeFile.FullName}");
            Console.ResetColor();
            return;
        }

        if (!plyFile.Exists)
        {
            Console.ForegroundColor = ConsoleColor.Red;
            Console.WriteLine($"Error: PLY file not found: {plyFile.FullName}");
            Console.ResetColor();
            return;
        }

        Console.WriteLine("╔══════════════════════════════════════════════════════════════╗");
        Console.WriteLine("║              OctreeFlow API Demo (Traversal)                 ║");
        Console.WriteLine("╚══════════════════════════════════════════════════════════════╝");
        Console.WriteLine();

        Console.WriteLine($"Octree:      {octreeFile.FullName}");
        Console.WriteLine($"PLY:         {plyFile.FullName}");
        Console.WriteLine($"Max depth:   {maxDepth}");
        Console.WriteLine($"Cache size:  {cacheSizeMB} MB");
        Console.WriteLine($"GPU size:    {gpuSizeMB} MB");
        Console.WriteLine();

        try
        {
            Console.Write("Initializing reader... ");
            var sw = System.Diagnostics.Stopwatch.StartNew();
            
            // Convert MB to bytes for the API
            long gpuSizeBytes = (long)gpuSizeMB * 1024 * 1024;
            
            using var reader = new OctreeFlowReader(
                octreeFile.FullName,
                plyFile.FullName,
                cacheSizeMB,
                gpuSizeBytes);

            await reader.InitializeAsync((status, current, total) =>
            {
                int percent = total > 0 ? (int)(100.0 * current / total) : 0;
                Console.Write($"\rInitializing reader... {status} {percent}%   ");
            });

            sw.Stop();
            Console.WriteLine($"\rInitializing reader... Done! ({FormatTime(sw.Elapsed)})");
            Console.WriteLine();

            Console.WriteLine($"Total nodes:  {reader.TotalNodes:N0}");
            Console.WriteLine($"Total points: {reader.TotalPoints:N0}");
            Console.WriteLine($"Bounds:       {reader.Bounds.Minimum} - {reader.Bounds.Maximum}");
            Console.WriteLine();

            // Display available features
            Console.WriteLine("Available features (Vector4):");
            foreach (var feature in reader.FeaturesVector4)
            {
                Console.WriteLine($"  {feature.Key}");
            }
            Console.WriteLine("Available features (Float32):");
            foreach (var feature in reader.FeaturesFloat32)
            {
                Console.WriteLine($"  {feature.Key}");
            }
            Console.WriteLine();

            // Define a traversal delegate that accepts nodes up to maxDepth
            // and marks leaf nodes or nodes at maxDepth for display
            Console.WriteLine($"Traversing (max depth = {maxDepth})...");
            sw.Restart();

            var traversalResult = reader.Traverse(nodeInfo =>
            {
                // Accept all nodes up to maxDepth
                bool accept = nodeInfo.Level <= maxDepth;
                
                // Mark for display if at the target depth or is a leaf
                bool display = accept && (nodeInfo.Level == maxDepth || nodeInfo.IsLeaf);
                
                // Continue to children only if not at max depth
                bool continueChildren = nodeInfo.Level < maxDepth;

                return new TraversalDecision(accept, display, continueChildren);
            });

            sw.Stop();
            Console.WriteLine($"Traversal complete! ({traversalResult.TraversalTimeMs}ms)");
            Console.WriteLine($"  Nodes visited:  {traversalResult.NodesVisited:N0}");
            Console.WriteLine($"  Nodes accepted: {traversalResult.NodesAccepted:N0}");
            Console.WriteLine($"  Caching nodes:  {traversalResult.CachingNodes.Count:N0} ({traversalResult.TotalCachingPoints:N0} points)");
            Console.WriteLine($"  Viewing nodes:  {traversalResult.ViewingNodes.Count:N0} ({traversalResult.TotalViewingPoints:N0} points)");
            Console.WriteLine();

            // Load to cache
            Console.Write("Loading to cache... ");
            sw.Restart();

            var cacheResult = await reader.LoadToCacheAsync(traversalResult.CachingNodes);

            sw.Stop();
            Console.WriteLine($"Done! ({cacheResult.LoadTimeMs}ms)");
            Console.WriteLine($"  Nodes loaded:  {cacheResult.NodesLoaded:N0}");
            Console.WriteLine($"  Points loaded: {cacheResult.TotalPointsLoaded:N0}");
            Console.WriteLine($"  Cache version: {cacheResult.Version}");
            Console.WriteLine();

            // Update sector manager (buffer data output)
            Console.Write("Preparing buffer data... ");
            sw.Restart();

            var bufferResult = reader.SectorManager!.Update(traversalResult.ViewingNodes);

            sw.Stop();
            Console.WriteLine($"Done! ({bufferResult.UpdateTimeMs}ms)");
            Console.WriteLine($"  Nodes loaded:   {bufferResult.NodesLoaded:N0}");
            Console.WriteLine($"  Points in buffer: {bufferResult.TotalPointsInBuffer:N0}");
            Console.WriteLine($"  Active sectors: {bufferResult.ActiveSectors.Length}");
            Console.WriteLine($"  New sectors:    {bufferResult.NewSectors.Count}");
            Console.WriteLine($"  Buffer version: {bufferResult.Version}");
            Console.WriteLine();

            // Display sector info
            var activeSectors = bufferResult.ActiveSectors.Take(10).ToList();
            if (activeSectors.Any())
            {
                Console.WriteLine("Sample buffer sectors:");
                foreach (var sector in activeSectors)
                {
                    Console.WriteLine($"  Sector {sector.SectorIndex}: Node {sector.NodeId}, {sector.PointCount:N0} points");
                    Console.WriteLine($"    ByteOffset Vector4: {sector.ByteOffsetVector4}, Float: {sector.ByteOffsetFloat}");
                }
                if (bufferResult.ActiveSectors.Length > 10)
                {
                    Console.WriteLine($"  ... and {bufferResult.ActiveSectors.Length - 10} more");
                }
            }
            
            // Display new sector feature data structure
            if (bufferResult.NewSectors.Count > 0)
            {
                Console.WriteLine();
                Console.WriteLine("New sector data structure (first sector):");
                var firstSector = bufferResult.NewSectors[0];
                Console.WriteLine($"  Sector {firstSector.SectorIndex}: {firstSector.PointCount} points");
                Console.WriteLine($"  ByteOffsetVector4: {firstSector.ByteOffsetVector4}");
                Console.WriteLine($"  ByteOffsetFloat32: {firstSector.ByteOffsetFloat32}");
                Console.WriteLine($"  Vector4 Features:");
                if (firstSector.HasPosition)
                    Console.WriteLine($"    Position: {firstSector.PositionData!.Length} elements");
                if (firstSector.HasColors)
                    Console.WriteLine($"    Colors: {firstSector.ColorsData!.Length} elements");
                if (firstSector.HasNormals)
                    Console.WriteLine($"    Normals: {firstSector.NormalsData!.Length} elements");
                Console.WriteLine($"  Float32 Features:");
                if (firstSector.HasIntensity)
                    Console.WriteLine($"    Intensity: {firstSector.IntensityData!.Length} elements");
                foreach (var feature in firstSector.ScalarFeatures)
                {
                    Console.WriteLine($"    {feature.Key}: {feature.Value.Length} elements");
                }
            }

            Console.WriteLine();
            Console.ForegroundColor = ConsoleColor.Green;
            Console.WriteLine("API demo complete!");
            Console.ResetColor();

            // Now demo the simpler UpdateFrame API
            Console.WriteLine();
            Console.WriteLine("═══════════════════════════════════════════════════════════════");
            Console.WriteLine("  Demonstrating simplified UpdateFrame API");
            Console.WriteLine("═══════════════════════════════════════════════════════════════");
            Console.WriteLine();

            // Simulate a few "frames" with different traversal depths
            for (int frame = 0; frame < 3; frame++)
            {
                int targetDepth = 2 + frame; // Increasing depth each frame
                
                // Step 1: Traverse (separate from buffer update)
                var traversal = reader.Traverse(nodeInfo =>
                {
                    bool accept = nodeInfo.Level <= targetDepth;
                    bool display = accept && (nodeInfo.Level == targetDepth || nodeInfo.IsLeaf);
                    bool continueChildren = nodeInfo.Level < targetDepth;
                    return new TraversalDecision(accept, display, continueChildren);
                });

                // Step 2: Update buffer with traversal result
                var frameResult = reader.UpdateFrame(traversal);

                Console.WriteLine($"Frame {frame + 1} (depth={targetDepth}):");
                Console.WriteLine($"  Total time:        {frameResult.TotalTimeMs}ms");
                Console.WriteLine($"  Viewing nodes:     {frameResult.Traversal.ViewingNodes.Count}");
                Console.WriteLine($"  New sectors:       {frameResult.NewSectors.Count}");
                Console.WriteLine($"  Points in buffer:  {frameResult.TotalPointsInBuffer:N0}");
                Console.WriteLine($"  Active sectors:    {frameResult.ActiveSectors.Length}");
                Console.WriteLine();
            }
        }
        catch (Exception ex)
        {
            Console.ForegroundColor = ConsoleColor.Red;
            Console.WriteLine($"\nError: {ex.Message}");
            Console.WriteLine(ex.StackTrace);
            Console.ResetColor();
        }
    }
}
