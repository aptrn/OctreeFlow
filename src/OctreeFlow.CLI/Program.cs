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

        // Batch command (process folder)
        var batchCommand = CreateBatchCommand();
        rootCommand.AddCommand(batchCommand);

        // Update command (add/refresh per-node metadata without rebuilding)
        var updateCommand = CreateUpdateCommand();
        rootCommand.AddCommand(updateCommand);

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
            description: "Output file path (without extension, will create .octree)");

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

        var threadsOption = new Option<int>(
            aliases: new[] { "--threads", "-t" },
            getDefaultValue: () => 0,
            description: "Number of threads to use (0 = auto-detect based on CPU cores)");

        var skipMetadataOption = new Option<bool>(
            aliases: new[] { "--skip-metadata" },
            getDefaultValue: () => false,
            description: "Skip computing per-node metadata (average color, point density) after build");

        var buildCommand = new Command("build", "Build an octree from a PLY file")
        {
            inputOption,
            outputOption,
            pointsPerNodeOption,
            minDistanceOption,
            ratioOption,
            seedOption,
            maxDepthOption,
            verboseOption,
            threadsOption,
            skipMetadataOption
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
            var threads = context.ParseResult.GetValueForOption(threadsOption);
            var skipMetadata = context.ParseResult.GetValueForOption(skipMetadataOption);

            await BuildOctree(input, output, pointsPerNode, minDistance, ratio, seed, maxDepth, verbose, threads, skipMetadata);
        });

        return buildCommand;
    }

    static Command CreateInfoCommand()
    {
        var inputOption = new Option<FileInfo>(
            aliases: new[] { "--input", "-i" },
            description: "Input .octree file path")
        { IsRequired = true };

        var infoCommand = new Command("info", "Display information about an .octree file")
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
        bool verbose,
        int threads,
        bool skipMetadata = false)
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

        // Determine thread count
        int actualThreads = threads > 0 ? threads : Math.Max(1, Environment.ProcessorCount - 1);

        Console.WriteLine("╔══════════════════════════════════════════════════════════════╗");
        Console.WriteLine("║           OctreeFlow Builder (Parallel Streaming Mode)       ║");
        Console.WriteLine("╚══════════════════════════════════════════════════════════════╝");
        Console.WriteLine();

        Console.WriteLine($"Input:           {input.FullName}");
        Console.WriteLine($"File size:       {FormatFileSize(input.Length)}");
        Console.WriteLine($"Output:          {output}.octree");
        Console.WriteLine($"Points per node: {pointsPerNode}");
        Console.WriteLine($"Min distance:    {minDistance}");
        Console.WriteLine($"Level ratio:     {ratio}");
        Console.WriteLine($"Max depth:       {(maxDepth == 0 ? "unlimited" : maxDepth)}");
        Console.WriteLine($"Threads:         {actualThreads}");
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
        var lastProgressTime = DateTime.UtcNow;

        try
        {
            // Use the parallel builder for better performance
            using var builder = new ParallelStreamingOctreeBuilder(config, input.FullName, actualThreads);

            builder.OnProgress = (phase, current, total, depth) =>
            {
                int percent = total > 0 ? (int)(100.0 * current / total) : 0;
                
                // Throttle progress updates to avoid console spam
                var now = DateTime.UtcNow;
                bool shouldUpdate = percent != lastPercent || (now - lastProgressTime).TotalMilliseconds > 250;
                
                if (shouldUpdate)
                {
                    lastPercent = percent;
                    lastProgressTime = now;
                    
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
                Interlocked.Increment(ref nodeCount);
                if (verbose && node.Level != lastLevel)
                {
                    Console.WriteLine();
                    Console.Write($"  Level {node.Level}: ");
                    Interlocked.Exchange(ref lastLevel, node.Level);
                }
                else if (verbose)
                {
                    Console.Write(".");
                }
            };

            Console.WriteLine("Building octree (parallel mode)...");
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

        // Compute per-node metadata (average color, point density)
        if (!skipMetadata && plyIndex != null)
        {
            Console.Write("Computing node metadata... ");
            stopwatch.Restart();
            int lastMetaPhase = -1;
            int lastMetaPercent = -1;
            var lastMetaTime = DateTime.UtcNow;

            try
            {
                var metaComputer = new NodeMetadataComputer();
                metaComputer.OnProgress = (phase, current, total) =>
                {
                    int pct = total > 0 ? (int)(100.0 * current / total) : 0;
                    var now = DateTime.UtcNow;
                    if (pct != lastMetaPercent || phase != lastMetaPhase || (now - lastMetaTime).TotalMilliseconds > 250)
                    {
                        lastMetaPercent = pct;
                        lastMetaPhase = phase;
                        lastMetaTime = now;
                        string phaseName = phase == 0 ? "Mapping" : "Colors";
                        Console.Write($"\rComputing node metadata... {phaseName}: {pct}%   ");
                    }
                };
                metaComputer.Compute(root, plyIndex);
                stopwatch.Stop();
                Console.Write("\r                                                          \r");
                Console.ForegroundColor = ConsoleColor.Green;
                Console.Write("Computing node metadata... Done!");
                Console.ResetColor();
                Console.WriteLine($" ({FormatTime(stopwatch.Elapsed)})");
                Console.WriteLine();
            }
            catch (Exception ex)
            {
                stopwatch.Stop();
                Console.ForegroundColor = ConsoleColor.Yellow;
                Console.WriteLine($"\nWarning: Metadata computation failed ({ex.Message}). Saving without metadata.");
                Console.ResetColor();
                Console.WriteLine();
            }
        }

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
                serializer.SaveOctreeFile(root, plyIndex, input.FullName, output + ".octree", pointsPerNode);
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

        // File sizes
        var octreeFileInfo = new FileInfo(output + ".octree");

        Console.WriteLine();
        Console.WriteLine($"Output file:");
        if (octreeFileInfo.Exists)
            Console.WriteLine($"  {octreeFileInfo.Name}: {FormatFileSize(octreeFileInfo.Length)}");
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
            OctreeFileInfo? fileInfo = null;

            if (input.Extension.Equals(".octree", StringComparison.OrdinalIgnoreCase))
            {
                var (loadedRoot, info) = serializer.LoadOctreeFile(input.FullName);
                root = loadedRoot;
                fileInfo = info;

                if (fileInfo != null)
                {
                    Console.WriteLine("File Info:");
                    Console.WriteLine($"  Version:        {fileInfo.Version}");
                    Console.WriteLine($"  Total points:   {fileInfo.TotalPoints:N0}");
                    Console.WriteLine($"  Points/node:    {(fileInfo.PointsPerNode > 0 ? fileInfo.PointsPerNode.ToString("N0") : "not specified (legacy file)")}");
                    Console.WriteLine($"  PLY path:       {fileInfo.PlyPath}");
                    Console.WriteLine($"  Properties:     {string.Join(", ", fileInfo.PropertyNames)}");
                    Console.WriteLine($"  Bounds:         {fileInfo.Bounds.Minimum} - {fileInfo.Bounds.Maximum}");
                    Console.WriteLine($"  Node count:     {fileInfo.NodeCount:N0}");
                    Console.WriteLine();
                }
            }
            else
            {
                Console.WriteLine("Note: Only .octree files are supported for info display.");
                Console.WriteLine();
                return;
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

    static Command CreateUpdateCommand()
    {
        var inputOption = new Option<string>(
            aliases: new[] { "--input", "-i" },
            description: "Path to a .octree file or a folder containing .octree files to update")
        { IsRequired = true };

        var plyOption = new Option<string?>(
            aliases: new[] { "--ply", "-p" },
            description: "Override PLY file path (single-file mode only; ignored for folder input)");

        var recursiveOption = new Option<bool>(
            aliases: new[] { "--recursive", "-r" },
            getDefaultValue: () => false,
            description: "Search for .octree files recursively in subfolders (folder mode only)");

        var forceOption = new Option<bool>(
            aliases: new[] { "--force" },
            getDefaultValue: () => false,
            description: "Recompute metadata even if the file already has it");

        var updateCommand = new Command(
            "update",
            "Recompute per-node metadata (average color, point density) for existing .octree files without rebuilding")
        {
            inputOption,
            plyOption,
            recursiveOption,
            forceOption
        };

        updateCommand.SetHandler(async (context) =>
        {
            var input = context.ParseResult.GetValueForOption(inputOption)!;
            var ply = context.ParseResult.GetValueForOption(plyOption);
            var recursive = context.ParseResult.GetValueForOption(recursiveOption);
            var force = context.ParseResult.GetValueForOption(forceOption);
            await UpdateOctreeFiles(input, ply, recursive, force);
        });

        return updateCommand;
    }

    static Task UpdateOctreeFiles(string inputPath, string? plyOverride, bool recursive, bool force)
    {
        Console.WriteLine("╔══════════════════════════════════════════════════════════════╗");
        Console.WriteLine("║              OctreeFlow Metadata Updater                     ║");
        Console.WriteLine("╚══════════════════════════════════════════════════════════════╝");
        Console.WriteLine();

        // Collect .octree files to process.
        List<string> octreeFiles;
        if (File.Exists(inputPath) && inputPath.EndsWith(".octree", StringComparison.OrdinalIgnoreCase))
        {
            octreeFiles = new List<string> { inputPath };
        }
        else if (Directory.Exists(inputPath))
        {
            var searchOption = recursive ? SearchOption.AllDirectories : SearchOption.TopDirectoryOnly;
            octreeFiles = Directory
                .GetFiles(inputPath, "*.octree", searchOption)
                .OrderBy(f => f)
                .ToList();
        }
        else
        {
            Console.ForegroundColor = ConsoleColor.Red;
            Console.WriteLine($"Error: '{inputPath}' is not a valid .octree file or directory.");
            Console.ResetColor();
            return Task.CompletedTask;
        }

        if (octreeFiles.Count == 0)
        {
            Console.WriteLine("No .octree files found.");
            return Task.CompletedTask;
        }

        Console.WriteLine($"Found {octreeFiles.Count} .octree file(s) to process.");
        Console.WriteLine();

        int processed = 0, skipped = 0, failed = 0;
        var serializer = new StreamingOctreeSerializer();
        var totalSw = System.Diagnostics.Stopwatch.StartNew();

        foreach (var octreePath in octreeFiles)
        {
            Console.WriteLine($"[{processed + skipped + failed + 1}/{octreeFiles.Count}] {Path.GetFileName(octreePath)}");

            try
            {
                // Load the octree.
                var (root, fileInfo) = serializer.LoadOctreeFile(octreePath);
                if (root == null)
                {
                    Console.ForegroundColor = ConsoleColor.Yellow;
                    Console.WriteLine("  Skipped: could not load octree root.");
                    Console.ResetColor();
                    skipped++;
                    continue;
                }

                // Check if metadata is already present (check root node).
                if (!force && root.HasMetadata)
                {
                    Console.ForegroundColor = ConsoleColor.DarkGray;
                    Console.WriteLine("  Skipped: metadata already up-to-date (use --force to recompute).");
                    Console.ResetColor();
                    skipped++;
                    continue;
                }

                // Resolve PLY path.
                string plyPath = plyOverride ?? fileInfo.PlyPath;
                if (!File.Exists(plyPath))
                {
                    // Try the PLY next to the .octree file using the stored filename.
                    string adjacent = Path.Combine(
                        Path.GetDirectoryName(octreePath) ?? ".",
                        Path.GetFileName(fileInfo.PlyPath));
                    if (File.Exists(adjacent))
                        plyPath = adjacent;
                    else
                    {
                        Console.ForegroundColor = ConsoleColor.Red;
                        Console.WriteLine($"  Error: PLY file not found: {plyPath}");
                        Console.ResetColor();
                        failed++;
                        continue;
                    }
                }

                Console.WriteLine($"  PLY: {plyPath}");

                // Build PLY header index (header only — bounds come from octree file).
                var plyIndex = new PlyIndex(plyPath);
                plyIndex.BuildIndexHeaderOnly();
                plyIndex.SetBounds(fileInfo.Bounds);

                // Compute metadata.
                Console.Write("  Computing metadata... ");
                var sw = System.Diagnostics.Stopwatch.StartNew();
                int lastPhase = -1, lastPct = -1;
                var lastTime = DateTime.UtcNow;

                var computer = new NodeMetadataComputer();
                computer.OnProgress = (phase, current, total) =>
                {
                    int pct = total > 0 ? (int)(100.0 * current / total) : 0;
                    var now = DateTime.UtcNow;
                    if (pct != lastPct || phase != lastPhase || (now - lastTime).TotalMilliseconds > 250)
                    {
                        lastPct = pct; lastPhase = phase; lastTime = now;
                        string phaseName = phase == 0 ? "Mapping" : "Colors";
                        Console.Write($"\r  Computing metadata... {phaseName}: {pct}%   ");
                    }
                };
                computer.Compute(root, plyIndex);
                plyIndex.Dispose();

                sw.Stop();
                Console.Write("\r                                                          \r");
                Console.Write("  Computing metadata... ");
                Console.ForegroundColor = ConsoleColor.Green;
                Console.Write("Done!");
                Console.ResetColor();
                Console.WriteLine($" ({FormatTime(sw.Elapsed)})");

                // Save updated octree to a temp file, then replace the original.
                string tempPath = octreePath + ".tmp";
                Console.Write("  Saving... ");
                sw.Restart();

                // Rebuild a minimal PlyIndex for serialization (needs VertexCount + Properties).
                // We can reconstruct this from fileInfo.
                var savePlyIndex = new PlyIndex(plyPath);
                savePlyIndex.BuildIndexHeaderOnly();
                savePlyIndex.SetBounds(fileInfo.Bounds);

                serializer.SaveOctreeFile(root, savePlyIndex, fileInfo.PlyPath, tempPath, fileInfo.PointsPerNode);
                savePlyIndex.Dispose();

                File.Move(tempPath, octreePath, overwrite: true);
                sw.Stop();

                Console.ForegroundColor = ConsoleColor.Green;
                Console.Write("Done!");
                Console.ResetColor();
                Console.WriteLine($" ({FormatTime(sw.Elapsed)})");
                processed++;
            }
            catch (Exception ex)
            {
                Console.ForegroundColor = ConsoleColor.Red;
                Console.WriteLine($"  Error: {ex.Message}");
                Console.ResetColor();
                failed++;
            }

            Console.WriteLine();
        }

        totalSw.Stop();
        Console.WriteLine("════════════════════════════════════════════════════════════════");
        Console.WriteLine($"Total time:  {FormatTime(totalSw.Elapsed)}");
        Console.ForegroundColor = ConsoleColor.Green;
        Console.WriteLine($"Updated:     {processed}");
        Console.ResetColor();
        Console.ForegroundColor = ConsoleColor.DarkGray;
        Console.WriteLine($"Skipped:     {skipped}");
        Console.ResetColor();
        if (failed > 0)
        {
            Console.ForegroundColor = ConsoleColor.Red;
            Console.WriteLine($"Failed:      {failed}");
            Console.ResetColor();
        }

        return Task.CompletedTask;
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

        var bufferSizeOption = new Option<int>(
            aliases: new[] { "--buffer-size", "-b" },
            getDefaultValue: () => 512,
            description: "Maximum size per buffer in MB (e.g., 512, 1024, 2048)");

        var traverseCommand = new Command("traverse", "Traverse an octree and demonstrate the API")
        {
            octreeOption,
            plyOption,
            maxDepthOption,
            cacheSizeOption,
            bufferSizeOption
        };

        traverseCommand.SetHandler(async (context) =>
        {
            var octree = context.ParseResult.GetValueForOption(octreeOption)!;
            var ply = context.ParseResult.GetValueForOption(plyOption)!;
            var maxDepth = context.ParseResult.GetValueForOption(maxDepthOption);
            var cacheSize = context.ParseResult.GetValueForOption(cacheSizeOption);
            var bufferSize = context.ParseResult.GetValueForOption(bufferSizeOption);

            await DemoTraversal(octree, ply, maxDepth, cacheSize, bufferSize);
        });

        return traverseCommand;
    }

    static async Task DemoTraversal(
        FileInfo octreeFile,
        FileInfo plyFile,
        int maxDepth,
        int cacheSizeMB,
        int bufferSizeMB)
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
        Console.WriteLine($"Buffer size: {bufferSizeMB} MB per buffer");
        Console.WriteLine();

        try
        {
            Console.Write("Initializing reader... ");
            var sw = System.Diagnostics.Stopwatch.StartNew();
            
            using var reader = new OctreeFlowReader(
                octreeFile.FullName,
                plyFile.FullName,
                cacheSizeMB,
                bufferSizeMB);

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
            
            Console.WriteLine("Buffer configuration:");
            Console.WriteLine($"  Points/node (info):{reader.PointsPerNode:N0} ({reader.PointsPerNodeSource})");
            Console.WriteLine($"  Max buffer size:   {reader.MaxBufferSizeMB} MB per buffer");
            Console.WriteLine($"  Buffer capacity:   {reader.BufferCapacityPoints:N0} points max");
            Console.WriteLine($"  Vector4 buffer:    {FormatFileSize(reader.BufferSizeBytesVector4)}");
            Console.WriteLine($"  Float32 buffer:    {FormatFileSize(reader.BufferSizeBytesFloat32)}");
            Console.WriteLine();

            // Display available features
            Console.WriteLine("Available features (Vector4):");
            foreach (var feature in reader.PointFeaturesVector4)
            {
                Console.WriteLine($"  {feature.Key}");
            }
            Console.WriteLine("Available features (Float32):");
            foreach (var feature in reader.PointFeaturesFloat32)
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
                foreach (var feature in firstSector.VectorFeatures)
                {
                    Console.WriteLine($"    {feature.Key}: {feature.Value.Length} elements");
                }
                Console.WriteLine($"  Float32 Features:");
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

    static Command CreateBatchCommand()
    {
        var folderOption = new Option<DirectoryInfo>(
            aliases: new[] { "--folder", "-f" },
            description: "Folder containing PLY files to process")
        { IsRequired = true };

        var recursiveOption = new Option<bool>(
            aliases: new[] { "--recursive", "-r" },
            getDefaultValue: () => false,
            description: "Search for PLY files recursively in subfolders");

        var pointsPerNodeOption = new Option<int>(
            aliases: new[] { "--points-per-node", "-n" },
            getDefaultValue: () => 1000,
            description: "Number of points per node");

        var minDistanceOption = new Option<float>(
            aliases: new[] { "--min-distance", "-d" },
            getDefaultValue: () => 1.0f,
            description: "Starting minimum distance between points at level 0");

        var ratioOption = new Option<float>(
            aliases: new[] { "--level-ratio" },
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

        var threadsOption = new Option<int>(
            aliases: new[] { "--threads", "-t" },
            getDefaultValue: () => 0,
            description: "Number of threads to use (0 = auto-detect based on CPU cores)");

        var forceOption = new Option<bool>(
            aliases: new[] { "--force" },
            getDefaultValue: () => false,
            description: "Force reprocessing even if octree file exists with latest version");

        var batchCommand = new Command("batch", "Process all PLY files in a folder")
        {
            folderOption,
            recursiveOption,
            pointsPerNodeOption,
            minDistanceOption,
            ratioOption,
            seedOption,
            maxDepthOption,
            verboseOption,
            threadsOption,
            forceOption
        };

        batchCommand.SetHandler(async (context) =>
        {
            var folder = context.ParseResult.GetValueForOption(folderOption)!;
            var recursive = context.ParseResult.GetValueForOption(recursiveOption);
            var pointsPerNode = context.ParseResult.GetValueForOption(pointsPerNodeOption);
            var minDistance = context.ParseResult.GetValueForOption(minDistanceOption);
            var ratio = context.ParseResult.GetValueForOption(ratioOption);
            var seed = context.ParseResult.GetValueForOption(seedOption);
            var maxDepth = context.ParseResult.GetValueForOption(maxDepthOption);
            var verbose = context.ParseResult.GetValueForOption(verboseOption);
            var threads = context.ParseResult.GetValueForOption(threadsOption);
            var force = context.ParseResult.GetValueForOption(forceOption);

            await BatchProcessFolder(folder, recursive, pointsPerNode, minDistance, ratio, seed, maxDepth, verbose, threads, force);
        });

        return batchCommand;
    }

    static async Task BatchProcessFolder(
        DirectoryInfo folder,
        bool recursive,
        int pointsPerNode,
        float minDistance,
        float ratio,
        int? seed,
        int maxDepth,
        bool verbose,
        int threads,
        bool force)
    {
        if (!folder.Exists)
        {
            Console.ForegroundColor = ConsoleColor.Red;
            Console.WriteLine($"Error: Folder not found: {folder.FullName}");
            Console.ResetColor();
            return;
        }

        Console.WriteLine("╔══════════════════════════════════════════════════════════════╗");
        Console.WriteLine("║           OctreeFlow Batch Processor                         ║");
        Console.WriteLine("╚══════════════════════════════════════════════════════════════╝");
        Console.WriteLine();

        Console.WriteLine($"Folder:          {folder.FullName}");
        Console.WriteLine($"Recursive:       {recursive}");
        Console.WriteLine($"Points per node: {pointsPerNode}");
        Console.WriteLine($"Min distance:    {minDistance}");
        Console.WriteLine($"Level ratio:     {ratio}");
        Console.WriteLine($"Max depth:       {(maxDepth == 0 ? "unlimited" : maxDepth)}");
        Console.WriteLine($"Force rebuild:   {force}");
        Console.WriteLine();

        // Find all PLY files
        var searchOption = recursive ? SearchOption.AllDirectories : SearchOption.TopDirectoryOnly;
        var plyFiles = folder.GetFiles("*.ply", searchOption)
            .OrderBy(f => f.FullName)
            .ToList();

        if (plyFiles.Count == 0)
        {
            Console.WriteLine("No PLY files found in the specified folder.");
            return;
        }

        Console.WriteLine($"Found {plyFiles.Count} PLY file(s)");
        Console.WriteLine();

        // Check which files need processing
        var serializer = new StreamingOctreeSerializer();
        var filesToProcess = new List<FileInfo>();
        var skippedFiles = new List<(FileInfo file, string reason)>();

        const int LatestVersion = 6; // Must match StreamingOctreeSerializer.CurrentVersion

        foreach (var plyFile in plyFiles)
        {
            var octreePath = Path.Combine(
                plyFile.DirectoryName ?? ".",
                Path.GetFileNameWithoutExtension(plyFile.Name) + ".octree");

            if (!force && File.Exists(octreePath))
            {
                try
                {
                    var (_, info) = serializer.LoadOctreeFile(octreePath);
                    if (info.Version >= LatestVersion)
                    {
                        skippedFiles.Add((plyFile, $"up-to-date (v{info.Version})"));
                        continue;
                    }
                    else
                    {
                        skippedFiles.Add((plyFile, $"outdated version (v{info.Version}), will rebuild"));
                        filesToProcess.Add(plyFile);
                    }
                }
                catch (Exception ex)
                {
                    // Corrupted or unreadable octree file - rebuild it
                    skippedFiles.Add((plyFile, $"invalid octree file ({ex.Message}), will rebuild"));
                    filesToProcess.Add(plyFile);
                }
            }
            else
            {
                filesToProcess.Add(plyFile);
            }
        }

        // Report skipped files
        if (skippedFiles.Count > 0)
        {
            Console.WriteLine("Skipped files:");
            foreach (var (file, reason) in skippedFiles)
            {
                if (reason.Contains("up-to-date"))
                {
                    Console.ForegroundColor = ConsoleColor.DarkGray;
                    Console.WriteLine($"  [SKIP] {file.Name} - {reason}");
                    Console.ResetColor();
                }
                else
                {
                    Console.ForegroundColor = ConsoleColor.Yellow;
                    Console.WriteLine($"  [REBUILD] {file.Name} - {reason}");
                    Console.ResetColor();
                }
            }
            Console.WriteLine();
        }

        if (filesToProcess.Count == 0)
        {
            Console.ForegroundColor = ConsoleColor.Green;
            Console.WriteLine("All files are up-to-date. Nothing to process.");
            Console.ResetColor();
            return;
        }

        Console.WriteLine($"Processing {filesToProcess.Count} file(s)...");
        Console.WriteLine();

        var totalStopwatch = System.Diagnostics.Stopwatch.StartNew();
        int processed = 0;
        int failed = 0;

        foreach (var plyFile in filesToProcess)
        {
            Console.WriteLine("────────────────────────────────────────────────────────────────");
            Console.WriteLine($"[{processed + failed + 1}/{filesToProcess.Count}] {plyFile.Name}");
            Console.WriteLine("────────────────────────────────────────────────────────────────");

            try
            {
                await BuildOctree(plyFile, null, pointsPerNode, minDistance, ratio, seed, maxDepth, verbose, threads);
                processed++;
            }
            catch (Exception ex)
            {
                Console.ForegroundColor = ConsoleColor.Red;
                Console.WriteLine($"Error processing {plyFile.Name}: {ex.Message}");
                Console.ResetColor();
                failed++;
            }

            Console.WriteLine();
        }

        totalStopwatch.Stop();

        // Summary
        Console.WriteLine("════════════════════════════════════════════════════════════════");
        Console.WriteLine("                        BATCH COMPLETE");
        Console.WriteLine("════════════════════════════════════════════════════════════════");
        Console.WriteLine();
        Console.WriteLine($"Total time:     {FormatTime(totalStopwatch.Elapsed)}");
        Console.WriteLine($"Files found:    {plyFiles.Count}");
        Console.ForegroundColor = ConsoleColor.DarkGray;
        Console.WriteLine($"Skipped:        {skippedFiles.Count(s => s.reason.Contains("up-to-date"))} (already up-to-date)");
        Console.ResetColor();
        Console.ForegroundColor = ConsoleColor.Green;
        Console.WriteLine($"Processed:      {processed}");
        Console.ResetColor();
        if (failed > 0)
        {
            Console.ForegroundColor = ConsoleColor.Red;
            Console.WriteLine($"Failed:         {failed}");
            Console.ResetColor();
        }
        Console.WriteLine();
    }
}
