using Stride.Core.Mathematics;
using System.Text;
using System.Globalization;

namespace OctreeFlow.IO;

/// <summary>
/// PLY file format type.
/// </summary>
public enum PlyFormat
{
    Ascii,
    BinaryLittleEndian,
    BinaryBigEndian
}

/// <summary>
/// Property definition from PLY header.
/// </summary>
public class PlyProperty
{
    public string Name { get; set; } = "";
    public PlyDataType Type { get; set; }
    public int ByteOffset { get; set; }
    public int ByteSize { get; set; }
}

/// <summary>
/// PLY data types.
/// </summary>
public enum PlyDataType
{
    Int8, UInt8, Int16, UInt16, Int32, UInt32, Float32, Float64
}

/// <summary>
/// Lightweight index into a PLY file for streaming access.
/// Does NOT load point data into memory - only builds an index.
/// </summary>
public class PlyIndex : IDisposable
{
    private readonly string _filePath;
    private FileStream? _stream;
    private BinaryReader? _reader;

    public PlyFormat Format { get; private set; }
    public int VertexCount { get; private set; }
    public long DataStartOffset { get; private set; }
    public int BytesPerVertex { get; private set; }
    public List<PlyProperty> Properties { get; } = new();
    public BoundingBox Bounds { get; private set; }

    // Property indices for fast lookup
    private int _xIndex = -1, _yIndex = -1, _zIndex = -1;
    private int _rIndex = -1, _gIndex = -1, _bIndex = -1, _aIndex = -1;
    private int _nxIndex = -1, _nyIndex = -1, _nzIndex = -1;
    private int _intensityIndex = -1;

    public PlyIndex(string filePath)
    {
        _filePath = filePath;
    }

    /// <summary>
    /// Scans the PLY file to build the index without loading point data.
    /// Also computes bounds by streaming through all points once.
    /// </summary>
    public void BuildIndex(Action<int, int>? onProgress = null)
    {
        _stream = File.OpenRead(_filePath);
        _reader = new BinaryReader(_stream, Encoding.ASCII, leaveOpen: true);

        // Parse header
        ParseHeader();

        // Build property index map
        BuildPropertyMap();

        // Compute bounds by streaming through points
        ComputeBounds(onProgress);
    }

    /// <summary>
    /// Builds only the header index without computing bounds.
    /// Useful when bounds are already known (e.g., from an octree file).
    /// </summary>
    public void BuildIndexHeaderOnly()
    {
        _stream = File.OpenRead(_filePath);
        _reader = new BinaryReader(_stream, Encoding.ASCII, leaveOpen: true);

        // Parse header
        ParseHeader();

        // Build property index map
        BuildPropertyMap();

        // Don't compute bounds - leave at default
        // Bounds can be set externally if needed
    }

    /// <summary>
    /// Sets the bounds externally (used when bounds are loaded from octree file).
    /// </summary>
    public void SetBounds(BoundingBox bounds)
    {
        Bounds = bounds;
    }

    /// <summary>
    /// Single-pass: builds index, computes bounds, AND writes positions cache file.
    /// Much more efficient than separate operations.
    /// </summary>
    public void BuildIndexWithPositionsCache(string positionsCachePath, Action<int, int>? onProgress = null)
    {
        _stream = File.OpenRead(_filePath);
        _reader = new BinaryReader(_stream, Encoding.ASCII, leaveOpen: true);

        // Parse header
        ParseHeader();

        // Build property index map
        BuildPropertyMap();

        // Single pass: compute bounds AND write positions cache
        ComputeBoundsAndWritePositions(positionsCachePath, onProgress);
    }

    private void ComputeBoundsAndWritePositions(string positionsCachePath, Action<int, int>? onProgress)
    {
        if (_stream == null || _reader == null) return;

        _stream.Position = DataStartOffset;

        Vector3 min = new Vector3(float.MaxValue);
        Vector3 max = new Vector3(float.MinValue);

        int reportInterval = Math.Max(1, VertexCount / 100);

        using var posWriter = new BinaryWriter(File.Create(positionsCachePath));

        if (Format == PlyFormat.Ascii)
        {
            using var streamReader = new StreamReader(_stream, Encoding.ASCII, leaveOpen: true);
            for (int i = 0; i < VertexCount; i++)
            {
                var line = streamReader.ReadLine();
                if (line == null) break;

                var values = line.Split(' ', StringSplitOptions.RemoveEmptyEntries);
                var pos = ParsePositionFromAscii(values);

                min = Vector3.Min(min, pos);
                max = Vector3.Max(max, pos);

                // Write to positions cache
                posWriter.Write(pos.X);
                posWriter.Write(pos.Y);
                posWriter.Write(pos.Z);

                if (i % reportInterval == 0)
                    onProgress?.Invoke(i, VertexCount);
            }
        }
        else
        {
            bool bigEndian = Format == PlyFormat.BinaryBigEndian;

            for (int i = 0; i < VertexCount; i++)
            {
                // Read all values for this vertex
                var values = ReadVertexBinaryFull(bigEndian);
                
                var pos = new Vector3(
                    _xIndex >= 0 && _xIndex < values.Length ? values[_xIndex] : 0,
                    _yIndex >= 0 && _yIndex < values.Length ? values[_yIndex] : 0,
                    _zIndex >= 0 && _zIndex < values.Length ? values[_zIndex] : 0
                );

                min = Vector3.Min(min, pos);
                max = Vector3.Max(max, pos);

                // Write to positions cache
                posWriter.Write(pos.X);
                posWriter.Write(pos.Y);
                posWriter.Write(pos.Z);

                if (i % reportInterval == 0)
                    onProgress?.Invoke(i, VertexCount);
            }
        }

        Bounds = new BoundingBox(min, max);
        onProgress?.Invoke(VertexCount, VertexCount);
    }

    private float[] ReadVertexBinaryFull(bool bigEndian)
    {
        if (_reader == null) return Array.Empty<float>();

        var values = new float[Properties.Count];
        for (int i = 0; i < Properties.Count; i++)
        {
            values[i] = ReadBinaryValue(_reader, Properties[i].Type, bigEndian);
        }
        return values;
    }

    private void ParseHeader()
    {
        if (_stream == null) throw new InvalidOperationException("Stream not initialized");

        var headerBytes = new List<byte>();
        bool foundEndHeader = false;

        // Read header line by line
        while (!foundEndHeader)
        {
            var line = ReadLine();
            if (line == null) throw new FormatException("Unexpected end of file in header");

            var trimmed = line.Trim();

            if (trimmed.Equals("end_header", StringComparison.OrdinalIgnoreCase))
            {
                foundEndHeader = true;
                DataStartOffset = _stream.Position;
            }
            else if (trimmed.StartsWith("format", StringComparison.OrdinalIgnoreCase))
            {
                var parts = trimmed.Split(' ', StringSplitOptions.RemoveEmptyEntries);
                if (parts.Length >= 2)
                {
                    Format = parts[1].ToLower() switch
                    {
                        "ascii" => PlyFormat.Ascii,
                        "binary_little_endian" => PlyFormat.BinaryLittleEndian,
                        "binary_big_endian" => PlyFormat.BinaryBigEndian,
                        _ => throw new FormatException($"Unknown PLY format: {parts[1]}")
                    };
                }
            }
            else if (trimmed.StartsWith("element vertex", StringComparison.OrdinalIgnoreCase))
            {
                var parts = trimmed.Split(' ', StringSplitOptions.RemoveEmptyEntries);
                if (parts.Length >= 3)
                {
                    VertexCount = int.Parse(parts[2], CultureInfo.InvariantCulture);
                }
            }
            else if (trimmed.StartsWith("property", StringComparison.OrdinalIgnoreCase) && !trimmed.Contains("list"))
            {
                var parts = trimmed.Split(' ', StringSplitOptions.RemoveEmptyEntries);
                if (parts.Length >= 3)
                {
                    var prop = new PlyProperty
                    {
                        Type = ParseDataType(parts[1]),
                        Name = parts[2].ToLower(),
                        ByteOffset = BytesPerVertex
                    };
                    prop.ByteSize = GetTypeSize(prop.Type);
                    BytesPerVertex += prop.ByteSize;
                    Properties.Add(prop);
                }
            }
        }

        if (Format == PlyFormat.Ascii)
        {
            // For ASCII, we need to handle variable line lengths
            // This is inherently slower but we'll optimize where we can
            BytesPerVertex = 0; // Not fixed for ASCII
        }
    }

    private string? ReadLine()
    {
        if (_stream == null) return null;

        var sb = new StringBuilder();
        int b;
        while ((b = _stream.ReadByte()) != -1)
        {
            if (b == '\n') break;
            if (b != '\r') sb.Append((char)b);
        }
        return b == -1 && sb.Length == 0 ? null : sb.ToString();
    }

    private void BuildPropertyMap()
    {
        for (int i = 0; i < Properties.Count; i++)
        {
            var name = Properties[i].Name;
            switch (name)
            {
                case "x": _xIndex = i; break;
                case "y": _yIndex = i; break;
                case "z": _zIndex = i; break;
                case "red" or "r": _rIndex = i; break;
                case "green" or "g": _gIndex = i; break;
                case "blue" or "b": _bIndex = i; break;
                case "alpha" or "a": _aIndex = i; break;
                case "nx": _nxIndex = i; break;
                case "ny": _nyIndex = i; break;
                case "nz": _nzIndex = i; break;
                case "intensity" or "scalar_intensity": _intensityIndex = i; break;
            }
        }
    }

    private void ComputeBounds(Action<int, int>? onProgress)
    {
        if (_stream == null || _reader == null) return;

        _stream.Position = DataStartOffset;

        Vector3 min = new Vector3(float.MaxValue);
        Vector3 max = new Vector3(float.MinValue);

        int reportInterval = Math.Max(1, VertexCount / 100);

        if (Format == PlyFormat.Ascii)
        {
            using var streamReader = new StreamReader(_stream, Encoding.ASCII, leaveOpen: true);
            for (int i = 0; i < VertexCount; i++)
            {
                var line = streamReader.ReadLine();
                if (line == null) break;

                var values = line.Split(' ', StringSplitOptions.RemoveEmptyEntries);
                var pos = ParsePositionFromAscii(values);

                min = Vector3.Min(min, pos);
                max = Vector3.Max(max, pos);

                if (i % reportInterval == 0)
                    onProgress?.Invoke(i, VertexCount);
            }
        }
        else
        {
            bool bigEndian = Format == PlyFormat.BinaryBigEndian;

            for (int i = 0; i < VertexCount; i++)
            {
                var pos = ReadPositionBinary(bigEndian);

                min = Vector3.Min(min, pos);
                max = Vector3.Max(max, pos);

                // Skip remaining properties
                int posBytes = (_xIndex >= 0 ? Properties[_xIndex].ByteSize : 0) +
                               (_yIndex >= 0 ? Properties[_yIndex].ByteSize : 0) +
                               (_zIndex >= 0 ? Properties[_zIndex].ByteSize : 0);
                _stream.Position += BytesPerVertex - posBytes;

                if (i % reportInterval == 0)
                    onProgress?.Invoke(i, VertexCount);
            }
        }

        Bounds = new BoundingBox(min, max);
        onProgress?.Invoke(VertexCount, VertexCount);
    }

    /// <summary>
    /// Gets the byte offset for a specific vertex index (binary format only).
    /// </summary>
    public long GetVertexOffset(int index)
    {
        if (Format == PlyFormat.Ascii)
            throw new NotSupportedException("Random access not efficient for ASCII PLY files");

        return DataStartOffset + (long)index * BytesPerVertex;
    }

    /// <summary>
    /// Reads position for a vertex at the given index.
    /// </summary>
    public Vector3 ReadPosition(int index)
    {
        if (_stream == null || _reader == null)
            throw new InvalidOperationException("Index not built");

        _stream.Position = GetVertexOffset(index);
        return ReadPositionBinary(Format == PlyFormat.BinaryBigEndian);
    }

    /// <summary>
    /// Reads full point data for a vertex at the given index.
    /// Returns values array in property order.
    /// </summary>
    public float[] ReadVertex(int index)
    {
        if (_stream == null || _reader == null)
            throw new InvalidOperationException("Index not built");

        _stream.Position = GetVertexOffset(index);
        return ReadVertexBinary(Format == PlyFormat.BinaryBigEndian);
    }

    /// <summary>
    /// Streams through all vertices, calling the callback for each.
    /// Much more efficient than random access.
    /// </summary>
    public void StreamVertices(Action<int, Vector3, float[]> onVertex, Action<int, int>? onProgress = null)
    {
        if (_stream == null)
            throw new InvalidOperationException("Index not built");

        // Reset stream position and recreate BinaryReader to clear its internal buffer
        // BinaryReader buffers data, so changing stream position directly can cause issues
        _stream.Position = DataStartOffset;
        
        // Recreate BinaryReader to reset its internal buffer state
        _reader?.Dispose();
        _reader = new BinaryReader(_stream, Encoding.ASCII, leaveOpen: true);
        
        int reportInterval = Math.Max(1, VertexCount / 100);

        if (Format == PlyFormat.Ascii)
        {
            StreamVerticesAscii(onVertex, onProgress, reportInterval);
        }
        else
        {
            StreamVerticesBinary(onVertex, onProgress, reportInterval);
        }
    }

    private void StreamVerticesAscii(Action<int, Vector3, float[]> onVertex, Action<int, int>? onProgress, int reportInterval)
    {
        if (_stream == null) return;

        _stream.Position = DataStartOffset;
        using var reader = new StreamReader(_stream, Encoding.ASCII, leaveOpen: true);

        for (int i = 0; i < VertexCount; i++)
        {
            var line = reader.ReadLine();
            if (line == null) break;

            var parts = line.Split(' ', StringSplitOptions.RemoveEmptyEntries);
            var values = new float[parts.Length];

            for (int j = 0; j < parts.Length && j < Properties.Count; j++)
            {
                float.TryParse(parts[j], NumberStyles.Float, CultureInfo.InvariantCulture, out values[j]);
            }

            var pos = ParsePositionFromAscii(parts);
            onVertex(i, pos, values);

            if (i % reportInterval == 0)
                onProgress?.Invoke(i, VertexCount);
        }

        onProgress?.Invoke(VertexCount, VertexCount);
    }

    private void StreamVerticesBinary(Action<int, Vector3, float[]> onVertex, Action<int, int>? onProgress, int reportInterval)
    {
        if (_stream == null || _reader == null) return;

        // Reset to start of data
        _stream.Position = DataStartOffset;
        
        bool bigEndian = Format == PlyFormat.BinaryBigEndian;

        // Verify we have enough data in the file
        long expectedDataSize = (long)VertexCount * BytesPerVertex;
        long availableDataSize = _stream.Length - DataStartOffset;
        
        if (availableDataSize < expectedDataSize)
        {
            throw new InvalidOperationException(
                $"PLY file appears truncated: expected {expectedDataSize} bytes of vertex data " +
                $"(starting at offset {DataStartOffset}), but only {availableDataSize} bytes available. " +
                $"File size: {_stream.Length} bytes, VertexCount: {VertexCount}, BytesPerVertex: {BytesPerVertex}");
        }

        long startPosition = _stream.Position;
        
        for (int i = 0; i < VertexCount; i++)
        {
            // Check if we have enough bytes remaining
            long bytesRemaining = _stream.Length - _stream.Position;
            if (bytesRemaining < BytesPerVertex)
            {
                long bytesRead = _stream.Position - startPosition;
                long expectedBytes = (long)VertexCount * BytesPerVertex;
                throw new InvalidOperationException(
                    $"Unexpected end of file while reading vertex {i} of {VertexCount}. " +
                    $"Position: {_stream.Position}, File size: {_stream.Length}, " +
                    $"Bytes remaining: {bytesRemaining}, BytesPerVertex: {BytesPerVertex}. " +
                    $"Bytes read so far: {bytesRead}, Expected total: {expectedBytes}");
            }

            try
            {
                var values = ReadVertexBinary(bigEndian);
                var pos = new Vector3(
                    _xIndex >= 0 ? values[_xIndex] : 0,
                    _yIndex >= 0 ? values[_yIndex] : 0,
                    _zIndex >= 0 ? values[_zIndex] : 0
                );

                onVertex(i, pos, values);

                if (i % reportInterval == 0)
                    onProgress?.Invoke(i, VertexCount);
            }
            catch (IOException ex)
            {
                // Catch ALL IOExceptions
                long bytesRead = _stream.Position - startPosition;
                long expectedBytes = (long)VertexCount * BytesPerVertex;
                throw new InvalidOperationException(
                    $"Error reading vertex {i} of {VertexCount} at stream position {_stream.Position}. " +
                    $"File length: {_stream.Length}, BytesPerVertex: {BytesPerVertex}, " +
                    $"Properties.Count: {Properties.Count}. " +
                    $"Bytes read so far: {bytesRead}, Expected total: {expectedBytes}. " +
                    $"IOException message: {ex.Message}. " +
                    $"This may indicate the PLY file is truncated or the header information is incorrect.", ex);
            }
            catch (Exception ex)
            {
                // Catch any other exceptions too
                long bytesRead = _stream.Position - startPosition;
                long expectedBytes = (long)VertexCount * BytesPerVertex;
                throw new InvalidOperationException(
                    $"Unexpected error reading vertex {i} of {VertexCount} at stream position {_stream.Position}. " +
                    $"File length: {_stream.Length}, BytesPerVertex: {BytesPerVertex}, " +
                    $"Properties.Count: {Properties.Count}. " +
                    $"Bytes read so far: {bytesRead}, Expected total: {expectedBytes}. " +
                    $"Exception type: {ex.GetType().Name}, Message: {ex.Message}.", ex);
            }
        }

        // Verify we read exactly the expected number of bytes
        long totalBytesRead = _stream.Position - startPosition;
        long expectedTotalBytes = (long)VertexCount * BytesPerVertex;
        long bytesRemainingAfterRead = _stream.Length - _stream.Position;
        
        if (totalBytesRead != expectedTotalBytes)
        {
            // This is a warning, not an error, as the file might have extra data
            // But log it for debugging
            System.Diagnostics.Debug.WriteLine(
                $"Warning: Read {totalBytesRead} bytes but expected {expectedTotalBytes} bytes " +
                $"({totalBytesRead - expectedTotalBytes} bytes difference). " +
                $"Bytes remaining in file: {bytesRemainingAfterRead}");
        }

        try
        {
            onProgress?.Invoke(VertexCount, VertexCount);
        }
        catch (Exception ex)
        {
            throw new InvalidOperationException(
                $"Error in final progress callback after reading all vertices. " +
                $"Total bytes read: {totalBytesRead}, Expected: {expectedTotalBytes}, " +
                $"Bytes remaining: {bytesRemainingAfterRead}. Error: {ex.Message}", ex);
        }
    }

    private Vector3 ParsePositionFromAscii(string[] values)
    {
        float x = 0, y = 0, z = 0;
        if (_xIndex >= 0 && _xIndex < values.Length)
            float.TryParse(values[_xIndex], NumberStyles.Float, CultureInfo.InvariantCulture, out x);
        if (_yIndex >= 0 && _yIndex < values.Length)
            float.TryParse(values[_yIndex], NumberStyles.Float, CultureInfo.InvariantCulture, out y);
        if (_zIndex >= 0 && _zIndex < values.Length)
            float.TryParse(values[_zIndex], NumberStyles.Float, CultureInfo.InvariantCulture, out z);
        return new Vector3(x, y, z);
    }

    private Vector3 ReadPositionBinary(bool bigEndian)
    {
        if (_reader == null) return Vector3.Zero;

        float x = 0, y = 0, z = 0;

        for (int i = 0; i < Properties.Count; i++)
        {
            float val = ReadBinaryValue(_reader, Properties[i].Type, bigEndian);
            if (i == _xIndex) x = val;
            else if (i == _yIndex) y = val;
            else if (i == _zIndex) z = val;

            // Stop early if we have all position components
            if (i >= Math.Max(_xIndex, Math.Max(_yIndex, _zIndex)))
            {
                // Skip remaining bytes for this vertex
                int remaining = BytesPerVertex - Properties.Take(i + 1).Sum(p => p.ByteSize);
                if (remaining > 0 && _stream != null)
                    _stream.Position += remaining;
                break;
            }
        }

        return new Vector3(x, y, z);
    }

    private float[] ReadVertexBinary(bool bigEndian)
    {
        if (_reader == null || _stream == null) return Array.Empty<float>();

        var values = new float[Properties.Count];
        for (int i = 0; i < Properties.Count; i++)
        {
            long positionBeforeRead = _stream.Position;
            try
            {
                values[i] = ReadBinaryValue(_reader, Properties[i].Type, bigEndian);
            }
            catch (EndOfStreamException ex)
            {
                throw new InvalidOperationException(
                    $"Failed to read property '{Properties[i].Name}' (index {i}) of vertex at stream position {positionBeforeRead}. " +
                    $"Current position: {_stream.Position}, File length: {_stream.Length}. " +
                    $"Expected BytesPerVertex: {BytesPerVertex}, Properties.Count: {Properties.Count}. " +
                    $"Property type: {Properties[i].Type}, ByteSize: {Properties[i].ByteSize}. " +
                    $"This may indicate the PLY file is truncated or the header information is incorrect.", ex);
            }
            catch (IOException ex)
            {
                // Catch ALL IOExceptions (EndOfStreamException is already caught above)
                throw new InvalidOperationException(
                    $"Failed to read property '{Properties[i].Name}' (index {i}) of vertex at stream position {positionBeforeRead}. " +
                    $"Current position: {_stream.Position}, File length: {_stream.Length}. " +
                    $"Expected BytesPerVertex: {BytesPerVertex}, Properties.Count: {Properties.Count}. " +
                    $"Property type: {Properties[i].Type}, ByteSize: {Properties[i].ByteSize}. " +
                    $"IOException message: {ex.Message}. " +
                    $"This may indicate the PLY file is truncated or the header information is incorrect.", ex);
            }
        }
        return values;
    }

    private static float ReadBinaryValue(BinaryReader reader, PlyDataType type, bool bigEndian)
    {
        switch (type)
        {
            case PlyDataType.Int8:
                return reader.ReadSByte();
            case PlyDataType.UInt8:
                return reader.ReadByte();
            case PlyDataType.Int16:
                var s = reader.ReadInt16();
                return bigEndian ? ReverseEndianness(s) : s;
            case PlyDataType.UInt16:
                var us = reader.ReadUInt16();
                return bigEndian ? ReverseEndianness(us) : us;
            case PlyDataType.Int32:
                var i = reader.ReadInt32();
                return bigEndian ? ReverseEndianness(i) : i;
            case PlyDataType.UInt32:
                var ui = reader.ReadUInt32();
                return bigEndian ? ReverseEndianness(ui) : ui;
            case PlyDataType.Float32:
                if (bigEndian)
                {
                    var bytes = reader.ReadBytes(4);
                    Array.Reverse(bytes);
                    return BitConverter.ToSingle(bytes);
                }
                return reader.ReadSingle();
            case PlyDataType.Float64:
                if (bigEndian)
                {
                    var bytes = reader.ReadBytes(8);
                    Array.Reverse(bytes);
                    return (float)BitConverter.ToDouble(bytes);
                }
                return (float)reader.ReadDouble();
            default:
                return reader.ReadSingle();
        }
    }

    private static PlyDataType ParseDataType(string typeName)
    {
        return typeName.ToLower() switch
        {
            "char" or "int8" => PlyDataType.Int8,
            "uchar" or "uint8" => PlyDataType.UInt8,
            "short" or "int16" => PlyDataType.Int16,
            "ushort" or "uint16" => PlyDataType.UInt16,
            "int" or "int32" => PlyDataType.Int32,
            "uint" or "uint32" => PlyDataType.UInt32,
            "float" or "float32" => PlyDataType.Float32,
            "double" or "float64" => PlyDataType.Float64,
            _ => PlyDataType.Float32
        };
    }

    private static int GetTypeSize(PlyDataType type)
    {
        return type switch
        {
            PlyDataType.Int8 or PlyDataType.UInt8 => 1,
            PlyDataType.Int16 or PlyDataType.UInt16 => 2,
            PlyDataType.Int32 or PlyDataType.UInt32 or PlyDataType.Float32 => 4,
            PlyDataType.Float64 => 8,
            _ => 4
        };
    }

    private static short ReverseEndianness(short value) =>
        (short)((value & 0xFF) << 8 | (value >> 8) & 0xFF);

    private static ushort ReverseEndianness(ushort value) =>
        (ushort)((value & 0xFF) << 8 | (value >> 8) & 0xFF);

    private static int ReverseEndianness(int value) =>
        (int)(((uint)value & 0xFF) << 24 |
              ((uint)value & 0xFF00) << 8 |
              ((uint)value & 0xFF0000) >> 8 |
              ((uint)value >> 24) & 0xFF);

    private static uint ReverseEndianness(uint value) =>
        (value & 0xFF) << 24 |
        (value & 0xFF00) << 8 |
        (value & 0xFF0000) >> 8 |
        (value >> 24) & 0xFF;

    public void Dispose()
    {
        _reader?.Dispose();
        _stream?.Dispose();
        _reader = null;
        _stream = null;
    }
}

