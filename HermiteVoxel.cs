using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct HermiteVoxel
{
    // Edge intersection positions along +X, +Y, +Z edges (0..255 mapped to 0.0f..1.0f)
    public byte EdgeX;
    public byte EdgeY;
    public byte EdgeZ;

    // Bitmask indicating which of the 3 edges actually cross the isosurface (bits 0, 1, 2)
    public byte EdgeMask;

    // Quantized surface normal (-127..+127 mapped to -1.0f..+1.0f)
    public sbyte NormalX;
    public sbyte NormalY;
    public sbyte NormalZ;

    // Padding/flags byte to keep 8-byte alignment
    public byte Flags;

    // Vertex/Surface RGB color
    public byte R;
    public byte G;
    public byte B;

    // 4 material weights packed into 2 bytes (4 bits each: 0..15)
    private ushort _packedWeights;

    // 3 bytes reserved/alignment to hit exact 16-byte SIMD/cache-line friendly size
    private byte _reserved0;
    private byte _reserved1;
    private byte _reserved2;

    public readonly byte Weight0
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => (byte)(_packedWeights & 0xF);
    }

    public readonly byte Weight1
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => (byte)((_packedWeights >> 4) & 0xF);
    }

    public readonly byte Weight2
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => (byte)((_packedWeights >> 8) & 0xF);
    }

    public readonly byte Weight3
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => (byte)((_packedWeights >> 12) & 0xF);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SetPackedWeights(byte w0, byte w1, byte w2, byte w3)
    {
        _packedWeights = (ushort)((w0 & 0xF)
            | ((w1 & 0xF) << 4)
            | ((w2 & 0xF) << 8)
            | ((w3 & 0xF) << 12));
    }
}
