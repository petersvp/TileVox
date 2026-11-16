using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace TileVox
{

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct TerrainVoxel : IVoxelDensityProvider, IVoxelRgbProvider
    {
        public sbyte Density; // -128 (solid) to +127 (air), 0 = surface
        public byte R;
        public byte G;
        public byte B;

        // 4 material weights (0..255 each) for texture splatting/blending
        public byte Weight0;
        public byte Weight1;
        public byte Weight2;
        public byte Weight3;

        sbyte IVoxelDensityProvider.Density { get => Density; set => Density = value; }
        byte IVoxelRgbProvider.R => R;
        byte IVoxelRgbProvider.G => G;
        byte IVoxelRgbProvider.B => B;

        public readonly bool IsSolid
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => Density < 0;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SetWeights(byte w0, byte w1, byte w2, byte w3)
        {
            Weight0 = w0;
            Weight1 = w1;
            Weight2 = w2;
            Weight3 = w3;
        }
    }
}
