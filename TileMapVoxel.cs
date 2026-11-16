using System;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace TileVox
{
    // Legacy VolumeBuilder user-facing feature
    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct TileVoxel : IVoxelTileMapIdProvider, IVoxelMaterialIdProvider, IVoxelColorProvider
    {
        public ushort TileId;
        public byte MaterialId;
        public UInt32 color; // Packed 0xRRGGBBAA.

        ushort IVoxelTileMapIdProvider.TileId { get => TileId; set => TileId = value; }
        uint IVoxelMaterialIdProvider.MaterialId { get => MaterialId; set => MaterialId = checked((byte)value); }
        byte IVoxelColorProvider.R { get => (byte)(color >> 24); set => color = (color & 0x00FFFFFF) | ((uint)value << 24); }
        byte IVoxelColorProvider.G { get => (byte)(color >> 16); set => color = (color & 0xFF00FFFF) | ((uint)value << 16); }
        byte IVoxelColorProvider.B { get => (byte)(color >> 8); set => color = (color & 0xFFFF00FF) | ((uint)value << 8); }
        byte IVoxelColorProvider.A { get => (byte)color; set => color = (color & 0xFFFFFF00) | value; }
    }
}
