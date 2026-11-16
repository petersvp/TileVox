using System;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace TileVox
{
    [Flags]
    public enum NeighborFlags : byte
    {
        None = 0,
        PosX = 1 << 0, // Right
        NegX = 1 << 1, // Left
        PosY = 1 << 2, // Up
        NegY = 1 << 3, // Down
        PosZ = 1 << 4, // Forward
        NegZ = 1 << 5, // Back
        All = 0x3F
    }

    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct TileVoxelEx : IVoxelTileMapIdProvider, IVoxelMaterialIdProvider, IVoxelTile2AdjacencyProvider
    {
        // 2 Bytes: Structural tile/prefab ID (0 = empty/air)
        public ushort TileId;

        // 1 Byte: Primary material ID of this voxel (0..255)
        public byte MaterialId;

        // 1 Byte: 6-bit adjacency flags (bits 0..5 = faces, bits 6..7 = rotation/orientation 0..3)
        public byte AdjacencyAndRotation;

        // 4 Bytes: 6 neighbor material IDs packed at 5 bits each (0..31 material categories)
        // Layout: [NegZ: 5b][PosZ: 5b][NegY: 5b][PosY: 5b][NegX: 5b][PosX: 5b] (30 bits used)
        private uint _packedNeighborMaterials;

        ushort IVoxelTileMapIdProvider.TileId { get => TileId; set => TileId = value; }
        uint IVoxelMaterialIdProvider.MaterialId { get => MaterialId; set => MaterialId = checked((byte)value); }

        public readonly NeighborFlags Adjacency
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => (NeighborFlags)(AdjacencyAndRotation & 0x3F);
        }

        public readonly byte Rotation
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => (byte)((AdjacencyAndRotation >> 6) & 0x03);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SetAdjacencyAndRotation(NeighborFlags flags, byte rotation)
        {
            AdjacencyAndRotation = (byte)(((byte)flags & 0x3F) | ((rotation & 0x03) << 6));
        }

        // Direction index: 0 = +X, 1 = -X, 2 = +Y, 3 = -Y, 4 = +Z, 5 = -Z
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public readonly byte GetNeighborMaterial(int directionIndex)
        {
            return (byte)((_packedNeighborMaterials >> (directionIndex * 5)) & 0x1Fu);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SetNeighborMaterial(int directionIndex, byte neighborMatId)
        {
            int shift = directionIndex * 5;
            _packedNeighborMaterials = (_packedNeighborMaterials & ~(0x1Fu << shift))
                | ((uint)(neighborMatId & 0x1Fu) << shift);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SetAllNeighborMaterials(byte posX, byte negX, byte posY, byte negY, byte posZ, byte negZ)
        {
            _packedNeighborMaterials = (uint)(
                (posX & 0x1F)
                | ((negX & 0x1F) << 5)
                | ((posY & 0x1F) << 10)
                | ((negY & 0x1F) << 15)
                | ((posZ & 0x1F) << 20)
                | ((negZ & 0x1F) << 25));
        }
    }
}
