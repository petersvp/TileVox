#if UNITY_5_3_OR_NEWER
using g4;
using UnityEngine;
#endif

namespace TileVox
{
    // used by all meshers
    public interface IVoxelColorProvider 
    {
        byte R { get; set; }
        byte G { get; set; }
        byte B { get; set; }
        byte A { get; set; }
    }

    /// <summary>RGB voxel color without an alpha channel.</summary>
    public interface IVoxelRgbProvider
    {
        byte R { get; }
        byte G { get; }
        byte B { get; }
    }
#if UNITY_5_3_OR_NEWER
    public interface IUnityColorProvider
    {
        Color UColor { get; set; }
    }
#endif

    public interface IVoxelMaterialIdProvider
    {
        uint MaterialId { get; set; }
    }

    // used by the Tile Map Mesher
    public interface IVoxelTileMapIdProvider
    {
        ushort TileId { get; set; }
    }

    public interface IVoxelDensityProvider
    {
        sbyte Density { get; set; }
    }

    public interface IVoxelWeightProvider
    {
        byte Weight0 { get; set; }
        byte Weight1 { get; set; }
        byte Weight2 { get; set; }
        byte Weight3 { get; set; }
    }

    /// <summary>Optional tile orientation and cached neighbor data for tile map meshing.</summary>
    public interface IVoxelTile2AdjacencyProvider
    {
        /// <summary>Faces marked as adjacent to another tile.</summary>
        NeighborFlags Adjacency { get; }

        /// <summary>Horizontal quarter-turns of the tile.</summary>
        byte Rotation { get; }

        /// <summary>Gets the material category of a neighbor in direction order +X, -X, +Y, -Y, +Z, -Z.</summary>
        byte GetNeighborMaterial(int directionIndex);
    }

    /// <summary>Optional Hermite edge crossings for dual contouring alongside a density source.</summary>
    public interface IVoxelHermiteProvider
    {
        /// <summary>Gets the positive X, Y, or Z edge crossing and its quantized normal.</summary>
        /// <param name="directionIndex">0 for +X, 1 for +Y, or 2 for +Z.</param>
        /// <param name="position">Crossing fraction from 0 to 255 along the edge.</param>
        /// <param name="normalX">Quantized X component of the crossing normal.</param>
        /// <param name="normalY">Quantized Y component of the crossing normal.</param>
        /// <param name="normalZ">Quantized Z component of the crossing normal.</param>
        /// <returns>True if this edge crosses the surface; false otherwise.</returns>
        bool TryGetEdge(int directionIndex, out byte position, out sbyte normalX, out sbyte normalY, out sbyte normalZ);
    }

    /// <summary>Builds a self-contained 3D mesh result from voxel data.</summary>
    public interface IVoxelDataMesher3D
    {
        /// <summary>Generates geometry and numeric material assignments.</summary>
        MesherResult3D BuildMesh();
    }
}
