using g4;
using System.Collections.Generic;

namespace TileVox.Meshing
{
    /// <summary>How a voxel attribute partitions the terrain surface.</summary>
    public enum TerrainBoundaryMode
    {
        /// <summary>Keep the original surface topology and vertex attributes.</summary>
        None,
        /// <summary>Split where the nearest surface-vertex label changes.</summary>
        Fast,
        /// <summary>Split where interpolated voxel label fields cross.</summary>
        Exact,
        /// <summary>Project square tiles from the XZ block grid onto the surface.</summary>
        Cubic
    }

    /// <summary>A terrain surface point and its outward density-gradient normal.</summary>
    public struct TerrainVertex
    {
        public Vector3d position;
        public Vector3f normal;
        public uint color;
        public uint material;

        public TerrainVertex(Vector3d position, Vector3f normal, uint color, uint material = 0)
        {
            this.position = position;
            this.normal = normal;
            this.color = color;
            this.material = material;
        }
    }

    /// <summary>Three indices into a terrain result's vertices and the triangle's material ID.</summary>
    public struct TerrainTriangle
    {
        public int a, b, c;
        public uint material;

        public TerrainTriangle(int a, int b, int c, uint material)
        {
            this.a = a;
            this.b = b;
            this.c = c;
            this.material = material;
        }
    }

    /// <summary>Raw terrain triangles with density normals and optional surface seams.</summary>
    public class TerrainMesherResult
    {
        public IList<TerrainVertex> Vertices;
        public IList<TerrainTriangle> Triangles;
        public IList<Edge> MaterialBoundaries;
        public IList<Edge> ColorBoundaries;
    }
}
