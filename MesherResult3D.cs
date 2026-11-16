using g4;
using System.Collections.Generic;

namespace TileVox
{
    /// <summary>Position-based block quads and boundaries, or indexed geometry from a modifier.</summary>
    public class MesherResult3D
    {
        /// <summary>Indexed geometry used by triangle-based modifiers; block meshers fill MeshQuads instead.</summary>
        public DMesh3 Mesh { get; }

        /// <summary>Maps each source material ID to its triangle or quad group.</summary>
        public Dictionary<uint, int> MaterialGroups { get; } = new();

        /// <summary>Optional extracted triangle mesh for each material ID.</summary>
        public Dictionary<uint, DSubmesh3> MaterialSubmeshes { get; } = new();

        /// <summary>Alpha in mesh vertex-ID order; DMesh3 vertex colors contain RGB only.</summary>
        public List<byte> VertexAlpha { get; } = new();

        /// <summary>Optional tangent and handedness in mesh vertex-ID order.</summary>
        public List<Vector4f> VertexTangents { get; } = new();

        /// <summary>Quad vertex IDs and material groups; each quad owns four separate vertices in Mesh.</summary>
        public List<(int a, int b, int c, int d, int group)> Quads { get; } = new();

        /// <summary>Position-based block quads with color and numeric material ID, before vertex generation.</summary>
        public List<(Vector3d a, Vector3d b, Vector3d c, Vector3d d, uint color, uint material)> MeshQuads { get; } = new();

        /// <summary>Climbable lips where a floor surface meets a descending wall.</summary>
        public List<(Vector3d a, Vector3d b)> FloorBoundaries { get; } = new();

        /// <summary>Edges where a floor meets a rising wall.</summary>
        public List<(Vector3d a, Vector3d b)> GroundBoundaries { get; } = new();

        /// <summary>Exposed wall edges and corners.</summary>
        public List<(Vector3d a, Vector3d b)> WallBoundaries { get; } = new();

        /// <summary>Geometric edges explicitly marked as hard, with endpoints in ascending position order.</summary>
        public HashSet<(Vector3d a, Vector3d b)> HardEdges { get; } = new();

        /// <summary>Creates a result with color, UV, and triangle-group channels.</summary>
        public MesherResult3D(bool vertexNormals)
        {
            Mesh = new DMesh3(vertexNormals, true, true, true);
        }
    }
}
