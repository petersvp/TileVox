using g4;
using System.Collections.Generic;

namespace TileVox.Meshing
{
    /// <summary>Block faces and classified boundaries for downstream mesh creation.</summary>
    public class StructureMesherResult3D
    {
        /// <summary>Position-based quad faces with color and material metadata.</summary>
        public IList<Face> Quads;

        /// <summary>Faces whose boundary follows an ending bevel profile.</summary>
        public IList<NgonFace> Ngons;

        /// <summary>Triangle faces created by modifiers at corners and irregular boundaries.</summary>
        public IList<TriangleFace> Triangles;

        /// <summary>Final indexed mesh produced by the welder.</summary>
        public DMesh3 Mesh;

        /// <summary>Alpha by DMesh vertex ID; DMesh3 stores only RGB vertex colors.</summary>
        public IList<byte> VertexAlpha;

        /// <summary>Maps material IDs to triangle groups in Mesh.</summary>
        public Dictionary<uint, int> MaterialGroups;

        /// <summary>Climbable lips where a floor surface meets a descending wall.</summary>
        public IList<Edge> LedgeBoundaries;

        /// <summary>Edges where a floor meets a rising wall.</summary>
        public IList<Edge> BaseBoundaries;

        /// <summary>Edges where a descending wall meets a ceiling underside (Г profile).</summary>
        public IList<Edge> WallToCeilingBoundaries;

        /// <summary>Edges where a ceiling underside meets a rising wall (L profile).</summary>
        public IList<Edge> CeilingToWallBoundaries;

        /// <summary>Exposed wall edges and corners.</summary>
        public IList<Edge> WallCornerBoundaries;

        /// <summary>Exposed surface edges shared by faces with different material IDs.</summary>
        public IList<Edge> MaterialBoundaries;

        /// <summary>Exposed surface edges shared by faces with different packed colors.</summary>
        public IList<Edge> ColorBoundaries;
    }
}
