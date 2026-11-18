using System.Collections.Generic;

namespace TileVox.Meshing
{
    /// <summary>Block faces and classified boundaries for downstream mesh creation.</summary>
    public class StructureMesherResult3D
    {
        /// <summary>Position-based quad faces with color and material metadata.</summary>
        public IList<Face> Quads;

        /// <summary>Climbable lips where a floor surface meets a descending wall.</summary>
        public IList<Edge> FloorBoundaries;

        /// <summary>Edges where a floor meets a rising wall.</summary>
        public IList<Edge> GroundBoundaries;

        /// <summary>Edges where a wall meets a ceiling underside.</summary>
        public IList<Edge> CeilingBoundaries;

        /// <summary>Exposed wall edges and corners.</summary>
        public IList<Edge> WallBoundaries;

        /// <summary>Exposed surface edges shared by faces with different material IDs.</summary>
        public IList<Edge> MaterialBoundaries;

        /// <summary>Exposed surface edges shared by faces with different packed colors.</summary>
        public IList<Edge> ColorBoundaries;
    }
}
