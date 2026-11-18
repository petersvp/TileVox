using g4;

namespace TileVox.Meshing
{
    /// <summary>Material and color associated with a stored cell.</summary>
    public struct CellData
    {
        // Might need expansion. 
        public uint color, material;
    }

    /// <summary>Axis-aligned exposed face of a stored cell.</summary>
    public struct FaceData
    {
        public int x, y, z, axis, sign;
        public uint color, material;
    }

    /// <summary>Surface types incident on an edge.</summary>
    public struct EdgeData
    {
        public int faceCount;
        public bool top, bottom, sideAbove, sideBelow;
        public bool materialSeam, colorSeam;
        public uint material, color;
        public int wallMask;
    }

    /// <summary>Quad face and the metadata needed to construct its mesh vertices.</summary>
    public struct Face
    {
        public Vector3d a, b, c, d;
        public uint color, material;

        public Face(Vector3d a, Vector3d b, Vector3d c, Vector3d d, uint color, uint material)
        {
            this.a = a;
            this.b = b;
            this.c = c;
            this.d = d;
            this.color = color;
            this.material = material;
        }
    }

    /// <summary>Boundary edge between two mesh positions.</summary>
    public struct Edge
    {
        public Vector3d a, b;

        public Edge(Vector3d a, Vector3d b)
        {
            this.a = a;
            this.b = b;
        }

    }
}
