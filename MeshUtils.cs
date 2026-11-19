using g4;
using System.Collections.Generic;

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

    /// <summary>Triangle face produced where a modifier cannot preserve a quad.</summary>
    public struct TriangleFace
    {
        public Vector3d a, b, c;
        public uint color, material;

        public TriangleFace(Vector3d a, Vector3d b, Vector3d c, uint color, uint material)
        {
            this.a = a;
            this.b = b;
            this.c = c;
            this.color = color;
            this.material = material;
        }
    }

    /// <summary>Planar polygon retaining its boundary until a render mesh is built.</summary>
    public struct NgonFace
    {
        public IList<Vector3d> points;
        public uint color, material;

        public NgonFace(IList<Vector3d> points, uint color, uint material)
        {
            this.points = points;
            this.color = color;
            this.material = material;
        }

        /// <summary>Triangulates a concave planar polygon without changing its stored outline.</summary>
        public IEnumerable<TriangleFace> Triangulate()
        {
            if (points == null || points.Count < 3) yield break;
            var remaining = new List<int>(points.Count);
            for (int i = 0; i < points.Count; i++)
                if (remaining.Count == 0 ||
                    (points[i] - points[remaining[remaining.Count - 1]]).LengthSquared > 1e-20)
                    remaining.Add(i);
            if (remaining.Count > 1 &&
                (points[remaining[0]] - points[remaining[remaining.Count - 1]]).LengthSquared < 1e-20)
                remaining.RemoveAt(remaining.Count - 1);
            if (remaining.Count < 3) yield break;

            Vector3d normal = Vector3d.Zero;
            for (int i = 0; i < remaining.Count; i++)
                normal += Vector3d.Cross(points[remaining[i]], points[remaining[(i + 1) % remaining.Count]]);
            if (normal.LengthSquared < 1e-20) yield break;
            while (remaining.Count > 3)
            {
                bool found = false;
                for (int i = 0; i < remaining.Count; i++)
                {
                    int previous = remaining[(i + remaining.Count - 1) % remaining.Count];
                    int current = remaining[i];
                    int next = remaining[(i + 1) % remaining.Count];
                    Vector3d a = points[previous], b = points[current], c = points[next];
                    if (Vector3d.Dot(Vector3d.Cross(b - a, c - b), normal) <= 1e-16)
                        continue;

                    bool occupied = false;
                    foreach (int other in remaining)
                    {
                        if (other == previous || other == current || other == next) continue;
                        Vector3d p = points[other];
                        if (Vector3d.Dot(Vector3d.Cross(b - a, p - a), normal) >= -1e-16 &&
                            Vector3d.Dot(Vector3d.Cross(c - b, p - b), normal) >= -1e-16 &&
                            Vector3d.Dot(Vector3d.Cross(a - c, p - c), normal) >= -1e-16)
                        {
                            occupied = true;
                            break;
                        }
                    }
                    if (occupied) continue;
                    yield return new TriangleFace(a, b, c, color, material);
                    remaining.RemoveAt(i);
                    found = true;
                    break;
                }
                if (!found)
                {
                    for (int i = 0; i < remaining.Count; i++)
                    {
                        Vector3d a = points[remaining[(i + remaining.Count - 1) % remaining.Count]];
                        Vector3d b = points[remaining[i]];
                        Vector3d c = points[remaining[(i + 1) % remaining.Count]];
                        if (Vector3d.Cross(b - a, c - b).LengthSquared > 1e-20) continue;
                        remaining.RemoveAt(i);
                        found = true;
                        break;
                    }
                    if (!found) yield break;
                }
            }
            if (remaining.Count == 3)
                yield return new TriangleFace(points[remaining[0]], points[remaining[1]],
                    points[remaining[2]], color, material);
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
