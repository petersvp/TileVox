using g4;
using System;
using System.Collections.Generic;

namespace TileVox
{
    /// <summary>Bevels hard edges in a quad mesher result without welding its vertices.</summary>
    /// <remarks>Shared positions identify topology; a discontinuity in the source vertex normals identifies a hard edge. Face interiors and bevel bands remain quads. Corner caps use triangles.</remarks>
    public class ModifierBevel
    {
        private struct VertexData
        {
            public Vector3d position;
            public Vector3f normal;
            public Vector3f color;
            public Vector2f uv;
            public Vector4f tangent;
            public byte alpha;
        }

        private class FaceData
        {
            public int[] positions;
            public VertexData[] corners;
            public int group;
        }

        private class EdgeData
        {
            public int start, end;
            public int faceA = -1, faceB = -1;
            public bool bevel;
        }

        /// <summary>Quad mesh to modify.</summary>
        public MesherResult3D source;

        /// <summary>Distance inset from each hard edge, in mesh units. Zero preserves the shape and averages coincident normals.</summary>
        public double width = 0.1;

        /// <summary>Minimum normal break, in degrees, at either end of an edge.</summary>
        public double angleDegrees = 30;

        /// <summary>Number of bevel quads across an edge. One produces a straight chamfer.</summary>
        public int segments = 1;

        /// <summary>Creates a modifier for a quad mesh.</summary>
        public ModifierBevel(MesherResult3D source)
        {
            this.source = source;
        }

        /// <summary>Builds a new mesh with beveled hard edges and smoothed zero-width seams.</summary>
        public MesherResult3D Apply()
        {
            if (source == null) throw new InvalidOperationException("A source mesh is required.");
            if (source.Quads.Count == 0) throw new ArgumentException("Bevel requires a quad mesher result.");
            if (source.Mesh.TriangleCount != 0) throw new ArgumentException("Bevel requires a quad-only source mesh.");
            if (width < 0 || double.IsNaN(width)) throw new ArgumentOutOfRangeException(nameof(width));
            if (segments < 1) throw new ArgumentOutOfRangeException(nameof(segments));

            var positions = new List<Vector3d>();
            var positionIds = new Dictionary<(double x, double y, double z), int>();
            var faces = new List<FaceData>(source.Quads.Count);
            var edges = new Dictionary<(int a, int b), EdgeData>();
            foreach (var quad in source.Quads)
            {
                int[] vertices = { quad.a, quad.b, quad.c, quad.d };
                var face = new FaceData
                {
                    positions = new int[4],
                    corners = new VertexData[4],
                    group = quad.group
                };
                for (int i = 0; i < 4; i++)
                {
                    int vertex = vertices[i];
                    Vector3d position = source.Mesh.GetVertex(vertex);
                    var key = (position.x, position.y, position.z);
                    if (!positionIds.TryGetValue(key, out int positionId))
                    {
                        positionId = positions.Count;
                        positionIds.Add(key, positionId);
                        positions.Add(position);
                    }
                    face.positions[i] = positionId;
                    face.corners[i] = ReadVertex(vertex, position);
                }
                if (!source.Mesh.HasVertexNormals)
                {
                    Vector3d normal = Vector3d.Cross(face.corners[1].position - face.corners[0].position,
                        face.corners[3].position - face.corners[0].position).Normalized;
                    for (int i = 0; i < 4; i++)
                        face.corners[i].normal = new Vector3f((float)normal.x, (float)normal.y, (float)normal.z);
                }

                int faceId = faces.Count;
                faces.Add(face);
                for (int i = 0; i < 4; i++)
                {
                    int a = face.positions[i], b = face.positions[(i + 1) % 4];
                    if (a == b) throw new ArgumentException("Bevel requires nondegenerate quads.");
                    var key = a < b ? (a, b) : (b, a);
                    if (!edges.TryGetValue(key, out EdgeData edge))
                    {
                        edge = new EdgeData { start = a, end = b, faceA = faceId };
                        edges.Add(key, edge);
                    }
                    else if (edge.faceB < 0) edge.faceB = faceId;
                    else throw new ArgumentException("Bevel requires at most two quads per geometric edge.");
                }
            }

            var result = new MesherResult3D(true);
            foreach (var material in source.MaterialGroups)
                result.MaterialGroups.Add(material.Key, material.Value);

            if (width == 0)
            {
                var normalSums = new Vector3d[positions.Count];
                foreach (FaceData face in faces)
                    for (int i = 0; i < 4; i++)
                    {
                        Vector3f normal = face.corners[i].normal;
                        normalSums[face.positions[i]] += new Vector3d(normal.x, normal.y, normal.z);
                    }
                foreach (FaceData face in faces)
                {
                    var corners = (VertexData[])face.corners.Clone();
                    for (int i = 0; i < 4; i++)
                    {
                        Vector3d normal = normalSums[face.positions[i]].Normalized;
                        corners[i].normal = new Vector3f((float)normal.x, (float)normal.y, (float)normal.z);
                    }
                    AddFace(result, corners, face.group);
                }
                return result;
            }

            double threshold = Math.Cos(Math.Max(0, Math.Min(180, angleDegrees)) * Math.PI / 180);
            bool hasBevel = false;
            foreach (EdgeData edge in edges.Values)
            {
                if (edge.faceB < 0) continue;
                FaceData a = faces[edge.faceA], b = faces[edge.faceB];
                int aStart = CornerIndex(a, edge.start), aEnd = CornerIndex(a, edge.end);
                int bStart = CornerIndex(b, edge.start), bEnd = CornerIndex(b, edge.end);
                double startDot = DotNormal(a.corners[aStart].normal, b.corners[bStart].normal);
                double endDot = DotNormal(a.corners[aEnd].normal, b.corners[bEnd].normal);
                bool marked = source.HardEdges.Count == 0 || source.HardEdges.Contains(
                    positions[edge.start].CompareTo(positions[edge.end]) <= 0
                        ? (positions[edge.start], positions[edge.end])
                        : (positions[edge.end], positions[edge.start]));
                edge.bevel = marked && Math.Min(startDot, endDot) < threshold - 0.000001;
                hasBevel |= edge.bevel;
            }
            if (!hasBevel)
            {
                foreach (FaceData face in faces) AddFace(result, face.corners, face.group);
                return result;
            }

            // A single width keeps both ends of every inset edge apart, including acute corners.
            double inset = width;
            foreach (FaceData face in faces)
                for (int i = 0; i < 4; i++)
                {
                    int prev = (i + 3) % 4, next = (i + 1) % 4;
                    bool bevelPrev = GetEdge(edges, face.positions[prev], face.positions[i]).bevel;
                    bool bevelNext = GetEdge(edges, face.positions[i], face.positions[next]).bevel;
                    if (!bevelPrev && !bevelNext) continue;
                    Vector3d toPrev = positions[face.positions[prev]] - positions[face.positions[i]];
                    Vector3d toNext = positions[face.positions[next]] - positions[face.positions[i]];
                    double sine = Vector3d.Cross(toPrev.Normalized, toNext.Normalized).Length;
                    inset = Math.Min(inset, 0.45 * Math.Min(toPrev.Length, toNext.Length) * sine);
                }
            if (inset <= 0) throw new ArgumentException("Bevel requires nondegenerate quad corners.");

            foreach (FaceData face in faces)
            {
                var original = (VertexData[])face.corners.Clone();
                for (int i = 0; i < 4; i++)
                {
                    int prev = (i + 3) % 4, next = (i + 1) % 4;
                    bool bevelPrev = GetEdge(edges, face.positions[prev], face.positions[i]).bevel;
                    bool bevelNext = GetEdge(edges, face.positions[i], face.positions[next]).bevel;
                    if (!bevelPrev && !bevelNext) continue;
                    VertexData corner = original[i];
                    Vector3d toPrev = original[prev].position - corner.position;
                    Vector3d toNext = original[next].position - corner.position;
                    double sine = Vector3d.Cross(toPrev.Normalized, toNext.Normalized).Length;
                    double prevFraction = (bevelNext ? inset : 0) / (sine * toPrev.Length);
                    double nextFraction = (bevelPrev ? inset : 0) / (sine * toNext.Length);
                    corner.position += toPrev * prevFraction + toNext * nextFraction;
                    corner.uv += (original[prev].uv - corner.uv) * (float)prevFraction
                        + (original[next].uv - corner.uv) * (float)nextFraction;
                    face.corners[i] = corner;
                }
            }

            var bevelVertices = new HashSet<int>();
            foreach (EdgeData edge in edges.Values)
                if (edge.bevel) { bevelVertices.Add(edge.start); bevelVertices.Add(edge.end); }
            var caps = new List<VertexData>[positions.Count];
            var capGroups = new int[positions.Count];
            foreach (FaceData face in faces)
            {
                AddFace(result, face.corners, face.group);
                for (int i = 0; i < 4; i++)
                {
                    int positionId = face.positions[i];
                    if (!bevelVertices.Contains(positionId)) continue;
                    if (caps[positionId] == null)
                    {
                        caps[positionId] = new List<VertexData>();
                        capGroups[positionId] = face.group;
                    }
                    caps[positionId].Add(face.corners[i]);
                }
            }

            foreach (EdgeData edge in edges.Values)
            {
                if (!edge.bevel) continue;
                FaceData a = faces[edge.faceA], b = faces[edge.faceB];
                VertexData aStart = a.corners[CornerIndex(a, edge.start)];
                VertexData aEnd = a.corners[CornerIndex(a, edge.end)];
                VertexData bStart = b.corners[CornerIndex(b, edge.start)];
                VertexData bEnd = b.corners[CornerIndex(b, edge.end)];
                for (int segment = 0; segment < segments; segment++)
                {
                    double t0 = (double)segment / segments;
                    double t1 = (double)(segment + 1) / segments;
                    VertexData p0 = BevelPoint(aStart, bStart, positions[edge.start], t0);
                    VertexData p1 = BevelPoint(aEnd, bEnd, positions[edge.end], t0);
                    VertexData p2 = BevelPoint(aEnd, bEnd, positions[edge.end], t1);
                    VertexData p3 = BevelPoint(aStart, bStart, positions[edge.start], t1);
                    Vector3d expected = ToDouble(aStart.normal) + ToDouble(bStart.normal);
                    var band = Vector3d.Dot(Vector3d.Cross(p1.position - p0.position,
                        p3.position - p0.position), expected) >= 0
                        ? new[] { p0, p1, p2, p3 } : new[] { p1, p0, p3, p2 };
                    AddFace(result, band, a.group);
                    if (segment + 1 < segments)
                    {
                        caps[edge.start].Add(p3);
                        caps[edge.end].Add(p2);
                    }
                }
            }

            foreach (int positionId in bevelVertices)
                AddCap(result, caps[positionId], capGroups[positionId]);
            return result;
        }

        private VertexData ReadVertex(int id, Vector3d position)
        {
            DMesh3 mesh = source.Mesh;
            return new VertexData
            {
                position = position,
                normal = mesh.HasVertexNormals ? mesh.GetVertexNormal(id) : new Vector3f(0, 1, 0),
                color = mesh.HasVertexColors ? mesh.GetVertexColor(id) : new Vector3f(1, 1, 1),
                uv = mesh.HasVertexUVs ? mesh.GetVertexUV(id) : new Vector2f(0, 0),
                tangent = id < source.VertexTangents.Count ? source.VertexTangents[id] : new Vector4f(0),
                alpha = id < source.VertexAlpha.Count ? source.VertexAlpha[id] : (byte)255
            };
        }

        private static EdgeData GetEdge(Dictionary<(int a, int b), EdgeData> edges, int a, int b)
            => edges[a < b ? (a, b) : (b, a)];

        private static int CornerIndex(FaceData face, int positionId)
        {
            for (int i = 0; i < 4; i++) if (face.positions[i] == positionId) return i;
            throw new InvalidOperationException("A face does not contain its edge endpoint.");
        }

        private static Vector3d ToDouble(Vector3f value) => new Vector3d(value.x, value.y, value.z);

        private static double DotNormal(Vector3f a, Vector3f b)
        {
            Vector3d x = ToDouble(a), y = ToDouble(b);
            if (x.LengthSquared < 0.00000001 || y.LengthSquared < 0.00000001) return 1;
            return Vector3d.Dot(x.Normalized, y.Normalized);
        }

        private static VertexData BevelPoint(VertexData a, VertexData b, Vector3d corner, double t)
        {
            double oneMinusT = 1 - t;
            Vector3d normal = (ToDouble(a.normal) * oneMinusT + ToDouble(b.normal) * t).Normalized;
            return new VertexData
            {
                position = a.position * (oneMinusT * oneMinusT) + corner * (2 * oneMinusT * t)
                    + b.position * (t * t),
                normal = new Vector3f((float)normal.x, (float)normal.y, (float)normal.z),
                color = a.color * (float)oneMinusT + b.color * (float)t,
                uv = a.uv * (float)oneMinusT + b.uv * (float)t,
                tangent = a.tangent * (float)oneMinusT + b.tangent * (float)t,
                alpha = (byte)Math.Round(a.alpha * oneMinusT + b.alpha * t)
            };
        }

        private static void AddCap(MesherResult3D result, List<VertexData> points, int group)
        {
            var unique = new List<VertexData>(points.Count);
            foreach (VertexData point in points)
            {
                bool found = false;
                foreach (VertexData other in unique)
                    if ((point.position - other.position).LengthSquared < 0.000000000001)
                    { found = true; break; }
                if (!found) unique.Add(point);
            }
            if (unique.Count < 3) return;

            Vector3d center = new Vector3d(0, 0, 0), normal = new Vector3d(0, 0, 0);
            Vector2f uv = new Vector2f(0, 0);
            Vector3f color = new Vector3f(0, 0, 0);
            double alpha = 0;
            foreach (VertexData point in unique)
            {
                center += point.position;
                normal += ToDouble(point.normal);
                uv += point.uv;
                color += point.color;
                alpha += point.alpha;
            }
            center /= unique.Count;
            normal = normal.Normalized;
            if (normal.LengthSquared < 0.00000001) return;
            Vector3d axisX = (unique[0].position - center).Normalized;
            if (axisX.LengthSquared < 0.00000001) return;
            Vector3d axisY = Vector3d.Cross(normal, axisX).Normalized;
            unique.Sort((a, b) => Math.Atan2(Vector3d.Dot(a.position - center, axisY),
                Vector3d.Dot(a.position - center, axisX)).CompareTo(Math.Atan2(
                Vector3d.Dot(b.position - center, axisY), Vector3d.Dot(b.position - center, axisX))));

            if (unique.Count == 3)
            {
                VertexData a = unique[0], b = unique[1], c = unique[2];
                Vector3d cross = Vector3d.Cross(b.position - a.position, c.position - a.position);
                if (cross.LengthSquared > 0.000000000001)
                    AddFace(result, Vector3d.Dot(cross, normal) >= 0
                        ? new[] { a, b, c } : new[] { a, c, b }, group);
                return;
            }

            var middle = new VertexData
            {
                position = center,
                normal = new Vector3f((float)normal.x, (float)normal.y, (float)normal.z),
                uv = uv / unique.Count,
                color = color / unique.Count,
                alpha = (byte)Math.Round(alpha / unique.Count),
                tangent = new Vector4f(0)
            };
            for (int i = 0; i < unique.Count; i++)
            {
                VertexData a = unique[i], b = unique[(i + 1) % unique.Count];
                Vector3d cross = Vector3d.Cross(a.position - center, b.position - center);
                if (cross.LengthSquared < 0.000000000001) continue;
                AddFace(result, Vector3d.Dot(cross, normal) >= 0
                    ? new[] { middle, a, b } : new[] { middle, b, a }, group);
            }
        }

        private static void AddFace(MesherResult3D result, VertexData[] corners, int group)
        {
            var ids = new int[corners.Length];
            for (int i = 0; i < corners.Length; i++)
            {
                VertexData corner = corners[i];
                ids[i] = result.Mesh.AppendVertex(new NewVertexInfo(corner.position,
                    corner.normal, corner.color, corner.uv));
                result.VertexAlpha.Add(corner.alpha);
                result.VertexTangents.Add(corner.tangent);
            }
            if (corners.Length == 4)
            {
                result.Quads.Add((ids[0], ids[1], ids[2], ids[3], group));
                result.Mesh.AppendTriangle(ids[0], ids[1], ids[2], group);
                result.Mesh.AppendTriangle(ids[0], ids[2], ids[3], group);
            }
            else result.Mesh.AppendTriangle(ids[0], ids[1], ids[2], group);
        }
    }
}
