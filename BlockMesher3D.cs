using g4;
using PiDev.VoxelCore;
using System;
using System.Collections.Generic;
using TileVox;

namespace TileVox.Meshing
{
    /// <summary>Selects unit quads or merged coplanar quads.</summary>
    public enum BlockMeshingMode
    {
        QuadPerTile,
        PlanarSimplified
    }

    /// <summary>Builds block quads and classified surface boundaries from stored voxels.</summary>
    /// <remarks>BuildMesh keeps working data local. The source and settings must remain stable during a build.</remarks>
    public class BlockMesher3D<T> : IVoxelDataMesher3D
    {
        private struct FaceGroupKey : IEquatable<FaceGroupKey>
        {
            public int axis, sign, plane;
            public uint color, material;

            public FaceGroupKey(int axis, int sign, int plane, uint color, uint material)
            {
                this.axis = axis;
                this.sign = sign;
                this.plane = plane;
                this.color = color;
                this.material = material;
            }

            public bool Equals(FaceGroupKey other) => axis == other.axis && sign == other.sign &&
                plane == other.plane && color == other.color && material == other.material;
            public override bool Equals(object obj) => obj is FaceGroupKey other && Equals(other);
            public override int GetHashCode()
            {
                unchecked
                {
                    int hash = axis;
                    hash = hash * 31 + sign;
                    hash = hash * 31 + plane;
                    hash = hash * 31 + (int)color;
                    return hash * 31 + (int)material;
                }
            }
        }

        private struct FaceGroupData
        {
            public List<Vector2i> order;
            public HashSet<Vector2i> cells;
        }

        private struct EdgeKey : IEquatable<EdgeKey>
        {
            public Vector3i a, b;

            public EdgeKey(Vector3i a, Vector3i b)
            {
                this.a = a;
                this.b = b;
            }

            public bool Equals(EdgeKey other) => a.Equals(other.a) && b.Equals(other.b);
            public override bool Equals(object obj) => obj is EdgeKey other && Equals(other);
            public override int GetHashCode()
            {
                unchecked { return a.GetHashCode() * 31 + b.GetHashCode(); }
            }
        }

        public IVoxelDataAccess3D<T> data;
        public Range3D range;
        public BlockMeshingMode mode = BlockMeshingMode.QuadPerTile;

        /// <summary>Creates a mesher whose settings can be supplied before BuildMesh.</summary>
        public BlockMesher3D() { }

        /// <summary>Creates a mesher for a voxel source and inclusive output range.</summary>
        public BlockMesher3D(IVoxelDataAccess3D<T> data, Range3D range)
        {
            this.data = data;
            this.range = range;
        }

        /// <summary>Enumerates stored cells once and builds quads and boundary segments.</summary>
        public StructureMesherResult3D BuildMesh()
        {
            var result = new StructureMesherResult3D();
            result.Quads = new List<Face>();
            result.FloorBoundaries = new List<Edge>();
            result.GroundBoundaries = new List<Edge>();
            result.CeilingBoundaries = new List<Edge>();
            result.WallBoundaries = new List<Edge>();
            result.MaterialBoundaries = new List<Edge>();
            result.ColorBoundaries = new List<Edge>();
            var source = data ?? throw new InvalidOperationException("A voxel source is required.");
            var bounds = range;
            if (bounds.minX > bounds.maxX || bounds.minY > bounds.maxY || bounds.minZ > bounds.maxZ)
                return result;

            var halo = new Range3D(
                bounds.minX == int.MinValue ? int.MinValue : bounds.minX - 1,
                bounds.maxX == int.MaxValue ? int.MaxValue : bounds.maxX + 1,
                bounds.minY == int.MinValue ? int.MinValue : bounds.minY - 1,
                bounds.maxY == int.MaxValue ? int.MaxValue : bounds.maxY + 1,
                bounds.minZ == int.MinValue ? int.MinValue : bounds.minZ - 1,
                bounds.maxZ == int.MaxValue ? int.MaxValue : bounds.maxZ + 1);
            var cells = new Dictionary<Vector3i, CellData>();
            foreach (var sequence in source.EnumerateRange(halo, false))
                foreach (var cell in sequence)
                {
                    if (!halo.Contains(cell.x, cell.y, cell.z) || !IsSolid(cell.value)) continue;
                    cells[new Vector3i(cell.x, cell.y, cell.z)] = new CellData
                    {
                        color = GetColor(cell.value),
                        material = GetMaterial(cell.value)
                    };
                }

            var faces = new List<FaceData>();
            foreach (var entry in cells)
            {
                var p = entry.Key;
                if (!bounds.Contains(p.x, p.y, p.z)) continue;
                for (int axis = 0; axis < 3; axis++)
                    for (int sign = -1; sign <= 1; sign += 2)
                    {
                        int x = p.x + (axis == 0 ? sign : 0);
                        int y = p.y + (axis == 1 ? sign : 0);
                        int z = p.z + (axis == 2 ? sign : 0);
                        if (cells.ContainsKey(new Vector3i(x, y, z))) continue;
                        faces.Add(new FaceData
                        {
                            x = p.x, y = p.y, z = p.z,
                            axis = axis, sign = sign,
                            color = entry.Value.color, material = entry.Value.material
                        });
                    }
            }

            if (mode == BlockMeshingMode.QuadPerTile) BuildQuadsPerTile(result, faces);
            else BuildPlanarSimplified(result, faces);
            return result;
        }

        private static void BuildQuadsPerTile(StructureMesherResult3D result, List<FaceData> faces)
        {
            foreach (var face in faces)
            {
                GetFacePlane(face, out int plane, out int u, out int v);
                AddQuad(result, face.axis, face.sign, plane, u, v, 1, 1, face.color, face.material);
            }
            BuildBoundaries(result, faces);
        }

        private static void BuildPlanarSimplified(StructureMesherResult3D result, List<FaceData> faces)
        {
            var groups = new Dictionary<FaceGroupKey, FaceGroupData>();
            foreach (var face in faces)
            {
                GetFacePlane(face, out int plane, out int u, out int v);
                var key = new FaceGroupKey(face.axis, face.sign, plane, face.color, face.material);
                if (!groups.TryGetValue(key, out var group))
                {
                    group = new FaceGroupData { order = new List<Vector2i>(), cells = new HashSet<Vector2i>() };
                    groups.Add(key, group);
                }
                group.order.Add(new Vector2i(u, v));
                group.cells.Add(new Vector2i(u, v));
            }

            foreach (var entry in groups)
            {
                var remaining = entry.Value.cells;
                foreach (var point in entry.Value.order)
                {
                    if (!remaining.Contains(point)) continue;
                    int width = 1, height = 1;
                    while (remaining.Contains(new Vector2i(point.x + width, point.y))) width++;
                    bool extend = true;
                    while (extend)
                    {
                        for (int x = 0; x < width; x++)
                            if (!remaining.Contains(new Vector2i(point.x + x, point.y + height)))
                            {
                                extend = false;
                                break;
                            }
                        if (extend) height++;
                    }
                    for (int y = 0; y < height; y++)
                        for (int x = 0; x < width; x++)
                            remaining.Remove(new Vector2i(point.x + x, point.y + y));
                    var key = entry.Key;
                    AddQuad(result, key.axis, key.sign, key.plane, point.x, point.y,
                        width, height, key.color, key.material);
                }
            }
            BuildBoundaries(result, faces);
            MergeSegments(result.FloorBoundaries);
            MergeSegments(result.GroundBoundaries);
            MergeSegments(result.CeilingBoundaries);
            MergeSegments(result.WallBoundaries);
            MergeSegments(result.MaterialBoundaries);
            MergeSegments(result.ColorBoundaries);
        }

        private static void GetFacePlane(FaceData face, out int plane, out int u, out int v)
        {
            plane = (face.axis == 0 ? face.x : face.axis == 1 ? face.y : face.z) + (face.sign > 0 ? 1 : 0);
            u = face.axis == 0 ? face.y : face.axis == 1 ? face.z : face.x;
            v = face.axis == 0 ? face.z : face.axis == 1 ? face.x : face.y;
        }

        private static Vector3i ToPoint(int axis, int plane, int u, int v)
        {
            return axis == 0 ? new Vector3i(plane, u, v)
                : axis == 1 ? new Vector3i(v, plane, u) : new Vector3i(u, v, plane);
        }

        private static Vector3d ToVector(Vector3i p) => new Vector3d(p.x, p.y, p.z);

        private static void AddQuad(StructureMesherResult3D result, int axis, int sign, int plane,
            int u, int v, int width, int height, uint color, uint material)
        {
            Vector3d a = ToVector(ToPoint(axis, plane, u, v));
            Vector3d b = ToVector(ToPoint(axis, plane, u + width, v));
            Vector3d c = ToVector(ToPoint(axis, plane, u + width, v + height));
            Vector3d d = ToVector(ToPoint(axis, plane, u, v + height));
            if (sign > 0) result.Quads.Add(new Face(a, b, c, d, color, material));
            else result.Quads.Add(new Face(a, d, c, b, color, material));
        }

        private static void BuildBoundaries(StructureMesherResult3D result, List<FaceData> faces)
        {
            var edges = new Dictionary<EdgeKey, EdgeData>();
            foreach (var face in faces)
            {
                GetFacePlane(face, out int plane, out int u, out int v);
                var a = ToPoint(face.axis, plane, u, v);
                var b = ToPoint(face.axis, plane, u + 1, v);
                var c = ToPoint(face.axis, plane, u + 1, v + 1);
                var d = ToPoint(face.axis, plane, u, v + 1);
                AddEdge(edges, a, b, face);
                AddEdge(edges, b, c, face);
                AddEdge(edges, c, d, face);
                AddEdge(edges, d, a, face);
            }

            foreach (var entry in edges)
            {
                EdgeData edge = entry.Value;
                var segment = new Edge(ToVector(entry.Key.a), ToVector(entry.Key.b));
                if (edge.materialSeam) result.MaterialBoundaries.Add(segment);
                if (edge.colorSeam) result.ColorBoundaries.Add(segment);
                if (edge.top)
                {
                    if (edge.sideBelow || edge.wallMask == 0 && edge.faceCount == 1)
                        result.FloorBoundaries.Add(segment);
                    if (edge.sideAbove) result.GroundBoundaries.Add(segment);
                }
                else if (edge.bottom && edge.wallMask != 0)
                    result.CeilingBoundaries.Add(segment);
                else if (edge.wallMask != 0 &&
                    (edge.faceCount == 1 || (edge.wallMask & (edge.wallMask - 1)) != 0))
                    result.WallBoundaries.Add(segment);
            }
        }

        private static void AddEdge(
            Dictionary<EdgeKey, EdgeData> edges,
            Vector3i a, Vector3i b, FaceData face)
        {
            if (ComparePoints(a, b) > 0)
            {
                Vector3i point = a;
                a = b;
                b = point;
            }
            var key = new EdgeKey(a, b);
            edges.TryGetValue(key, out EdgeData edge);
            if (edge.faceCount == 0)
            {
                edge.material = face.material;
                edge.color = face.color;
            }
            else
            {
                if (edge.material != face.material) edge.materialSeam = true;
                if (edge.color != face.color) edge.colorSeam = true;
            }
            edge.faceCount++;
            if (face.axis == 1)
            {
                if (face.sign > 0) edge.top = true;
                else edge.bottom = true;
            }
            else
            {
                edge.wallMask |= face.axis == 0 ? (face.sign > 0 ? 1 : 2) : (face.sign > 0 ? 4 : 8);
                if (a.y == b.y)
                {
                    if (a.y == face.y) edge.sideAbove = true;
                    else edge.sideBelow = true;
                }
            }
            edges[key] = edge;
        }

        private static int ComparePoints(Vector3i a, Vector3i b)
        {
            int x = a.x.CompareTo(b.x);
            if (x != 0) return x;
            int y = a.y.CompareTo(b.y);
            return y != 0 ? y : a.z.CompareTo(b.z);
        }

        private static void MergeSegments(IList<Edge> segments)
        {
            var lines = new Dictionary<Vector3i, List<Vector2i>>();
            foreach (var segment in segments)
            {
                Vector3d a = segment.a, b = segment.b;
                int axis = a.x != b.x ? 0 : a.y != b.y ? 1 : 2;
                var key = axis == 0 ? new Vector3i(axis, (int)a.y, (int)a.z)
                    : axis == 1 ? new Vector3i(axis, (int)a.x, (int)a.z)
                    : new Vector3i(axis, (int)a.x, (int)a.y);
                int start = axis == 0 ? (int)a.x : axis == 1 ? (int)a.y : (int)a.z;
                int end = axis == 0 ? (int)b.x : axis == 1 ? (int)b.y : (int)b.z;
                if (start > end)
                {
                    int coordinate = start;
                    start = end;
                    end = coordinate;
                }
                if (!lines.TryGetValue(key, out var spans))
                {
                    spans = new List<Vector2i>();
                    lines.Add(key, spans);
                }
                spans.Add(new Vector2i(start, end));
            }

            segments.Clear();
            foreach (var entry in lines)
            {
                var spans = entry.Value;
                spans.Sort((a, b) => a.x.CompareTo(b.x));
                int start = spans[0].x, end = spans[0].y;
                for (int i = 1; i < spans.Count; i++)
                {
                    if (spans[i].x <= end)
                    {
                        if (spans[i].y > end) end = spans[i].y;
                    }
                    else
                    {
                        segments.Add(new Edge(ToVector(ToPointOnLine(entry.Key, start)),
                            ToVector(ToPointOnLine(entry.Key, end))));
                        start = spans[i].x;
                        end = spans[i].y;
                    }
                }
                segments.Add(new Edge(ToVector(ToPointOnLine(entry.Key, start)),
                    ToVector(ToPointOnLine(entry.Key, end))));
            }
        }

        private static Vector3i ToPointOnLine(Vector3i line, int coordinate)
        {
            return line.x == 0 ? new Vector3i(coordinate, line.y, line.z)
                : line.x == 1 ? new Vector3i(line.y, coordinate, line.z)
                : new Vector3i(line.y, line.z, coordinate);
        }

        private static bool IsSolid(T voxel)
        {
            if (voxel is IVoxelSolidProvider solid) return solid.IsSolid();
            if (voxel is IVoxelDensityProvider density) return density.Density > 0;
            if (voxel is IVoxelTileMapIdProvider tile) return tile.TileId != 0;
            if (voxel is bool value) return value;
            return !EqualityComparer<T>.Default.Equals(voxel, default);
        }

        private static uint GetColor(T voxel)
        {
            if (voxel is IVoxelColorProvider color)
                return (uint)((color.R << 24) | (color.G << 16) | (color.B << 8) | color.A);
            return 0xFFFFFFFF;
        }

        private static uint GetMaterial(T voxel) => voxel is IVoxelMaterialIdProvider material ? material.MaterialId : 0;
    }
}
