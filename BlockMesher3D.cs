using g4;
using PiDev.VoxelCore;
using System;
using System.Collections.Generic;

namespace TileVox
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
        private struct CellData
        {
            public uint color;
            public uint material;
        }

        private struct FaceData
        {
            public int x, y, z, axis, sign;
            public uint color, material;
        }

        private struct EdgeData
        {
            public int faceCount;
            public bool top, bottom, sideAbove, sideBelow;
            public int wallMask;
        }

        public IVoxelDataAccess3D<T> data;
        public Range3D range;
        public BlockMeshingMode mode = BlockMeshingMode.QuadPerTile;
        public Func<T, bool> isSolid;

        /// <summary>Creates a mesher whose settings can be supplied before BuildMesh.</summary>
        public BlockMesher3D() { }

        /// <summary>Creates a mesher for a voxel source and inclusive output range.</summary>
        public BlockMesher3D(IVoxelDataAccess3D<T> data, Range3D range)
        {
            this.data = data;
            this.range = range;
        }

        /// <summary>Enumerates stored cells once and builds quads and boundary segments.</summary>
        public MesherResult3D BuildMesh()
        {
            var result = new MesherResult3D(true);
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
            var solid = isSolid ?? IsSolid;
            var cells = new Dictionary<(int x, int y, int z), CellData>();
            foreach (var sequence in source.EnumerateRange(halo, false))
                foreach (var cell in sequence)
                {
                    if (!halo.Contains(cell.x, cell.y, cell.z) || !solid(cell.value)) continue;
                    cells[(cell.x, cell.y, cell.z)] = new CellData
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
                        if (cells.ContainsKey((x, y, z))) continue;
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

        private static void BuildQuadsPerTile(MesherResult3D result, List<FaceData> faces)
        {
            foreach (var face in faces)
            {
                GetFacePlane(face, out int plane, out int u, out int v);
                AddQuad(result, face.axis, face.sign, plane, u, v, 1, 1, face.color, face.material);
            }
            BuildBoundaries(result, faces);
        }

        private static void BuildPlanarSimplified(MesherResult3D result, List<FaceData> faces)
        {
            var groups = new Dictionary<(int axis, int sign, int plane, uint color, uint material),
                (List<(int u, int v)> order, HashSet<(int u, int v)> cells)>();
            foreach (var face in faces)
            {
                GetFacePlane(face, out int plane, out int u, out int v);
                var key = (face.axis, face.sign, plane, face.color, face.material);
                if (!groups.TryGetValue(key, out var group))
                {
                    group = (new List<(int u, int v)>(), new HashSet<(int u, int v)>());
                    groups.Add(key, group);
                }
                group.order.Add((u, v));
                group.cells.Add((u, v));
            }

            foreach (var entry in groups)
            {
                var remaining = entry.Value.cells;
                foreach (var point in entry.Value.order)
                {
                    if (!remaining.Contains(point)) continue;
                    int width = 1, height = 1;
                    while (remaining.Contains((point.u + width, point.v))) width++;
                    bool extend = true;
                    while (extend)
                    {
                        for (int x = 0; x < width; x++)
                            if (!remaining.Contains((point.u + x, point.v + height)))
                            {
                                extend = false;
                                break;
                            }
                        if (extend) height++;
                    }
                    for (int y = 0; y < height; y++)
                        for (int x = 0; x < width; x++)
                            remaining.Remove((point.u + x, point.v + y));
                    var key = entry.Key;
                    AddQuad(result, key.axis, key.sign, key.plane, point.u, point.v,
                        width, height, key.color, key.material);
                }
            }
            BuildBoundaries(result, faces);
            MergeSegments(result.FloorBoundaries);
            MergeSegments(result.GroundBoundaries);
            MergeSegments(result.WallBoundaries);
        }

        private static void GetFacePlane(FaceData face, out int plane, out int u, out int v)
        {
            plane = (face.axis == 0 ? face.x : face.axis == 1 ? face.y : face.z) + (face.sign > 0 ? 1 : 0);
            u = face.axis == 0 ? face.y : face.axis == 1 ? face.z : face.x;
            v = face.axis == 0 ? face.z : face.axis == 1 ? face.x : face.y;
        }

        private static (int x, int y, int z) ToPoint(int axis, int plane, int u, int v)
        {
            return axis == 0 ? (plane, u, v) : axis == 1 ? (v, plane, u) : (u, v, plane);
        }

        private static Vector3d ToVector((int x, int y, int z) p) => new Vector3d(p.x, p.y, p.z);

        private static void AddQuad(MesherResult3D result, int axis, int sign, int plane,
            int u, int v, int width, int height, uint color, uint material)
        {
            Vector3d a = ToVector(ToPoint(axis, plane, u, v));
            Vector3d b = ToVector(ToPoint(axis, plane, u + width, v));
            Vector3d c = ToVector(ToPoint(axis, plane, u + width, v + height));
            Vector3d d = ToVector(ToPoint(axis, plane, u, v + height));
            if (sign > 0) result.MeshQuads.Add((a, b, c, d, color, material));
            else result.MeshQuads.Add((a, d, c, b, color, material));
        }

        private static void BuildBoundaries(MesherResult3D result, List<FaceData> faces)
        {
            var edges = new Dictionary<((int x, int y, int z) a, (int x, int y, int z) b), EdgeData>();
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
                var segment = (ToVector(entry.Key.a), ToVector(entry.Key.b));
                if (edge.top)
                {
                    if (edge.sideBelow || edge.wallMask == 0 && edge.faceCount == 1)
                        result.FloorBoundaries.Add(segment);
                    if (edge.sideAbove) result.GroundBoundaries.Add(segment);
                }
                else if (edge.wallMask != 0 &&
                    (edge.faceCount == 1 || edge.bottom || (edge.wallMask & (edge.wallMask - 1)) != 0))
                    result.WallBoundaries.Add(segment);
            }
        }

        private static void AddEdge(
            Dictionary<((int x, int y, int z) a, (int x, int y, int z) b), EdgeData> edges,
            (int x, int y, int z) a, (int x, int y, int z) b, FaceData face)
        {
            if (ComparePoints(a, b) > 0) (a, b) = (b, a);
            var key = (a, b);
            edges.TryGetValue(key, out EdgeData edge);
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

        private static int ComparePoints((int x, int y, int z) a, (int x, int y, int z) b)
        {
            int x = a.x.CompareTo(b.x);
            if (x != 0) return x;
            int y = a.y.CompareTo(b.y);
            return y != 0 ? y : a.z.CompareTo(b.z);
        }

        private static void MergeSegments(List<(Vector3d a, Vector3d b)> segments)
        {
            var lines = new Dictionary<(int axis, int a, int b), List<(int start, int end)>>();
            foreach (var segment in segments)
            {
                Vector3d a = segment.a, b = segment.b;
                int axis = a.x != b.x ? 0 : a.y != b.y ? 1 : 2;
                var key = axis == 0 ? (axis, (int)a.y, (int)a.z)
                    : axis == 1 ? (axis, (int)a.x, (int)a.z) : (axis, (int)a.x, (int)a.y);
                int start = axis == 0 ? (int)a.x : axis == 1 ? (int)a.y : (int)a.z;
                int end = axis == 0 ? (int)b.x : axis == 1 ? (int)b.y : (int)b.z;
                if (start > end) (start, end) = (end, start);
                if (!lines.TryGetValue(key, out var spans))
                {
                    spans = new List<(int start, int end)>();
                    lines.Add(key, spans);
                }
                spans.Add((start, end));
            }

            segments.Clear();
            foreach (var entry in lines)
            {
                var spans = entry.Value;
                spans.Sort((a, b) => a.start.CompareTo(b.start));
                int start = spans[0].start, end = spans[0].end;
                for (int i = 1; i < spans.Count; i++)
                {
                    if (spans[i].start <= end)
                    {
                        if (spans[i].end > end) end = spans[i].end;
                    }
                    else
                    {
                        segments.Add((ToVector(ToPointOnLine(entry.Key, start)),
                            ToVector(ToPointOnLine(entry.Key, end))));
                        start = spans[i].start;
                        end = spans[i].end;
                    }
                }
                segments.Add((ToVector(ToPointOnLine(entry.Key, start)),
                    ToVector(ToPointOnLine(entry.Key, end))));
            }
        }

        private static (int x, int y, int z) ToPointOnLine((int axis, int a, int b) line, int coordinate)
        {
            return line.axis == 0 ? (coordinate, line.a, line.b)
                : line.axis == 1 ? (line.a, coordinate, line.b) : (line.a, line.b, coordinate);
        }

        private static bool IsSolid(T voxel)
        {
            if (voxel is IVoxelDensityProvider density) return density.Density < 0;
            if (voxel is IVoxelTileMapIdProvider tile) return tile.TileId != 0;
            if (voxel is bool value) return value;
            return !EqualityComparer<T>.Default.Equals(voxel, default);
        }

        private static uint GetColor(T voxel)
        {
            if (voxel is IVoxelColorProvider color)
                return (uint)((color.R << 24) | (color.G << 16) | (color.B << 8) | color.A);
            if (voxel is IVoxelRgbProvider rgb)
                return (uint)((rgb.R << 24) | (rgb.G << 16) | (rgb.B << 8) | 255);
            return 0xFFFFFFFF;
        }

        private static uint GetMaterial(T voxel) => voxel is IVoxelMaterialIdProvider material ? material.MaterialId : 0;
    }
}
