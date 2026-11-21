using g4;
using PiDev.VoxelCore;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace TileVox.Meshing
{
    /// <summary>Builds an indexed isosurface from signed voxel densities.</summary>
    /// <remarks>Range contains inclusive cube origins. Density samples sit at voxel centers.
    /// Cube corners provide edge crossings; normals read neighboring source samples.
    /// Omitted cells use the accessor's default value. Supply the same default voxel
    /// to the mesher so candidate classification matches coordinate reads.
    /// Use a separate mesher instance for each concurrent build. The source must
    /// support concurrent reads if shared by multiple builds.</remarks>
    public class MarchingCubesMesher3D<T> where T : IVoxelDensityProvider
    {
        public IVoxelDataAccess3D<T> data;
        public Range3D range;
        public sbyte isoLevel;
        public TerrainBoundaryMode materialBoundaries;
        public TerrainBoundaryMode colorBoundaries;
        private T defaultVoxel;
        private bool hasDefaultVoxel;

        public MarchingCubesMesher3D() { }

        public MarchingCubesMesher3D(IVoxelDataAccess3D<T> data, Range3D range)
        {
            this.data = data;
            this.range = range;
        }

        /// <summary>Sets the implicit density for cells omitted by range enumeration.</summary>
        public MarchingCubesMesher3D(IVoxelDataAccess3D<T> data, Range3D range, T defaultVoxel) : this(data, range)
        {
            this.defaultVoxel = defaultVoxel;
            hasDefaultVoxel = true;
        }

        /// <summary>Generates terrain triangles, density normals, and any requested boundary segments.</summary>
        public TerrainMesherResult BuildMesh()
        {
            source = data;
            bounds = range;
            result = new TerrainMesherResult();
            materialMode = materialBoundaries;
            colorMode = colorBoundaries;
            splitMaterials = materialMode != TerrainBoundaryMode.None;
            splitColors = colorMode != TerrainBoundaryMode.None;
            if (bounds.minX > bounds.maxX || bounds.minY > bounds.maxY || bounds.minZ > bounds.maxZ)
            {
                result.Vertices = Array.Empty<TerrainVertex>();
                result.Triangles = Array.Empty<TerrainTriangle>();
                if (splitMaterials) result.MaterialBoundaries = Array.Empty<Edge>();
                if (splitColors) result.ColorBoundaries = Array.Empty<Edge>();
                return result;
            }
            cubicMaterial = materialMode == TerrainBoundaryMode.Cubic;
            cubicColor = colorMode == TerrainBoundaryMode.Cubic;
            cubicOnly = (cubicMaterial || cubicColor) && (!splitMaterials || cubicMaterial) && (!splitColors || cubicColor);
            sampleBounds = new Range3D(
                bounds.minX, bounds.maxX + 1,
                bounds.minY, bounds.maxY + 1,
                bounds.minZ, bounds.maxZ + 1);
            T implicitVoxel = default;
            implicitDensity = sbyte.MinValue;
            if (hasDefaultVoxel)
            {
                implicitVoxel = defaultVoxel;
                implicitDensity = implicitVoxel.Density;
            }
            else if (source is IChunk3D<T> chunk)
            {
                implicitVoxel = chunk.DefaultValue;
                implicitDensity = implicitVoxel.Density;
            }
            implicitMaterial = GetMaterial(implicitVoxel);
            implicitColor = GetColor(implicitVoxel);
            var cubes = new World3DFast<byte, Cube>(_ => new SparseChunk3DFast<byte, Cube>(16, default));
            int cubeCount = 0;
            bool implicitSolid = implicitDensity > isoLevel;

            long cellBuildTS = -1;
            long meshBuildTS = -1;
            var sw = new Stopwatch();
            sw.Start();
            try
            {

                // One source pass: scatter each voxel to its eight neighboring cubes.
                foreach (var sequence in source.EnumerateRange(sampleBounds, false))
                {
                    foreach (var cell in sequence)
                    {
                        //if (cell.x < sampleBounds.minX || cell.x > sampleBounds.maxX ||
                        //    cell.y < sampleBounds.minY || cell.y > sampleBounds.maxY ||
                        //    cell.z < sampleBounds.minZ || cell.z > sampleBounds.maxZ) continue;
                        if (!sampleBounds.Contains(cell.x, cell.y, cell.z)) continue;
                        sbyte density = cell.value.Density;

                        uint material = 0;
                        uint color = (uint)(object)-1;
                        //uint material = GetMaterial(cell.value);
                        //uint color = GetColor(cell.value);
                        bool opposite = (density > isoLevel) != implicitSolid;
                        int x = cell.x, y = cell.y, z = cell.z;

                        //if (x <= bounds.maxX && y <= bounds.maxY && z <= bounds.maxZ)
                            ScatterCorner(cubes, x, y, z, 0, density, material, color, opposite, ref cubeCount);
                        //if (x > bounds.minX && y <= bounds.maxY && z <= bounds.maxZ)
                            ScatterCorner(cubes, x - 1, y, z, 1, density, material, color, opposite, ref cubeCount);
                        //if (x > bounds.minX && y > bounds.minY && z <= bounds.maxZ)
                            ScatterCorner(cubes, x - 1, y - 1, z, 2, density, material, color, opposite, ref cubeCount);
                        //if (x <= bounds.maxX && y > bounds.minY && z <= bounds.maxZ)
                            ScatterCorner(cubes, x, y - 1, z, 3, density, material, color, opposite, ref cubeCount);
                        //if (x <= bounds.maxX && y <= bounds.maxY && z > bounds.minZ)
                            ScatterCorner(cubes, x, y, z - 1, 4, density, material, color, opposite, ref cubeCount);
                        //if (x > bounds.minX && y <= bounds.maxY && z > bounds.minZ)
                            ScatterCorner(cubes, x - 1, y, z - 1, 5, density, material, color, opposite, ref cubeCount);
                        //if (x > bounds.minX && y > bounds.minY && z > bounds.minZ)
                            ScatterCorner(cubes, x - 1, y - 1, z - 1, 6, density, material, color, opposite, ref cubeCount);
                        //if (x <= bounds.maxX && y > bounds.minY && z > bounds.minZ)
                            ScatterCorner(cubes, x, y - 1, z - 1, 7, density, material, color, opposite, ref cubeCount);
                    }
                }
                cellBuildTS = sw.ElapsedMilliseconds;
                sw.Stop(); sw.Reset();

                if (cubeCount == 0)
                {
                    result.Vertices = Array.Empty<TerrainVertex>();
                    result.Triangles = Array.Empty<TerrainTriangle>();
                    if (splitMaterials) result.MaterialBoundaries = Array.Empty<Edge>();
                    if (splitColors) result.ColorBoundaries = Array.Empty<Edge>();
                    return result;
                }

                int capacity = checked(cubeCount * 3);
                vertices = new List<TerrainVertex>(capacity);
                triangles = new List<TerrainTriangle>(capacity);
                if (splitMaterials) result.MaterialBoundaries = new List<Edge>(cubeCount);
                if (splitColors) result.ColorBoundaries = new List<Edge>(cubeCount);
                if (materialMode == TerrainBoundaryMode.Exact) materialSamples = new List<BoundarySample>(capacity);
                else materialSamples = null;
                if (colorMode == TerrainBoundaryMode.Exact) colorSamples = new List<BoundarySample>(capacity);
                else colorSamples = null;
                boundaryCapacity = capacity;
                edgeCutVertices = null;
                interiorCutVertices = null;
                regionVertices = null;
                regionEdges = null;
                edgeVertices = new Dictionary<(int x, int y, int z, int axis), int>(capacity);
                Span<int> crossings = stackalloc int[12];
                
                sw.Start();
                foreach (var sequence in cubes.EnumerateRange(bounds, exact: false))
                {
                    foreach (var cell in sequence)
                    {
                        Cube cubeData = cell.value;
                        int cube = implicitSolid ? 255 ^ cubeData.mask : cubeData.mask;
                        if (cube == 0 || cube == 255) continue;

                        int edges = edgeTable[cube];
                        for (int edge = 0; edge < 12; edge++)
                            if ((edges & (1 << edge)) != 0)
                                crossings[edge] = GetCrossing(edge, cell.x, cell.y, cell.z, ref cubeData);
                        if (!splitMaterials && !splitColors) BuildFast(cube, crossings);
                        else BuildWithBoundaries(cube, crossings, cell.x, cell.z, ref cubeData);
                    }
                }
                meshBuildTS = sw.ElapsedMilliseconds;
                sw.Stop();

                result.Vertices = vertices;
                result.Triangles = triangles;
                return result;
            }
            finally
            {
                UnityEngine.Debug.Log($"CELL time: {cellBuildTS}\nMESH time: {meshBuildTS}");
            }
        }

        /// <summary>Returns only the boundary sets requested for this build.</summary>
        public IEnumerable<MesherEdgeSet> GetEdgeSets(TerrainMesherResult result)
        {
            if (result.MaterialBoundaries != null)
                yield return new MesherEdgeSet { Name = nameof(result.MaterialBoundaries), Edges = () => result.MaterialBoundaries };
            if (result.ColorBoundaries != null)
                yield return new MesherEdgeSet { Name = nameof(result.ColorBoundaries), Edges = () => result.ColorBoundaries };
        }

        private struct CornerData
        {
            public sbyte density;
            public uint material, color;
        }

        private unsafe struct Cube
        {
            public byte mask, present;
            public fixed sbyte densities[8];
            public fixed uint materials[8];
            public fixed uint colors[8];

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void SetCorner(int index, sbyte density, uint material, uint color)
            {
                present |= (byte)(1 << index);
                densities[index] = density;
                materials[index] = material;
                colors[index] = color;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public CornerData GetCorner(int index, sbyte defaultDensity, uint defaultMaterial, uint defaultColor)
            {
                return (present & (1 << index)) == 0
                    ? new CornerData { density = defaultDensity, material = defaultMaterial, color = defaultColor }
                    : new CornerData { density = densities[index], material = materials[index], color = colors[index] };
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void ScatterCorner(World3DFast<byte, Cube> cubes,
            int x, int y, int z, int corner, sbyte density, uint material, uint color,
            bool opposite, ref int cubeCount)
        {
            Cube cube = cubes[x, y, z];
            if (opposite)
            {
                if (cube.mask == 0) cubeCount++;
                cube.mask |= (byte)(1 << corner);
            }
            cube.SetCorner(corner, density, material, color);
            cubes[x, y, z] = cube;
        }

        private struct TileLabels
        {
            public uint material, color;
        }

        private struct BoundarySample
        {
            public uint first, second;
            public double fraction;
        }

        private struct SurfaceField
        {
            public uint label;
            public Vector3d weights;
        }

        private struct SurfaceRegion
        {
            public uint material, color;
            public bool materialBoundary, colorBoundary;
        }

        private struct CubicGrid
        {
            // Each axis has a separate seam in each half of the other axis.
            public const int AcrossX = 3, AcrossZ = 12;
            public Vector3d x, z;
            public int seams;

            public int SeamsAt(Vector3d weights)
            {
                const double tolerance = 1e-9;
                double px = Vector3d.Dot(weights, x), pz = Vector3d.Dot(weights, z);
                int mask = 0;
                if (Math.Abs(px) <= tolerance)
                {
                    if (pz <= tolerance) mask |= 1;
                    if (pz >= -tolerance) mask |= 2;
                }
                if (Math.Abs(pz) <= tolerance)
                {
                    if (px <= tolerance) mask |= 4;
                    if (px >= -tolerance) mask |= 8;
                }
                return mask & seams;
            }
        }

        // Quantize barycentric keys so cuts from different clipping orders share vertices.
        private const double CutGrid = 1_000_000_000;
        // A triangle plus up to five rival fields per channel and two tile clipping planes.
        private const int RegionCapacity = 15;
        private const int CubicPolygonCapacity = 24;
        private IVoxelDataAccess3D<T> source;
        private Range3D bounds;
        private TerrainMesherResult result;
        private List<TerrainVertex> vertices;
        private List<TerrainTriangle> triangles;
        private List<BoundarySample> materialSamples, colorSamples;
        private Dictionary<(int a, int b, long step), int> edgeCutVertices;
        private Dictionary<(long u, long v), int> interiorCutVertices;
        private Dictionary<(int vertex, uint material, uint color), int> regionVertices;
        private Dictionary<(int a, int b), SurfaceRegion> regionEdges;
        private uint implicitMaterial, implicitColor;
        private sbyte implicitDensity;
        private Dictionary<(int x, int y, int z, int axis), int> edgeVertices;
        private bool splitMaterials, splitColors;
        private bool cubicMaterial, cubicColor, cubicOnly;
        private TerrainBoundaryMode materialMode, colorMode;
        private int boundaryCapacity;
        private Range3D sampleBounds;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private int GetCrossing(int edge, int x, int y, int z, ref Cube cubeData)
        {
            var start = edgeStarts[edge];
            int sx = x + start.x, sy = y + start.y, sz = z + start.z;
            var cacheKey = (sx, sy, sz, start.axis);
            if (edgeVertices.TryGetValue(cacheKey, out int cached)) return cached;

            int nx = sx + (start.axis == 0 ? 1 : 0);
            int ny = sy + (start.axis == 1 ? 1 : 0);
            int nz = sz + (start.axis == 2 ? 1 : 0);
            var corners = edgeCorners[edge];
            CornerData first = cubeData.GetCorner(corners.first, implicitDensity, implicitMaterial, implicitColor);
            CornerData second = cubeData.GetCorner(corners.second, implicitDensity, implicitMaterial, implicitColor);
            sbyte firstDensity = first.density, secondDensity = second.density;
            bool firstSolid = firstDensity > isoLevel;
            double t = (isoLevel - firstDensity) / (double)(secondDensity - firstDensity);
            var position = new Vector3d(x + 0.5 + start.x + (start.axis == 0 ? t : 0),
                y + 0.5 + start.y + (start.axis == 1 ? t : 0),
                z + 0.5 + start.z + (start.axis == 2 ? t : 0));

            var normal = Vector3f.Lerp(Gradient(sx, sy, sz), Gradient(nx, ny, nz), (float)t);
            if (normal.LengthSquared == 0)
            {
                float sign = firstSolid ? 1 : -1;
                normal = start.axis == 0 ? new Vector3f(sign, 0, 0)
                    : start.axis == 1 ? new Vector3f(0, sign, 0) : new Vector3f(0, 0, sign);
            }

            CornerData solid = firstSolid ? first : second;
            CornerData air = firstSolid ? second : first;
            TileLabels tile = default;
            if (cubicMaterial || cubicColor)
            {
                int tileX = sx + (start.axis == 0 && t > 0.5 ? 1 : 0);
                int tileZ = sz + (start.axis == 2 && t > 0.5 ? 1 : 0);
                tile = GetTileLabels(ref cubeData, tileX - x, tileZ - z);
            }
            uint material = cubicMaterial ? tile.material : solid.material;
            uint solidColor = cubicColor ? tile.color : solid.color;
            uint color = solidColor;
            uint airColor = 0;
            if (!splitColors || colorSamples != null)
                airColor = air.color;
            if (!splitColors)
                color = firstSolid ? MixColor(solidColor, airColor, (float)t) : MixColor(airColor, solidColor, (float)t);

            int id = AddVertex(new TerrainVertex(position, normal.Normalized, color, material));
            if (materialSamples != null)
            {
                uint airMaterial = air.material;
                materialSamples[id] = new BoundarySample
                {
                    first = firstSolid ? material : airMaterial,
                    second = firstSolid ? airMaterial : material,
                    fraction = t
                };
            }
            if (colorSamples != null)
            {
                colorSamples[id] = new BoundarySample
                {
                    first = firstSolid ? solidColor : airColor,
                    second = firstSolid ? airColor : solidColor,
                    fraction = t
                };
            }
            edgeVertices.Add(cacheKey, id);
            return id;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private Vector3f Gradient(int x, int y, int z)
        {
            return new Vector3f(source[x - 1, y, z].Density - source[x + 1, y, z].Density,
                source[x, y - 1, z].Density - source[x, y + 1, z].Density,
                source[x, y, z - 1].Density - source[x, y, z + 1].Density);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void BuildFast(int cube, ReadOnlySpan<int> crossings)
        {
            int end = (cube + 1) * 16;
            for (int i = cube * 16; i < end && triangleTable[i] >= 0; i += 3)
            {
                int a = crossings[triangleTable[i]], b = crossings[triangleTable[i + 1]], c = crossings[triangleTable[i + 2]];
                if (HasArea(a, b, c)) AddTriangle(a, b, c, DominantMaterial(a, b, c));
            }
        }

        private void BuildWithBoundaries(int cube, ReadOnlySpan<int> crossings, int worldX, int worldZ, ref Cube cubeData)
        {
            Span<TileLabels> tiles = stackalloc TileLabels[4];
            int seams = 0;
            if (cubicMaterial || cubicColor)
            {
                tiles[0] = GetTileLabels(ref cubeData, 0, 0);
                tiles[1] = GetTileLabels(ref cubeData, 0, 1);
                tiles[2] = GetTileLabels(ref cubeData, 1, 0);
                tiles[3] = GetTileLabels(ref cubeData, 1, 1);
                if (!SameTile(tiles[0], tiles[2])) seams |= 1;
                if (!SameTile(tiles[1], tiles[3])) seams |= 2;
                if (!SameTile(tiles[0], tiles[1])) seams |= 4;
                if (!SameTile(tiles[2], tiles[3])) seams |= 8;
            }

            int end = (cube + 1) * 16;
            bool uniformCubic = cubicOnly && seams == 0;
            for (int i = cube * 16; i < end && triangleTable[i] >= 0; i += 3)
            {
                int a = crossings[triangleTable[i]], b = crossings[triangleTable[i + 1]], c = crossings[triangleTable[i + 2]];
                if (!HasArea(a, b, c)) continue;
                if (uniformCubic)
                {
                    uint material = cubicMaterial ? tiles[0].material : DominantMaterial(a, b, c);
                    AddTriangle(a, b, c, material);
                }
                else SplitSurfaceTriangle(a, b, c, worldX, worldZ, tiles, seams);
            }
        }

        private static bool SameTile(TileLabels first, TileLabels second) =>
            first.material == second.material && first.color == second.color;

        private void SplitSurfaceTriangle(int a, int b, int c, int worldX, int worldZ,
            ReadOnlySpan<TileLabels> tiles, int seams)
        {
            if (edgeCutVertices == null)
            {
                edgeCutVertices = new(boundaryCapacity);
                interiorCutVertices = new(boundaryCapacity);
                regionVertices = new(boundaryCapacity);
                regionEdges = new(boundaryCapacity);
            }
            // Three crossings can each contribute two distinct voxel labels in Exact mode.
            Span<SurfaceField> materials = stackalloc SurfaceField[6];
            Span<SurfaceField> colors = stackalloc SurfaceField[6];
            int materialCount = GetFields(a, b, c, materialMode, materialSamples, true, materials);
            int colorCount = GetFields(a, b, c, colorMode, colorSamples, false, colors);
            if (cubicMaterial) materials[0].label = tiles[0].material;
            if (cubicColor) colors[0].label = tiles[0].color;
            if (seams == 0 && materialCount == 1 && colorCount == 1)
            {
                // A single label owns both ends of each edge in each active field.
                // An adjacent triangle shares those samples and cannot cross a seam there.
                uint material = materials[0].label, color = colors[0].label;
                AddTriangle(RegionVertex(a, material, color), RegionVertex(b, material, color),
                    RegionVertex(c, material, color), material);
                return;
            }
            materials = materials.Slice(0, materialCount);
            colors = colors.Slice(0, colorCount);

            interiorCutVertices.Clear();
            Span<Vector3d> triangle = stackalloc Vector3d[3];
            triangle[0] = new Vector3d(1, 0, 0);
            triangle[1] = new Vector3d(0, 1, 0);
            triangle[2] = new Vector3d(0, 0, 1);

            CubicGrid grid = default;
            if (seams != 0)
            {
                var first = vertices[a].position;
                var second = vertices[b].position;
                var third = vertices[c].position;
                double midX = worldX + 1.0, midZ = worldZ + 1.0;
                grid.x = new Vector3d(first.x - midX, second.x - midX, third.x - midX);
                grid.z = new Vector3d(first.z - midZ, second.z - midZ, third.z - midZ);
                grid.seams = seams;
            }
            SplitRegions(a, b, c, triangle, materials, colors, grid, tiles);
        }

        private void EmitCubicRegion(int a, int b, int c, ReadOnlySpan<Vector3d> input,
            uint material, uint color, CubicGrid grid, ReadOnlySpan<TileLabels> tiles)
        {
            Span<Vector3d> polygons = stackalloc Vector3d[4 * RegionCapacity];
            Span<int> counts = stackalloc int[4];
            counts.Clear();
            Span<Vector3d> column = stackalloc Vector3d[RegionCapacity];
            bool splitX = (grid.seams & CubicGrid.AcrossX) != 0;
            bool splitZ = (grid.seams & CubicGrid.AcrossZ) != 0;
            int xCount = splitX && grid.x.LengthSquared != 0 ? 2 : 1;
            int zCount = splitZ && grid.z.LengthSquared != 0 ? 2 : 1;
            for (int dx = 0; dx < xCount; dx++)
            {
                ReadOnlySpan<Vector3d> xPolygon = column;
                if (xCount == 2)
                {
                    int count = ClipPolygon(input, column, dx == 0 ? -grid.x : grid.x);
                    if (count < 3) continue;
                    xPolygon = column.Slice(0, count);
                }
                else xPolygon = input;
                for (int dz = 0; dz < zCount; dz++)
                {
                    int tile = 2 * dx + dz;
                    var polygon = polygons.Slice(tile * RegionCapacity, RegionCapacity);
                    int count = xPolygon.Length;
                    if (zCount == 2)
                        count = ClipPolygon(xPolygon, polygon, dz == 0 ? -grid.z : grid.z);
                    else
                        xPolygon.CopyTo(polygon);
                    if (count >= 3 && PolygonArea(polygon.Slice(0, count)) > 1e-14)
                        counts[tile] = count;
                }
            }

            // Join pieces across deleted grid edges before triangulating. A label occupying
            // three quadrants can leave a concave polygon, while diagonals stay separate.
            Span<Vector3d> joined = stackalloc Vector3d[CubicPolygonCapacity];
            Span<Vector3d> scratch = stackalloc Vector3d[CubicPolygonCapacity];
            int pending = 0;
            for (int i = 0; i < 4; i++)
                if (counts[i] != 0) pending |= 1 << i;
            for (int tile = 0; tile < 4; tile++)
            {
                if ((pending & (1 << tile)) == 0) continue;
                int members = 1 << tile;
                int count = counts[tile];
                polygons.Slice(tile * RegionCapacity, count).CopyTo(joined);
                bool merged;
                do
                {
                    merged = false;
                    for (int next = tile + 1; next < 4; next++)
                    {
                        int bit = 1 << next;
                        if (((pending & ~members) & bit) == 0 || !SameTile(tiles[tile], tiles[next])) continue;
                        int size = JoinTilePolygons(joined.Slice(0, count),
                            polygons.Slice(next * RegionCapacity, counts[next]), scratch);
                        if (size == 0) continue;
                        scratch.Slice(0, size).CopyTo(joined);
                        count = size;
                        members |= bit;
                        merged = true;
                    }
                } while (merged);

                uint tileMaterial = cubicMaterial ? tiles[tile].material : material;
                uint tileColor = cubicColor ? tiles[tile].color : color;
                count = RemoveInactiveTileCorners(joined, count, grid);
                if (!EmitRegion(a, b, c, joined.Slice(0, count), tileMaterial, tileColor, grid))
                {
                    // Keep the original convex pieces if lattice snapping collapses a merged corner.
                    for (int i = tile; i < 4; i++)
                        if ((members & (1 << i)) != 0)
                            EmitRegion(a, b, c, polygons.Slice(i * RegionCapacity, counts[i]), tileMaterial, tileColor, grid, true);
                }
                pending &= ~members;
            }
        }

        private static int JoinTilePolygons(ReadOnlySpan<Vector3d> first, ReadOnlySpan<Vector3d> second,
            Span<Vector3d> output)
        {
            for (int i = 0; i < first.Length; i++)
            {
                int next = (i + 1) % first.Length;
                if ((first[i] - first[next]).LengthSquared < 1e-20) continue;
                for (int j = 0; j < second.Length; j++)
                {
                    if ((first[next] - second[j]).LengthSquared > 1e-20 ||
                        (first[i] - second[(j + 1) % second.Length]).LengthSquared > 1e-20) continue;
                    int size = first.Length + second.Length - 2;
                    if (size > output.Length) return 0;
                    for (int k = 0; k < first.Length; k++) output[k] = first[(next + k) % first.Length];
                    for (int k = 2; k < second.Length; k++) output[first.Length + k - 2] = second[(j + k) % second.Length];
                    return size;
                }
            }
            return 0;
        }

        private static int RemoveInactiveTileCorners(Span<Vector3d> polygon, int count, CubicGrid grid)
        {
            for (int i = count - 1; i >= 0 && count > 3; i--)
            {
                Vector3d before = polygon[(i + count - 1) % count];
                Vector3d after = polygon[(i + 1) % count];
                if (grid.SeamsAt(polygon[i]) != 0 ||
                    Math.Abs(OrientedArea(before, polygon[i], after)) > 1e-14) continue;
                polygon.Slice(i + 1, count - i - 1).CopyTo(polygon.Slice(i));
                count--;
            }
            return count;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private TileLabels GetTileLabels(ref Cube cubeData, int x, int z)
        {
            // Cubic labels come from this cube's solid samples; no separate column cache.
            int bottom = x == 0 ? (z == 0 ? 0 : 4) : (z == 0 ? 1 : 5);
            int top = x == 0 ? (z == 0 ? 3 : 7) : (z == 0 ? 2 : 6);
            CornerData low = cubeData.GetCorner(bottom, implicitDensity, implicitMaterial, implicitColor);
            CornerData high = cubeData.GetCorner(top, implicitDensity, implicitMaterial, implicitColor);
            CornerData selected = high.density > isoLevel || low.density <= isoLevel && high.density >= low.density
                ? high : low;
            return new TileLabels { material = selected.material, color = selected.color };
        }

        private void SplitRegions(int a, int b, int c, ReadOnlySpan<Vector3d> input,
            ReadOnlySpan<SurfaceField> materials, ReadOnlySpan<SurfaceField> colors,
            CubicGrid grid, ReadOnlySpan<TileLabels> tiles)
        {
            if (materials.Length == 1 && colors.Length == 1)
            {
                EmitSurfaceRegion(a, b, c, input, materials[0].label, colors[0].label, grid, tiles);
                return;
            }

            // Clip in the triangle's barycentric coordinates. The winner has the largest
            // interpolated field. A label indicator of 0 or 1 is equivalent to a signed
            // mask of -1 or +1 when comparing two fields.
            Span<Vector3d> materialPolygon = stackalloc Vector3d[RegionCapacity];
            Span<Vector3d> region = stackalloc Vector3d[RegionCapacity];
            Span<Vector3d> scratch = stackalloc Vector3d[RegionCapacity];
            for (int material = 0; material < materials.Length; material++)
            {
                input.CopyTo(materialPolygon);
                int count = ClipToWinner(materials, material, materialPolygon, scratch, input.Length);
                if (count < 3) continue;
                if (colors.Length == 1)
                {
                    EmitSurfaceRegion(a, b, c, materialPolygon.Slice(0, count),
                        materials[material].label, colors[0].label, grid, tiles);
                    continue;
                }

                for (int color = 0; color < colors.Length; color++)
                {
                    materialPolygon.Slice(0, count).CopyTo(region);
                    int size = ClipToWinner(colors, color, region, scratch, count);
                    if (size >= 3)
                        EmitSurfaceRegion(a, b, c, region.Slice(0, size),
                            materials[material].label, colors[color].label, grid, tiles);
                }
            }
        }

        private void EmitSurfaceRegion(int a, int b, int c, ReadOnlySpan<Vector3d> polygon,
            uint material, uint color, CubicGrid grid, ReadOnlySpan<TileLabels> tiles)
        {
            if (grid.seams != 0) EmitCubicRegion(a, b, c, polygon, material, color, grid, tiles);
            else EmitRegion(a, b, c, polygon, material, color);
        }

        private int GetFields(int a, int b, int c, TerrainBoundaryMode mode,
            List<BoundarySample> samples, bool material, Span<SurfaceField> fields)
        {
            int count = 0;
            if (mode == TerrainBoundaryMode.Exact)
            {
                AddBoundarySample(fields, ref count, samples[a], 0);
                AddBoundarySample(fields, ref count, samples[b], 1);
                AddBoundarySample(fields, ref count, samples[c], 2);
            }
            else if (mode == TerrainBoundaryMode.Fast)
            {
                AddFieldWeight(fields, ref count, material ? vertices[a].material : vertices[a].color, 0, 1);
                AddFieldWeight(fields, ref count, material ? vertices[b].material : vertices[b].color, 1, 1);
                AddFieldWeight(fields, ref count, material ? vertices[c].material : vertices[c].color, 2, 1);
            }
            else
            {
                uint label = material && mode == TerrainBoundaryMode.None ? DominantMaterial(a, b, c) : 0;
                fields[count++] = new SurfaceField { label = label, weights = Vector3d.One };
            }
            return count;
        }

        private static void AddBoundarySample(Span<SurfaceField> fields, ref int count, BoundarySample sample, int corner)
        {
            if (sample.first == sample.second)
            {
                AddFieldWeight(fields, ref count, sample.first, corner, 1);
                return;
            }
            AddFieldWeight(fields, ref count, sample.first, corner, 1 - sample.fraction);
            AddFieldWeight(fields, ref count, sample.second, corner, sample.fraction);
        }

        private static void AddFieldWeight(Span<SurfaceField> fields, ref int count,
            uint label, int corner, double weight)
        {
            if (weight == 0) return;
            int index = 0;
            while (index < count && fields[index].label != label) index++;
            if (index == count) fields[count++] = new SurfaceField { label = label };
            fields[index].weights[corner] = weight;
        }

        private static int ClipToWinner(ReadOnlySpan<SurfaceField> fields, int index,
            Span<Vector3d> polygon, Span<Vector3d> scratch, int count)
        {
            SurfaceField winner = fields[index];
            for (int rival = 0; rival < fields.Length; rival++)
            {
                if (rival == index) continue;
                Vector3d difference = winner.weights - fields[rival].weights;
                if (difference.LengthSquared < 1e-24)
                {
                    // A full-face tie needs one owner, including when the neighbor order differs.
                    if (winner.label > fields[rival].label) return 0;
                    continue;
                }
                count = ClipPolygon(polygon.Slice(0, count), scratch, difference);
                if (count < 3) return 0;
                scratch.Slice(0, count).CopyTo(polygon);
            }
            return count;
        }

        private static int ClipPolygon(ReadOnlySpan<Vector3d> input, Span<Vector3d> output, Vector3d difference)
        {
            int size = 0;
            Vector3d previous = input[input.Length - 1];
            double previousWeight = Vector3d.Dot(previous, difference);
            foreach (Vector3d current in input)
            {
                double currentWeight = Vector3d.Dot(current, difference);
                if ((previousWeight < 0 && currentWeight > 0) || (previousWeight > 0 && currentWeight < 0))
                {
                    double t = previousWeight / (previousWeight - currentWeight);
                    output[size++] = previous * (1 - t) + current * t;
                }
                if (currentWeight >= 0) output[size++] = current;
                previous = current;
                previousWeight = currentWeight;
            }
            return size;
        }

        private static double OrientedArea(Vector3d a, Vector3d b, Vector3d c)
        {
            Vector3d u = b - a, v = c - a;
            return u.x * v.y - u.y * v.x;
        }

        private static double PolygonArea(ReadOnlySpan<Vector3d> polygon)
        {
            double area = 0;
            for (int i = 1; i + 1 < polygon.Length; i++)
                area += OrientedArea(polygon[0], polygon[i], polygon[i + 1]);
            return area;
        }

        private bool EmitRegion(int a, int b, int c, ReadOnlySpan<Vector3d> polygon,
            uint material, uint color, CubicGrid grid = default, bool convex = false)
        {
            if (PolygonArea(polygon) < 1e-14) return true;

            Span<int> indices = stackalloc int[CubicPolygonCapacity];
            Span<Vector3d> corners = stackalloc Vector3d[CubicPolygonCapacity];
            int count = 0;
            foreach (Vector3d weights in polygon)
            {
                int vertex = CutVertex(a, b, c, weights);
                if (count > 0 && indices[count - 1] == vertex) continue;
                indices[count] = vertex;
                corners[count++] = weights;
            }
            if (count > 1 && indices[count - 1] == indices[0]) count--;
            if (count < 3) return true;

            int fan = FindFan(corners.Slice(0, count), indices.Slice(0, count), grid.seams != 0);
            Span<int> ears = stackalloc int[(CubicPolygonCapacity - 2) * 3];
            bool useEars = fan < 0 && grid.seams != 0 && !convex;
            if (useEars && !TriangulateEars(corners.Slice(0, count), indices.Slice(0, count), ears))
                return false;

            // Cubic seams already identify the only edges that can change labels.
            bool trackAllEdges = !cubicOnly;
            for (int i = 0; i < count; i++)
            {
                int next = (i + 1) % count;
                if (trackAllEdges || (grid.SeamsAt(corners[i]) & grid.SeamsAt(corners[next])) != 0)
                    AddRegionEdge(indices[i], indices[next], material, color);
            }
            for (int i = 0; i < count; i++)
                indices[i] = RegionVertex(indices[i], material, color);

            if (fan >= 0)
            {
                for (int i = 1; i + 1 < count; i++)
                    AddTriangle(indices[fan], indices[(fan + i) % count], indices[(fan + i + 1) % count], material);
                return true;
            }
            if (useEars)
            {
                for (int i = 0; i < (count - 2) * 3; i += 3)
                    AddTriangle(indices[ears[i]], indices[ears[i + 1]], indices[ears[i + 2]], material);
                return true;
            }

            // Preserve collinear boundary points when a convex polygon needs an interior fan.
            Vector3d center = Vector3d.Zero;
            foreach (Vector3d weights in polygon) center += weights;
            int middle = RegionVertex(CutVertex(a, b, c, center / polygon.Length), material, color);
            for (int i = 0; i < count; i++)
            {
                int next = indices[(i + 1) % count];
                if (HasArea(middle, indices[i], next)) AddTriangle(middle, indices[i], next, material);
            }
            return true;
        }

        private int FindFan(ReadOnlySpan<Vector3d> corners, ReadOnlySpan<int> indices, bool checkWinding)
        {
            int count = indices.Length;
            for (int first = 0; first < count; first++)
            {
                bool valid = true;
                for (int i = 1; i + 1 < count; i++)
                {
                    int second = (first + i) % count, third = (first + i + 1) % count;
                    if ((!checkWinding || OrientedArea(corners[first], corners[second], corners[third]) > 1e-14) &&
                        HasArea(indices[first], indices[second], indices[third])) continue;
                    valid = false;
                    break;
                }
                if (valid) return first;
            }
            return -1;
        }

        private bool TriangulateEars(ReadOnlySpan<Vector3d> corners, ReadOnlySpan<int> indices, Span<int> triangles)
        {
            Span<int> ring = stackalloc int[CubicPolygonCapacity];
            int count = corners.Length, offset = 0;
            for (int i = 0; i < count; i++) ring[i] = i;
            while (count > 3)
            {
                bool found = false;
                for (int i = 0; i < count; i++)
                {
                    int a = ring[(i + count - 1) % count], b = ring[i], c = ring[(i + 1) % count];
                    if (OrientedArea(corners[a], corners[b], corners[c]) <= 1e-14 ||
                        !HasArea(indices[a], indices[b], indices[c])) continue;
                    bool blocked = false;
                    for (int j = 0; j < count; j++)
                    {
                        int check = ring[j];
                        if (check == a || check == b || check == c) continue;
                        Vector3d p = corners[check];
                        if (OrientedArea(corners[a], corners[b], p) < -1e-14 ||
                            OrientedArea(corners[b], corners[c], p) < -1e-14 ||
                            OrientedArea(corners[c], corners[a], p) < -1e-14) continue;
                        blocked = true;
                        break;
                    }
                    if (blocked) continue;
                    triangles[offset++] = a;
                    triangles[offset++] = b;
                    triangles[offset++] = c;
                    ring.Slice(i + 1, count - i - 1).CopyTo(ring.Slice(i));
                    count--;
                    found = true;
                    break;
                }
                if (!found) return false;
            }
            if (!HasArea(indices[ring[0]], indices[ring[1]], indices[ring[2]])) return false;
            ring.Slice(0, 3).CopyTo(triangles.Slice(offset));
            return true;
        }

        private int CutVertex(int a, int b, int c, Vector3d weights)
        {
            long u = (long)Math.Round(weights.x * CutGrid);
            long v = (long)Math.Round(weights.y * CutGrid);
            long w = (long)Math.Round(weights.z * CutGrid);
            if (u == 0 && v == 0) return c;
            if (v == 0 && w == 0) return a;
            if (w == 0 && u == 0) return b;
            if (u == 0) return EdgeCutVertex(b, c, weights.z / (weights.y + weights.z));
            if (v == 0) return EdgeCutVertex(a, c, weights.z / (weights.x + weights.z));
            if (w == 0 || u + v >= CutGrid) return EdgeCutVertex(a, b, weights.y / (weights.x + weights.y));

            var key = (u, v);
            if (interiorCutVertices.TryGetValue(key, out int id)) return id;
            double x = u / CutGrid, y = v / CutGrid, z = 1 - x - y;
            TerrainVertex first = vertices[a], second = vertices[b], third = vertices[c];
            var position = first.position * x + second.position * y + third.position * z;
            var normal = (first.normal * (float)x + second.normal * (float)y + third.normal * (float)z).Normalized;
            uint color = splitColors ? first.color : MixColor(first.color, second.color, third.color, x, y, z);
            uint material = x >= y && x >= z ? first.material : y >= z ? second.material : third.material;
            id = AddVertex(new TerrainVertex(position, normal, color, material));
            interiorCutVertices.Add(key, id);
            return id;
        }

        private int EdgeCutVertex(int a, int b, double fraction)
        {
            // Canonical edge order makes neighboring triangles share each cut vertex.
            if (b < a)
            {
                (a, b) = (b, a);
                fraction = 1 - fraction;
            }
            long step = (long)Math.Round(fraction * CutGrid);
            if (step <= 0) return a;
            if (step >= CutGrid) return b;
            var key = (a, b, step);
            if (edgeCutVertices.TryGetValue(key, out int id)) return id;
            double t = step / CutGrid;
            TerrainVertex first = vertices[a], second = vertices[b];
            var position = first.position * (1 - t) + second.position * t;
            var normal = Vector3f.Lerp(first.normal, second.normal, (float)t).Normalized;
            uint color = splitColors ? first.color : MixColor(first.color, second.color, (float)t);
            uint material = t <= 0.5 ? first.material : second.material;
            id = AddVertex(new TerrainVertex(position, normal, color, material));
            edgeCutVertices.Add(key, id);
            return id;
        }

        private int RegionVertex(int vertex, uint material, uint color)
        {
            var value = vertices[vertex];
            if (!splitMaterials) material = value.material;
            if (!splitColors) color = value.color;
            if (value.material == material && value.color == color) return vertex;
            var key = (vertex, material, color);
            if (regionVertices.TryGetValue(key, out int id)) return id;
            value.material = material;
            value.color = color;
            id = AddVertex(value);
            regionVertices.Add(key, id);
            return id;
        }

        private void AddRegionEdge(int a, int b, uint material, uint color)
        {
            if (b < a) (a, b) = (b, a);
            var key = (a, b);
            if (!regionEdges.TryGetValue(key, out SurfaceRegion neighbor))
            {
                regionEdges.Add(key, new SurfaceRegion { material = material, color = color });
                return;
            }

            bool materialBoundary = splitMaterials && !neighbor.materialBoundary && neighbor.material != material;
            bool colorBoundary = splitColors && !neighbor.colorBoundary && neighbor.color != color;
            if (!materialBoundary && !colorBoundary) return;

            var edge = new Edge(vertices[a].position, vertices[b].position);
            if (materialBoundary) result.MaterialBoundaries.Add(edge);
            if (colorBoundary) result.ColorBoundaries.Add(edge);
            neighbor.materialBoundary |= materialBoundary;
            neighbor.colorBoundary |= colorBoundary;
            regionEdges[key] = neighbor;
        }

        private uint DominantMaterial(int a, int b, int c)
        {
            uint first = vertices[a].material, second = vertices[b].material, third = vertices[c].material;
            if (first == second || first == third) return first;
            if (second == third) return second;
            return Math.Min(first, Math.Min(second, third));
        }

        private bool HasArea(int a, int b, int c)
        {
            var origin = vertices[a].position;
            return Vector3d.Cross(vertices[b].position - origin, vertices[c].position - origin).LengthSquared > 1e-20;
        }

        // The legacy lookup table winds case 1 toward its solid corner. Density
        // normals point from solid toward air, so reverse every emitted triangle.
        private void AddTriangle(int a, int b, int c, uint material) =>
            triangles.Add(new TerrainTriangle(a, c, b, material));

        private int AddVertex(TerrainVertex vertex)
        {
            int id = vertices.Count;
            vertices.Add(vertex);
            materialSamples?.Add(default);
            colorSamples?.Add(default);
            return id;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static uint GetMaterial(T value) => value is IVoxelMaterialIdProvider material ? material.MaterialId : 0;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static uint GetColor(T value) => value is IVoxelColorProvider color
            ? ((uint)color.R << 24) | ((uint)color.G << 16) | ((uint)color.B << 8) | color.A
            : uint.MaxValue;

        private static uint MixColor(uint a, uint b, float t)
        {
            if (a == b) return a;
            uint color = 0;
            for (int shift = 0; shift < 32; shift += 8)
            {
                int first = (byte)(a >> shift), second = (byte)(b >> shift);
                color |= (uint)(first + (second - first) * t + 0.5f) << shift;
            }
            return color;
        }

        private static uint MixColor(uint a, uint b, uint c, double x, double y, double z)
        {
            if (a == b && b == c) return a;
            uint color = 0;
            for (int shift = 0; shift < 32; shift += 8)
                color |= (uint)((byte)(a >> shift) * x + (byte)(b >> shift) * y + (byte)(c >> shift) * z + 0.5) << shift;
            return color;
        }

        // Standard cube corner order with positive-axis edge origins for shared edge caching.
        private static readonly (int x, int y, int z, int axis)[] edgeStarts =
        {
            (0, 0, 0, 0), (1, 0, 0, 1), (0, 1, 0, 0), (0, 0, 0, 1),
            (0, 0, 1, 0), (1, 0, 1, 1), (0, 1, 1, 0), (0, 0, 1, 1),
            (0, 0, 0, 2), (1, 0, 0, 2), (1, 1, 0, 2), (0, 1, 0, 2)
        };

        private static readonly (int first, int second)[] edgeCorners =
        {
            (0, 1), (1, 2), (3, 2), (0, 3),
            (4, 5), (5, 6), (7, 6), (4, 7),
            (0, 4), (1, 5), (2, 6), (3, 7)
        };

        // Cube cases and winding are carried over from legacy TileVox MarchingCubes.
        private static readonly int[] edgeTable =
        {
            0x0  , 0x109, 0x203, 0x30a, 0x406, 0x50f, 0x605, 0x70c,
            0x80c, 0x905, 0xa0f, 0xb06, 0xc0a, 0xd03, 0xe09, 0xf00,
            0x190, 0x99 , 0x393, 0x29a, 0x596, 0x49f, 0x795, 0x69c,
            0x99c, 0x895, 0xb9f, 0xa96, 0xd9a, 0xc93, 0xf99, 0xe90,
            0x230, 0x339, 0x33 , 0x13a, 0x636, 0x73f, 0x435, 0x53c,
            0xa3c, 0xb35, 0x83f, 0x936, 0xe3a, 0xf33, 0xc39, 0xd30,
            0x3a0, 0x2a9, 0x1a3, 0xaa , 0x7a6, 0x6af, 0x5a5, 0x4ac,
            0xbac, 0xaa5, 0x9af, 0x8a6, 0xfaa, 0xea3, 0xda9, 0xca0,
            0x460, 0x569, 0x663, 0x76a, 0x66 , 0x16f, 0x265, 0x36c,
            0xc6c, 0xd65, 0xe6f, 0xf66, 0x86a, 0x963, 0xa69, 0xb60,
            0x5f0, 0x4f9, 0x7f3, 0x6fa, 0x1f6, 0xff , 0x3f5, 0x2fc,
            0xdfc, 0xcf5, 0xfff, 0xef6, 0x9fa, 0x8f3, 0xbf9, 0xaf0,
            0x650, 0x759, 0x453, 0x55a, 0x256, 0x35f, 0x55 , 0x15c,
            0xe5c, 0xf55, 0xc5f, 0xd56, 0xa5a, 0xb53, 0x859, 0x950,
            0x7c0, 0x6c9, 0x5c3, 0x4ca, 0x3c6, 0x2cf, 0x1c5, 0xcc ,
            0xfcc, 0xec5, 0xdcf, 0xcc6, 0xbca, 0xac3, 0x9c9, 0x8c0,
            0x8c0, 0x9c9, 0xac3, 0xbca, 0xcc6, 0xdcf, 0xec5, 0xfcc,
            0xcc , 0x1c5, 0x2cf, 0x3c6, 0x4ca, 0x5c3, 0x6c9, 0x7c0,
            0x950, 0x859, 0xb53, 0xa5a, 0xd56, 0xc5f, 0xf55, 0xe5c,
            0x15c, 0x55 , 0x35f, 0x256, 0x55a, 0x453, 0x759, 0x650,
            0xaf0, 0xbf9, 0x8f3, 0x9fa, 0xef6, 0xfff, 0xcf5, 0xdfc,
            0x2fc, 0x3f5, 0xff , 0x1f6, 0x6fa, 0x7f3, 0x4f9, 0x5f0,
            0xb60, 0xa69, 0x963, 0x86a, 0xf66, 0xe6f, 0xd65, 0xc6c,
            0x36c, 0x265, 0x16f, 0x66 , 0x76a, 0x663, 0x569, 0x460,
            0xca0, 0xda9, 0xea3, 0xfaa, 0x8a6, 0x9af, 0xaa5, 0xbac,
            0x4ac, 0x5a5, 0x6af, 0x7a6, 0xaa , 0x1a3, 0x2a9, 0x3a0,
            0xd30, 0xc39, 0xf33, 0xe3a, 0x936, 0x83f, 0xb35, 0xa3c,
            0x53c, 0x435, 0x73f, 0x636, 0x13a, 0x33 , 0x339, 0x230,
            0xe90, 0xf99, 0xc93, 0xd9a, 0xa96, 0xb9f, 0x895, 0x99c,
            0x69c, 0x795, 0x49f, 0x596, 0x29a, 0x393, 0x99 , 0x190,
            0xf00, 0xe09, 0xd03, 0xc0a, 0xb06, 0xa0f, 0x905, 0x80c,
            0x70c, 0x605, 0x50f, 0x406, 0x30a, 0x203, 0x109, 0x0
        };

        private static readonly sbyte[] triangleTable =
        {
            -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            0, 1, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            1, 8, 3, 9, 8, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            0, 8, 3, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            9, 2, 10, 0, 2, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            2, 8, 3, 2, 10, 8, 10, 9, 8, -1, -1, -1, -1, -1, -1, -1,
            3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            0, 11, 2, 8, 11, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            1, 9, 0, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            1, 11, 2, 1, 9, 11, 9, 8, 11, -1, -1, -1, -1, -1, -1, -1,
            3, 10, 1, 11, 10, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            0, 10, 1, 0, 8, 10, 8, 11, 10, -1, -1, -1, -1, -1, -1, -1,
            3, 9, 0, 3, 11, 9, 11, 10, 9, -1, -1, -1, -1, -1, -1, -1,
            9, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            4, 3, 0, 7, 3, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            0, 1, 9, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            4, 1, 9, 4, 7, 1, 7, 3, 1, -1, -1, -1, -1, -1, -1, -1,
            1, 2, 10, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            3, 4, 7, 3, 0, 4, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1,
            9, 2, 10, 9, 0, 2, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1,
            2, 10, 9, 2, 9, 7, 2, 7, 3, 7, 9, 4, -1, -1, -1, -1,
            8, 4, 7, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            11, 4, 7, 11, 2, 4, 2, 0, 4, -1, -1, -1, -1, -1, -1, -1,
            9, 0, 1, 8, 4, 7, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1,
            4, 7, 11, 9, 4, 11, 9, 11, 2, 9, 2, 1, -1, -1, -1, -1,
            3, 10, 1, 3, 11, 10, 7, 8, 4, -1, -1, -1, -1, -1, -1, -1,
            1, 11, 10, 1, 4, 11, 1, 0, 4, 7, 11, 4, -1, -1, -1, -1,
            4, 7, 8, 9, 0, 11, 9, 11, 10, 11, 0, 3, -1, -1, -1, -1,
            4, 7, 11, 4, 11, 9, 9, 11, 10, -1, -1, -1, -1, -1, -1, -1,
            9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            9, 5, 4, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            0, 5, 4, 1, 5, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            8, 5, 4, 8, 3, 5, 3, 1, 5, -1, -1, -1, -1, -1, -1, -1,
            1, 2, 10, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            3, 0, 8, 1, 2, 10, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1,
            5, 2, 10, 5, 4, 2, 4, 0, 2, -1, -1, -1, -1, -1, -1, -1,
            2, 10, 5, 3, 2, 5, 3, 5, 4, 3, 4, 8, -1, -1, -1, -1,
            9, 5, 4, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            0, 11, 2, 0, 8, 11, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1,
            0, 5, 4, 0, 1, 5, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1,
            2, 1, 5, 2, 5, 8, 2, 8, 11, 4, 8, 5, -1, -1, -1, -1,
            10, 3, 11, 10, 1, 3, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1,
            4, 9, 5, 0, 8, 1, 8, 10, 1, 8, 11, 10, -1, -1, -1, -1,
            5, 4, 0, 5, 0, 11, 5, 11, 10, 11, 0, 3, -1, -1, -1, -1,
            5, 4, 8, 5, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1,
            9, 7, 8, 5, 7, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            9, 3, 0, 9, 5, 3, 5, 7, 3, -1, -1, -1, -1, -1, -1, -1,
            0, 7, 8, 0, 1, 7, 1, 5, 7, -1, -1, -1, -1, -1, -1, -1,
            1, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            9, 7, 8, 9, 5, 7, 10, 1, 2, -1, -1, -1, -1, -1, -1, -1,
            10, 1, 2, 9, 5, 0, 5, 3, 0, 5, 7, 3, -1, -1, -1, -1,
            8, 0, 2, 8, 2, 5, 8, 5, 7, 10, 5, 2, -1, -1, -1, -1,
            2, 10, 5, 2, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1,
            7, 9, 5, 7, 8, 9, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1,
            9, 5, 7, 9, 7, 2, 9, 2, 0, 2, 7, 11, -1, -1, -1, -1,
            2, 3, 11, 0, 1, 8, 1, 7, 8, 1, 5, 7, -1, -1, -1, -1,
            11, 2, 1, 11, 1, 7, 7, 1, 5, -1, -1, -1, -1, -1, -1, -1,
            9, 5, 8, 8, 5, 7, 10, 1, 3, 10, 3, 11, -1, -1, -1, -1,
            5, 7, 0, 5, 0, 9, 7, 11, 0, 1, 0, 10, 11, 10, 0, -1,
            11, 10, 0, 11, 0, 3, 10, 5, 0, 8, 0, 7, 5, 7, 0, -1,
            11, 10, 5, 7, 11, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            0, 8, 3, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            9, 0, 1, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            1, 8, 3, 1, 9, 8, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1,
            1, 6, 5, 2, 6, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            1, 6, 5, 1, 2, 6, 3, 0, 8, -1, -1, -1, -1, -1, -1, -1,
            9, 6, 5, 9, 0, 6, 0, 2, 6, -1, -1, -1, -1, -1, -1, -1,
            5, 9, 8, 5, 8, 2, 5, 2, 6, 3, 2, 8, -1, -1, -1, -1,
            2, 3, 11, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            11, 0, 8, 11, 2, 0, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1,
            0, 1, 9, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1,
            5, 10, 6, 1, 9, 2, 9, 11, 2, 9, 8, 11, -1, -1, -1, -1,
            6, 3, 11, 6, 5, 3, 5, 1, 3, -1, -1, -1, -1, -1, -1, -1,
            0, 8, 11, 0, 11, 5, 0, 5, 1, 5, 11, 6, -1, -1, -1, -1,
            3, 11, 6, 0, 3, 6, 0, 6, 5, 0, 5, 9, -1, -1, -1, -1,
            6, 5, 9, 6, 9, 11, 11, 9, 8, -1, -1, -1, -1, -1, -1, -1,
            5, 10, 6, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            4, 3, 0, 4, 7, 3, 6, 5, 10, -1, -1, -1, -1, -1, -1, -1,
            1, 9, 0, 5, 10, 6, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1,
            10, 6, 5, 1, 9, 7, 1, 7, 3, 7, 9, 4, -1, -1, -1, -1,
            6, 1, 2, 6, 5, 1, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1,
            1, 2, 5, 5, 2, 6, 3, 0, 4, 3, 4, 7, -1, -1, -1, -1,
            8, 4, 7, 9, 0, 5, 0, 6, 5, 0, 2, 6, -1, -1, -1, -1,
            7, 3, 9, 7, 9, 4, 3, 2, 9, 5, 9, 6, 2, 6, 9, -1,
            3, 11, 2, 7, 8, 4, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1,
            5, 10, 6, 4, 7, 2, 4, 2, 0, 2, 7, 11, -1, -1, -1, -1,
            0, 1, 9, 4, 7, 8, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1,
            9, 2, 1, 9, 11, 2, 9, 4, 11, 7, 11, 4, 5, 10, 6, -1,
            8, 4, 7, 3, 11, 5, 3, 5, 1, 5, 11, 6, -1, -1, -1, -1,
            5, 1, 11, 5, 11, 6, 1, 0, 11, 7, 11, 4, 0, 4, 11, -1,
            0, 5, 9, 0, 6, 5, 0, 3, 6, 11, 6, 3, 8, 4, 7, -1,
            6, 5, 9, 6, 9, 11, 4, 7, 9, 7, 11, 9, -1, -1, -1, -1,
            10, 4, 9, 6, 4, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            4, 10, 6, 4, 9, 10, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1,
            10, 0, 1, 10, 6, 0, 6, 4, 0, -1, -1, -1, -1, -1, -1, -1,
            8, 3, 1, 8, 1, 6, 8, 6, 4, 6, 1, 10, -1, -1, -1, -1,
            1, 4, 9, 1, 2, 4, 2, 6, 4, -1, -1, -1, -1, -1, -1, -1,
            3, 0, 8, 1, 2, 9, 2, 4, 9, 2, 6, 4, -1, -1, -1, -1,
            0, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            8, 3, 2, 8, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1,
            10, 4, 9, 10, 6, 4, 11, 2, 3, -1, -1, -1, -1, -1, -1, -1,
            0, 8, 2, 2, 8, 11, 4, 9, 10, 4, 10, 6, -1, -1, -1, -1,
            3, 11, 2, 0, 1, 6, 0, 6, 4, 6, 1, 10, -1, -1, -1, -1,
            6, 4, 1, 6, 1, 10, 4, 8, 1, 2, 1, 11, 8, 11, 1, -1,
            9, 6, 4, 9, 3, 6, 9, 1, 3, 11, 6, 3, -1, -1, -1, -1,
            8, 11, 1, 8, 1, 0, 11, 6, 1, 9, 1, 4, 6, 4, 1, -1,
            3, 11, 6, 3, 6, 0, 0, 6, 4, -1, -1, -1, -1, -1, -1, -1,
            6, 4, 8, 11, 6, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            7, 10, 6, 7, 8, 10, 8, 9, 10, -1, -1, -1, -1, -1, -1, -1,
            0, 7, 3, 0, 10, 7, 0, 9, 10, 6, 7, 10, -1, -1, -1, -1,
            10, 6, 7, 1, 10, 7, 1, 7, 8, 1, 8, 0, -1, -1, -1, -1,
            10, 6, 7, 10, 7, 1, 1, 7, 3, -1, -1, -1, -1, -1, -1, -1,
            1, 2, 6, 1, 6, 8, 1, 8, 9, 8, 6, 7, -1, -1, -1, -1,
            2, 6, 9, 2, 9, 1, 6, 7, 9, 0, 9, 3, 7, 3, 9, -1,
            7, 8, 0, 7, 0, 6, 6, 0, 2, -1, -1, -1, -1, -1, -1, -1,
            7, 3, 2, 6, 7, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            2, 3, 11, 10, 6, 8, 10, 8, 9, 8, 6, 7, -1, -1, -1, -1,
            2, 0, 7, 2, 7, 11, 0, 9, 7, 6, 7, 10, 9, 10, 7, -1,
            1, 8, 0, 1, 7, 8, 1, 10, 7, 6, 7, 10, 2, 3, 11, -1,
            11, 2, 1, 11, 1, 7, 10, 6, 1, 6, 7, 1, -1, -1, -1, -1,
            8, 9, 6, 8, 6, 7, 9, 1, 6, 11, 6, 3, 1, 3, 6, -1,
            0, 9, 1, 11, 6, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            7, 8, 0, 7, 0, 6, 3, 11, 0, 11, 6, 0, -1, -1, -1, -1,
            7, 11, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            3, 0, 8, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            0, 1, 9, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            8, 1, 9, 8, 3, 1, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1,
            10, 1, 2, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            1, 2, 10, 3, 0, 8, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1,
            2, 9, 0, 2, 10, 9, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1,
            6, 11, 7, 2, 10, 3, 10, 8, 3, 10, 9, 8, -1, -1, -1, -1,
            7, 2, 3, 6, 2, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            7, 0, 8, 7, 6, 0, 6, 2, 0, -1, -1, -1, -1, -1, -1, -1,
            2, 7, 6, 2, 3, 7, 0, 1, 9, -1, -1, -1, -1, -1, -1, -1,
            1, 6, 2, 1, 8, 6, 1, 9, 8, 8, 7, 6, -1, -1, -1, -1,
            10, 7, 6, 10, 1, 7, 1, 3, 7, -1, -1, -1, -1, -1, -1, -1,
            10, 7, 6, 1, 7, 10, 1, 8, 7, 1, 0, 8, -1, -1, -1, -1,
            0, 3, 7, 0, 7, 10, 0, 10, 9, 6, 10, 7, -1, -1, -1, -1,
            7, 6, 10, 7, 10, 8, 8, 10, 9, -1, -1, -1, -1, -1, -1, -1,
            6, 8, 4, 11, 8, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            3, 6, 11, 3, 0, 6, 0, 4, 6, -1, -1, -1, -1, -1, -1, -1,
            8, 6, 11, 8, 4, 6, 9, 0, 1, -1, -1, -1, -1, -1, -1, -1,
            9, 4, 6, 9, 6, 3, 9, 3, 1, 11, 3, 6, -1, -1, -1, -1,
            6, 8, 4, 6, 11, 8, 2, 10, 1, -1, -1, -1, -1, -1, -1, -1,
            1, 2, 10, 3, 0, 11, 0, 6, 11, 0, 4, 6, -1, -1, -1, -1,
            4, 11, 8, 4, 6, 11, 0, 2, 9, 2, 10, 9, -1, -1, -1, -1,
            10, 9, 3, 10, 3, 2, 9, 4, 3, 11, 3, 6, 4, 6, 3, -1,
            8, 2, 3, 8, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1,
            0, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            1, 9, 0, 2, 3, 4, 2, 4, 6, 4, 3, 8, -1, -1, -1, -1,
            1, 9, 4, 1, 4, 2, 2, 4, 6, -1, -1, -1, -1, -1, -1, -1,
            8, 1, 3, 8, 6, 1, 8, 4, 6, 6, 10, 1, -1, -1, -1, -1,
            10, 1, 0, 10, 0, 6, 6, 0, 4, -1, -1, -1, -1, -1, -1, -1,
            4, 6, 3, 4, 3, 8, 6, 10, 3, 0, 3, 9, 10, 9, 3, -1,
            10, 9, 4, 6, 10, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            4, 9, 5, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            0, 8, 3, 4, 9, 5, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1,
            5, 0, 1, 5, 4, 0, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1,
            11, 7, 6, 8, 3, 4, 3, 5, 4, 3, 1, 5, -1, -1, -1, -1,
            9, 5, 4, 10, 1, 2, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1,
            6, 11, 7, 1, 2, 10, 0, 8, 3, 4, 9, 5, -1, -1, -1, -1,
            7, 6, 11, 5, 4, 10, 4, 2, 10, 4, 0, 2, -1, -1, -1, -1,
            3, 4, 8, 3, 5, 4, 3, 2, 5, 10, 5, 2, 11, 7, 6, -1,
            7, 2, 3, 7, 6, 2, 5, 4, 9, -1, -1, -1, -1, -1, -1, -1,
            9, 5, 4, 0, 8, 6, 0, 6, 2, 6, 8, 7, -1, -1, -1, -1,
            3, 6, 2, 3, 7, 6, 1, 5, 0, 5, 4, 0, -1, -1, -1, -1,
            6, 2, 8, 6, 8, 7, 2, 1, 8, 4, 8, 5, 1, 5, 8, -1,
            9, 5, 4, 10, 1, 6, 1, 7, 6, 1, 3, 7, -1, -1, -1, -1,
            1, 6, 10, 1, 7, 6, 1, 0, 7, 8, 7, 0, 9, 5, 4, -1,
            4, 0, 10, 4, 10, 5, 0, 3, 10, 6, 10, 7, 3, 7, 10, -1,
            7, 6, 10, 7, 10, 8, 5, 4, 10, 4, 8, 10, -1, -1, -1, -1,
            6, 9, 5, 6, 11, 9, 11, 8, 9, -1, -1, -1, -1, -1, -1, -1,
            3, 6, 11, 0, 6, 3, 0, 5, 6, 0, 9, 5, -1, -1, -1, -1,
            0, 11, 8, 0, 5, 11, 0, 1, 5, 5, 6, 11, -1, -1, -1, -1,
            6, 11, 3, 6, 3, 5, 5, 3, 1, -1, -1, -1, -1, -1, -1, -1,
            1, 2, 10, 9, 5, 11, 9, 11, 8, 11, 5, 6, -1, -1, -1, -1,
            0, 11, 3, 0, 6, 11, 0, 9, 6, 5, 6, 9, 1, 2, 10, -1,
            11, 8, 5, 11, 5, 6, 8, 0, 5, 10, 5, 2, 0, 2, 5, -1,
            6, 11, 3, 6, 3, 5, 2, 10, 3, 10, 5, 3, -1, -1, -1, -1,
            5, 8, 9, 5, 2, 8, 5, 6, 2, 3, 8, 2, -1, -1, -1, -1,
            9, 5, 6, 9, 6, 0, 0, 6, 2, -1, -1, -1, -1, -1, -1, -1,
            1, 5, 8, 1, 8, 0, 5, 6, 8, 3, 8, 2, 6, 2, 8, -1,
            1, 5, 6, 2, 1, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            1, 3, 6, 1, 6, 10, 3, 8, 6, 5, 6, 9, 8, 9, 6, -1,
            10, 1, 0, 10, 0, 6, 9, 5, 0, 5, 6, 0, -1, -1, -1, -1,
            0, 3, 8, 5, 6, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            10, 5, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            11, 5, 10, 7, 5, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            11, 5, 10, 11, 7, 5, 8, 3, 0, -1, -1, -1, -1, -1, -1, -1,
            5, 11, 7, 5, 10, 11, 1, 9, 0, -1, -1, -1, -1, -1, -1, -1,
            10, 7, 5, 10, 11, 7, 9, 8, 1, 8, 3, 1, -1, -1, -1, -1,
            11, 1, 2, 11, 7, 1, 7, 5, 1, -1, -1, -1, -1, -1, -1, -1,
            0, 8, 3, 1, 2, 7, 1, 7, 5, 7, 2, 11, -1, -1, -1, -1,
            9, 7, 5, 9, 2, 7, 9, 0, 2, 2, 11, 7, -1, -1, -1, -1,
            7, 5, 2, 7, 2, 11, 5, 9, 2, 3, 2, 8, 9, 8, 2, -1,
            2, 5, 10, 2, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1,
            8, 2, 0, 8, 5, 2, 8, 7, 5, 10, 2, 5, -1, -1, -1, -1,
            9, 0, 1, 5, 10, 3, 5, 3, 7, 3, 10, 2, -1, -1, -1, -1,
            9, 8, 2, 9, 2, 1, 8, 7, 2, 10, 2, 5, 7, 5, 2, -1,
            1, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            0, 8, 7, 0, 7, 1, 1, 7, 5, -1, -1, -1, -1, -1, -1, -1,
            9, 0, 3, 9, 3, 5, 5, 3, 7, -1, -1, -1, -1, -1, -1, -1,
            9, 8, 7, 5, 9, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            5, 8, 4, 5, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1,
            5, 0, 4, 5, 11, 0, 5, 10, 11, 11, 3, 0, -1, -1, -1, -1,
            0, 1, 9, 8, 4, 10, 8, 10, 11, 10, 4, 5, -1, -1, -1, -1,
            10, 11, 4, 10, 4, 5, 11, 3, 4, 9, 4, 1, 3, 1, 4, -1,
            2, 5, 1, 2, 8, 5, 2, 11, 8, 4, 5, 8, -1, -1, -1, -1,
            0, 4, 11, 0, 11, 3, 4, 5, 11, 2, 11, 1, 5, 1, 11, -1,
            0, 2, 5, 0, 5, 9, 2, 11, 5, 4, 5, 8, 11, 8, 5, -1,
            9, 4, 5, 2, 11, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            2, 5, 10, 3, 5, 2, 3, 4, 5, 3, 8, 4, -1, -1, -1, -1,
            5, 10, 2, 5, 2, 4, 4, 2, 0, -1, -1, -1, -1, -1, -1, -1,
            3, 10, 2, 3, 5, 10, 3, 8, 5, 4, 5, 8, 0, 1, 9, -1,
            5, 10, 2, 5, 2, 4, 1, 9, 2, 9, 4, 2, -1, -1, -1, -1,
            8, 4, 5, 8, 5, 3, 3, 5, 1, -1, -1, -1, -1, -1, -1, -1,
            0, 4, 5, 1, 0, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            8, 4, 5, 8, 5, 3, 9, 0, 5, 0, 3, 5, -1, -1, -1, -1,
            9, 4, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            4, 11, 7, 4, 9, 11, 9, 10, 11, -1, -1, -1, -1, -1, -1, -1,
            0, 8, 3, 4, 9, 7, 9, 11, 7, 9, 10, 11, -1, -1, -1, -1,
            1, 10, 11, 1, 11, 4, 1, 4, 0, 7, 4, 11, -1, -1, -1, -1,
            3, 1, 4, 3, 4, 8, 1, 10, 4, 7, 4, 11, 10, 11, 4, -1,
            4, 11, 7, 9, 11, 4, 9, 2, 11, 9, 1, 2, -1, -1, -1, -1,
            9, 7, 4, 9, 11, 7, 9, 1, 11, 2, 11, 1, 0, 8, 3, -1,
            11, 7, 4, 11, 4, 2, 2, 4, 0, -1, -1, -1, -1, -1, -1, -1,
            11, 7, 4, 11, 4, 2, 8, 3, 4, 3, 2, 4, -1, -1, -1, -1,
            2, 9, 10, 2, 7, 9, 2, 3, 7, 7, 4, 9, -1, -1, -1, -1,
            9, 10, 7, 9, 7, 4, 10, 2, 7, 8, 7, 0, 2, 0, 7, -1,
            3, 7, 10, 3, 10, 2, 7, 4, 10, 1, 10, 0, 4, 0, 10, -1,
            1, 10, 2, 8, 7, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            4, 9, 1, 4, 1, 7, 7, 1, 3, -1, -1, -1, -1, -1, -1, -1,
            4, 9, 1, 4, 1, 7, 0, 8, 1, 8, 7, 1, -1, -1, -1, -1,
            4, 0, 3, 7, 4, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            4, 8, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            9, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            3, 0, 9, 3, 9, 11, 11, 9, 10, -1, -1, -1, -1, -1, -1, -1,
            0, 1, 10, 0, 10, 8, 8, 10, 11, -1, -1, -1, -1, -1, -1, -1,
            3, 1, 10, 11, 3, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            1, 2, 11, 1, 11, 9, 9, 11, 8, -1, -1, -1, -1, -1, -1, -1,
            3, 0, 9, 3, 9, 11, 1, 2, 9, 2, 11, 9, -1, -1, -1, -1,
            0, 2, 11, 8, 0, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            3, 2, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            2, 3, 8, 2, 8, 10, 10, 8, 9, -1, -1, -1, -1, -1, -1, -1,
            9, 10, 2, 0, 9, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            2, 3, 8, 2, 8, 10, 0, 1, 8, 1, 10, 8, -1, -1, -1, -1,
            1, 10, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            1, 3, 8, 9, 1, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            0, 9, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            0, 3, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
            -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        };
    }
}
