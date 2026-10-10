#!/usr/bin/env python3
"""Generates the forest region-of-interest scenery: roi_forest.sdf + roi_forest.yaml.

Why a generator and not hand-written SDF.  The forest is thousands of trees, grass
tiles and boundary posts, and the metadata in the YAML (trunk positions, extents)
must never drift out of step with the scenery it describes.  Everything here is
derived from a handful of numbers instead, so changing the size is one flag.

Why the output is committed rather than built.  ``worlds/fragments/`` is shipped
*data*, not an artefact (``docs/architecture.md``): ``base_world.sdf.j2`` includes
the fragment through a Jinja loader rooted at that directory, so generating at build
time would put shipped data in gitignored ``build/`` and give every ``uavsim world``
a dependency on this script.  A committed fragment also means a *reviewable diff*
when the forest changes -- which matters, because it changes what every camera in
every scenario sees.  Reproducibility is guaranteed instead by ``--check``, which
regenerates in memory and byte-compares; a test runs it.

Why this lives in tools/ and not uavsim/.  ``uavsim/`` is imported by the ROS 2
launch file and must stay dependency-light and fast to import.  An authoring tool
that runs once per forest does not belong on the runtime import path.

Why one link, not one model per tree.  As separate ``<include>``d models -- the
obvious way, and how the 200-tree forest was built -- every tile and tree becomes its
own physics body, and every tree drags in its full canopy mesh as a collision shape.
On one static link, with one bounding cylinder per tree for collision, the scenery
is cheaper than the old 200-tree forest despite having 3.6x the trees.

Why 500 m and not 1000 m.  Measured on an RTX 3060 with four gimbal-like cameras
(640x480, 10 Hz, subscribed so they actually render):

    forest                         real-time factor   server RSS
    legacy 200 trees, includes     1.00               2.4 GB
    500 m square,  726 trees       1.00               1.7 GB
    1000 m square, 2904 trees      0.43               4.9 GB
    1000 m square, as includes     did not finish loading in 3 min, >22 GB

A kilometre square at this density halves the simulation speed, so the default is
500 m; ``--size 1000 --oak 1742 --pine 1162`` builds the larger one.

    python3 tools/gen_roi_forest.py --size 500 --oak 436 --pine 290 \
            --fires 40 --seed 20251007
"""

from __future__ import annotations

import argparse
import math
import random
import sys
from dataclasses import dataclass, replace
from pathlib import Path

PKG_ROOT = Path(__file__).resolve().parents[1]
FRAGMENTS_DIR = PKG_ROOT / "worlds" / "fragments"

FUEL = "https://fuel.gazebosim.org/1.0/openrobotics/models"
GRASS_TEXTURE = "model://grasspatch_visual/materials/textures/grass_dry.png"


@dataclass(frozen=True)
class TreeKind:
    """One Fuel tree, referenced by its mesh and textures at a pinned version.

    The Fuel model's own collision is its whole canopy mesh, which is what made the
    per-tree ``<include>`` forest unloadable at this size; ``collision_*`` is a
    bounding cylinder from the mesh extents (the DAE is in inches, scaled 0.0254).
    """

    name: str
    files: str
    mesh: str
    branch_texture: str
    collision_radius_m: float
    collision_height_m: float

    def uri(self, path: str) -> str:
        return f"{self.files}/{path}"


OAK = TreeKind("Oak_Tree", f"{FUEL}/oak tree/7/files", "meshes/oak_tree.dae",
               "materials/textures/branch_diffuse.png", 5.0, 6.5)
PINE = TreeKind("Pine_Tree", f"{FUEL}/pine tree/6/files", "meshes/pine_tree.dae",
                "materials/textures/branch_2_diffuse.png", 1.6, 5.1)

# The origin axis marker, carried over verbatim from the legacy world.  It also
# carries the world's only navsat sensor, which the PX4 backend depends on -- do not
# drop it when editing this template.
AXES_SDF = """\
    <model name="axes">
      <static>1</static>
      <link name="link">
        <visual name="r">
          <cast_shadows>0</cast_shadows>
          <pose>5 0 0.1 0 0 0</pose>
          <geometry>
            <box>
              <size>10 0.01 0.01</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 0.8</ambient>
            <diffuse>1 0 0 0.8</diffuse>
            <emissive>1 0 0 0.8</emissive>
            <specular>0.5 0.5 0.5 0.8</specular>
          </material>
        </visual>
        <visual name="g">
          <cast_shadows>0</cast_shadows>
          <pose>0 5 0.1 0 0 0</pose>
          <geometry>
            <box>
              <size>0.01 10 0.01</size>
            </box>
          </geometry>
          <material>
            <ambient>0 1 0 0.8</ambient>
            <diffuse>0 1 0 0.8</diffuse>
            <emissive>0 1 0 0.8</emissive>
            <specular>0.5 0.5 0.5 0.8</specular>
          </material>
        </visual>
        <visual name="b">
          <cast_shadows>0</cast_shadows>
          <pose>0 0 5.1 0 0 0</pose>
          <geometry>
            <box>
              <size>0.01 0.01 10</size>
            </box>
          </geometry>
          <material>
            <ambient>0 0 1 0.8</ambient>
            <diffuse>0 0 1 0.8</diffuse>
            <emissive>0 0 1 0.8</emissive>
            <specular>0.5 0.5 0.5 0.8</specular>
          </material>
        </visual>
        <sensor name="navsat_sensor" type="navsat">
          <always_on>1</always_on>
          <update_rate>1</update_rate>
        </sensor>
      </link>
    </model>
"""


@dataclass(frozen=True)
class ForestParams:
    """Every number the forest is derived from.

    The forest is a square centred on the world origin, and the whole square is the
    search area: the boundary posts run round its edge and targets may be placed
    anywhere inside it.  Tree density is the legacy forest's (100 trees in a disc of
    r=104.7, 2.90e-3 per square metre), so a 500 m square needs ~726 trees.
    """

    size_m: float = 500.0                 # side of the square
    grass_tile_m: float = 15.0            # the grasspatch_visual plane
    grass_overlap_m: float = 0.17         # tiles overlap by at least this: no seams
    oak_count: int = 436                  # 3:2 oak:pine, as the legacy forest
    pine_count: int = 290
    fire_count: int = 40
    fire_z_m: float = 0.40
    edge_margin_m: float = 3.0            # trunks this far inside the edge
    boundary_spacing_m: float = 20.0      # posts round the edge
    min_tree_spacing_m: float = 2.5
    min_fire_spacing_m: float = 5.0
    min_fire_tree_m: float = 3.0
    seed: int = 20251007

    @property
    def half_m(self) -> float:
        return self.size_m / 2.0

    @property
    def tree_count(self) -> int:
        return self.oak_count + self.pine_count


# -- geometry ------------------------------------------------------------------

class _Grid:
    """Spatial hash for the minimum-spacing checks.

    Placement is rejection sampling against everything already placed; with ~2900
    trees a linear scan makes the generator (and the test that runs it) slow.
    """

    def __init__(self, cell_m: float) -> None:
        self.cell = cell_m
        self.cells: dict[tuple[int, int], list[tuple[float, float]]] = {}

    def _key(self, x: float, y: float) -> tuple[int, int]:
        return int(math.floor(x / self.cell)), int(math.floor(y / self.cell))

    def add(self, x: float, y: float) -> None:
        self.cells.setdefault(self._key(x, y), []).append((x, y))

    def near(self, x: float, y: float, radius_m: float) -> bool:
        """True if any stored point is closer than ``radius_m`` (<= the cell size)."""
        kx, ky = self._key(x, y)
        return any(math.hypot(x - px, y - py) < radius_m
                   for dx in (-1, 0, 1) for dy in (-1, 0, 1)
                   for px, py in self.cells.get((kx + dx, ky + dy), ()))


def grass_tiles(p: ForestParams) -> list[tuple[str, float, float]]:
    """Grass tile centres on a square grid that covers the square exactly.

    The pitch is stretched just enough that the outermost tiles end flush with the
    edge, while neighbours still overlap by at least ``grass_overlap_m``.
    """
    step = p.grass_tile_m - p.grass_overlap_m
    n = max(1, math.ceil((p.size_m - p.grass_overlap_m) / step))
    pitch = (p.size_m - p.grass_tile_m) / (n - 1) if n > 1 else 0.0
    first = -p.half_m + p.grass_tile_m / 2.0
    return [(f"grass_{i:02d}_{j:02d}", first + i * pitch, first + j * pitch)
            for i in range(n) for j in range(n)]


def ground_extent(p: ForestParams, tiles: list[tuple[str, float, float]]) -> list[float]:
    """The grass bounding box: tile centres grown by half a tile.

    This is what ``terrain_patch()`` clips the GCS ground visual against, so it must
    describe where the grass actually *is*.
    """
    half = p.grass_tile_m / 2.0
    xs = [x for _, x, _ in tiles]
    ys = [y for _, _, y in tiles]
    return [round(min(xs) - half, 2), round(max(xs) + half, 2),
            round(min(ys) - half, 2), round(max(ys) + half, 2)]


def mission_extent(p: ForestParams) -> list[float]:
    h = round(p.half_m, 2)
    return [-h, h, -h, h]


def boundary_posts(p: ForestParams) -> list[tuple[str, float, float]]:
    """Posts round the edge of the square, corners included, plus one at the origin."""
    per_side = max(1, round(p.size_m / p.boundary_spacing_m))
    h = p.half_m
    posts = [("boundary_origin", 0.0, 0.0)]
    # walk the perimeter anticlockwise from the south-west corner
    corners = [(-h, -h), (h, -h), (h, h), (-h, h)]
    k = 0
    for (x0, y0), (x1, y1) in zip(corners, corners[1:] + corners[:1]):
        for s in range(per_side):
            t = s / per_side
            posts.append((f"boundary_{k:03d}", x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))
            k += 1
    return posts


def _sample(rng: random.Random, half_m: float) -> tuple[float, float]:
    """Area-uniform point in the square [-half_m, half_m]^2."""
    return rng.uniform(-half_m, half_m), rng.uniform(-half_m, half_m)


def _place(rng: random.Random, count: int, half_m: float, grid: _Grid,
           min_spacing_m: float, label: str) -> list[tuple[float, float]]:
    out: list[tuple[float, float]] = []
    for _ in range(count):
        for _attempt in range(2000):
            x, y = _sample(rng, half_m)
            if grid.near(x, y, min_spacing_m):
                continue
            grid.add(x, y)
            out.append((x, y))
            break
        else:
            raise RuntimeError(
                f"could not place {label} {len(out) + 1} of {count} in a "
                f"{2 * half_m:g} m square at {min_spacing_m} m spacing; "
                f"lower the count or the spacing")
    return out


def tree_positions(p: ForestParams,
                   rng: random.Random) -> list[tuple[TreeKind, str, float, float, float]]:
    """``(kind, name, x, y, yaw)`` in the order they are emitted.

    The YAML trunk list must use the same order: ``uavsim/world.py`` reads it to keep
    placed targets out of canopies.
    """
    grid = _Grid(max(p.min_tree_spacing_m, 1.0))
    half = p.half_m - p.edge_margin_m
    oaks = _place(rng, p.oak_count, half, grid, p.min_tree_spacing_m, "oak")
    pines = _place(rng, p.pine_count, half, grid, p.min_tree_spacing_m, "pine")

    trees: list[tuple[TreeKind, str, float, float, float]] = []
    for kind, points in ((OAK, oaks), (PINE, pines)):
        for i, (x, y) in enumerate(points, 1):
            trees.append((kind, f"{kind.name}_{i}", x, y, rng.uniform(0.0, 2.0 * math.pi)))
    return trees


def fire_positions(p: ForestParams, trees: list[tuple[TreeKind, str, float, float, float]],
                   rng: random.Random) -> list[list[float]]:
    """Fire markers inside the square, clear of trunks and of each other.

    These are ground truth, not scenery: the scenario decides whether to render them
    (``targets.fires``), and ``place_casualties`` keeps casualties away from them.
    """
    trunks = _Grid(max(p.min_fire_tree_m, 1.0))
    for _, _, x, y, _ in trees:
        trunks.add(x, y)
    placed = _Grid(max(p.min_fire_spacing_m, 1.0))
    out: list[list[float]] = []
    for _ in range(p.fire_count):
        for _attempt in range(2000):
            x, y = _sample(rng, p.half_m - p.edge_margin_m)
            if trunks.near(x, y, p.min_fire_tree_m):
                continue
            if placed.near(x, y, p.min_fire_spacing_m):
                continue
            placed.add(x, y)
            out.append([round(x, 2), round(y, 2), p.fire_z_m])
            break
        else:
            raise RuntimeError(
                f"could not place fire {len(out) + 1} of {p.fire_count} in the "
                f"{p.size_m:g} m square; lower --fires")
    return out


# -- rendering -----------------------------------------------------------------

def _invocation(p: ForestParams) -> str:
    return (f"python3 tools/gen_roi_forest.py --size {p.size_m:g} --oak {p.oak_count} "
            f"--pine {p.pine_count} \\\n"
            f"        --fires {p.fire_count} --seed {p.seed}")


SCENERY_MODEL_NAME = "forest_scenery"


def render_sdf(p: ForestParams, tiles: list[tuple[str, float, float]],
               posts: list[tuple[str, float, float]],
               trees: list[tuple[TreeKind, str, float, float, float]]) -> str:
    """The fragment: the bulk scenery as visuals and collisions on one static link.

    Why one link: see the module docstring -- per-object models are far heavier.  It also keeps Gazebo's entity tree navigable: vehicles are created last,
    at runtime, and a flat list of thousands of scenery models buries them, so a
    user looking for their own drone concludes it never spawned.

    ``axes`` deliberately stays at world level: it carries the world's only navsat
    sensor, and nesting it would change that sensor's topic path, which the PX4
    backend depends on.

    The grass tiles are thin boxes whose top face is at z=0, not ``<plane>``s: the
    world has exactly one plane, in base_world.sdf.j2.
    """
    out = [f"""\
<!-- Forest region-of-interest scenery, centred on the world origin.
     GENERATED by tools/gen_roi_forest.py. Do not edit by hand. The exact
     regeneration command is in the header of roi_forest.yaml, which can carry the
     flags that an XML comment cannot; the generator's check mode asserts that the
     committed files still match it. Built from: size={p.size_m:g} m square,
     oak={p.oak_count}, pine={p.pine_count}, fires={p.fire_count}, seed={p.seed}.
     Included by worlds/templates/base_world.sdf.j2.
     {len(tiles)} grass tiles, {len(trees)} trees and {len(posts)} boundary posts, all as
     visuals and collisions of ONE static link in '{SCENERY_MODEL_NAME}': as separate
     models the same scenery is far heavier to load and simulate
     (tools/gen_roi_forest.py has the measurements). The axis marker stays at world level because it carries the
     navsat sensor the PX4 backend needs.
     Declares no plane geometry and no ground plane: there is exactly one plane
     collision per world, in base_world.sdf.j2.
     Fire/casualty/ROI targets are placed by the scenario, see roi_forest.yaml. -->
"""]
    out.append(AXES_SDF)
    out.append(f'    <model name="{SCENERY_MODEL_NAME}">\n')
    out.append("      <static>true</static>\n")
    out.append("      <pose>0 0 0 0 0 0</pose>\n")
    out.append('      <link name="link">\n')
    half = p.grass_tile_m / 2.0
    for name, x, y in tiles:
        out.append(f"""\
        <visual name="{name}">
          <pose>{x:.3f} {y:.3f} -0.005 0 0 0</pose>
          <cast_shadows>false</cast_shadows>
          <geometry><box><size>{2 * half:g} {2 * half:g} 0.01</size></box></geometry>
          <material>
            <diffuse>1 1 1 1</diffuse><specular>1 1 1 1</specular>
            <pbr><metal><albedo_map>{GRASS_TEXTURE}</albedo_map></metal></pbr>
          </material>
        </visual>
""")
    for name, x, y in posts:
        out.append(f"""\
        <visual name="{name}">
          <pose>{x:.3f} {y:.3f} 1.0 0 0 0</pose>
          <geometry><cylinder><radius>0.20</radius><length>2.0</length></cylinder></geometry>
        </visual>
        <collision name="{name}_collision">
          <pose>{x:.3f} {y:.3f} 1.0 0 0 0</pose>
          <geometry><cylinder><radius>0.20</radius><length>2.0</length></cylinder></geometry>
        </collision>
""")
    for kind, name, x, y, yaw in trees:
        mesh = kind.uri(kind.mesh)
        out.append(f"""\
        <visual name="{name}_branch">
          <pose>{x:.3f} {y:.3f} 0 0 0 {yaw:.3f}</pose>
          <geometry><mesh><uri>{mesh}</uri><submesh><name>Branch</name></submesh></mesh></geometry>
          <material>
            <double_sided>true</double_sided><diffuse>1 1 1 1</diffuse>
            <pbr><metal><albedo_map>{kind.uri(kind.branch_texture)}</albedo_map></metal></pbr>
          </material>
        </visual>
        <visual name="{name}_bark">
          <pose>{x:.3f} {y:.3f} 0 0 0 {yaw:.3f}</pose>
          <geometry><mesh><uri>{mesh}</uri><submesh><name>Bark</name></submesh></mesh></geometry>
          <material>
            <diffuse>1 1 1 1</diffuse>
            <pbr><metal><albedo_map>{kind.uri("materials/textures/bark_diffuse.png")}</albedo_map></metal></pbr>
          </material>
        </visual>
        <collision name="{name}_collision">
          <pose>{x:.3f} {y:.3f} {kind.collision_height_m / 2:g} 0 0 0</pose>
          <geometry><cylinder><radius>{kind.collision_radius_m:g}</radius><length>{kind.collision_height_m:g}</length></cylinder></geometry>
        </collision>
""")
    out.append("      </link>\n")
    out.append("    </model>\n")
    return "".join(out)


def render_yaml(p: ForestParams, tiles: list[tuple[str, float, float]],
                posts: list[tuple[str, float, float]],
                trees: list[tuple[TreeKind, str, float, float, float]],
                fires: list[list[float]], extent: list[float]) -> str:
    mission = mission_extent(p)
    lines = [
        "# Forest region-of-interest metadata.",
        "# GENERATED -- do not edit by hand. Regenerate with:",
        f"#   {_invocation(p).replace(chr(10), chr(10) + '#   ')}",
        "id: roi_forest",
        f"description: Forest search area, a {p.size_m:g} m x {p.size_m:g} m square "
        "centred on the world origin.",
        "centre: [0.0, 0.0]",
        f"size_m: [{p.size_m}, {p.size_m}]",
        f"mission_extent: [{mission[0]}, {mission[1]}, {mission[2]}, {mission[3]}]"
        "   # xmin, xmax, ymin, ymax; the boundary posts",
        f"tree_count: {len(trees)}",
        f"ground_extent: [{extent[0]}, {extent[1]}, {extent[2]}, {extent[3]}]"
        "   # xmin, xmax, ymin, ymax",
        "generated_by: tools/gen_roi_forest.py",
        f"generator_seed: {p.seed}",
        "generator_args: {"
        f"size_m: {p.size_m}, oak: {p.oak_count}, pine: {p.pine_count}, "
        f"fires: {p.fire_count}}}",
        "fires:",
    ]
    lines += [f"  - [{x:.2f}, {y:.2f}, {z:.2f}]" for x, y, z in fires]
    lines.append("trees:   # trunk positions, in SDF order; "
                 "used to keep placed targets out of the canopy")
    lines += [f"  - [{x:.2f}, {y:.2f}]" for _, _, x, y, _ in trees]
    return "\n".join(lines) + "\n"


def generate(p: ForestParams) -> tuple[str, str]:
    rng = random.Random(p.seed)
    tiles = grass_tiles(p)
    posts = boundary_posts(p)
    trees = tree_positions(p, rng)
    fires = fire_positions(p, trees, rng)
    extent = ground_extent(p, tiles)
    return render_sdf(p, tiles, posts, trees), render_yaml(p, tiles, posts, trees,
                                                           fires, extent)


# -- cli -----------------------------------------------------------------------

def main(argv: list[str] | None = None) -> int:
    d = ForestParams()
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--size", type=float, default=d.size_m,
                    help="side of the square forest, in metres")
    ap.add_argument("--oak", type=int, default=d.oak_count)
    ap.add_argument("--pine", type=int, default=d.pine_count)
    ap.add_argument("--fires", type=int, default=d.fire_count)
    ap.add_argument("--seed", type=int, default=d.seed)
    ap.add_argument("--out-dir", type=Path, default=FRAGMENTS_DIR)
    ap.add_argument("--check", action="store_true",
                    help="regenerate in memory and byte-compare; exit 1 on a mismatch")
    args = ap.parse_args(argv)

    params = replace(d, size_m=args.size, oak_count=args.oak, pine_count=args.pine,
                     fire_count=args.fires, seed=args.seed)

    sdf_text, yaml_text = generate(params)
    sdf_path = args.out_dir / "roi_forest.sdf"
    yaml_path = args.out_dir / "roi_forest.yaml"

    if args.check:
        stale = [p for p, text in ((sdf_path, sdf_text), (yaml_path, yaml_text))
                 if not p.exists() or p.read_text() != text]
        if stale:
            for path in stale:
                print(f"stale: {path.relative_to(PKG_ROOT)}", file=sys.stderr)
            print("regenerate with:\n  " + _invocation(params), file=sys.stderr)
            return 1
        print(f"up to date: {sdf_path.name}, {yaml_path.name}")
        return 0

    args.out_dir.mkdir(parents=True, exist_ok=True)
    sdf_path.write_text(sdf_text)
    yaml_path.write_text(yaml_text)
    print(f"wrote {sdf_path}\nwrote {yaml_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
