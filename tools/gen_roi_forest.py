#!/usr/bin/env python3
"""Generates the forest region-of-interest scenery: roi_forest.sdf + roi_forest.yaml.

Why a generator and not hand-written SDF.  The forest used to be ~2100 lines of
hand-maintained SDF inherited from the legacy ``forest.sdf``, which meant that
changing its size was a day of careful editing and the metadata in the YAML could
drift out of step with the scenery it describes.  Everything here is derived from a
handful of numbers instead, so "double the area" is one flag.

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

Density, not just extent.  Doubling the *area* means scaling the radius by sqrt(2)
AND doubling the counts; scaling positions alone would halve the density and the
forest would read as a different place.  The defaults below hold trees per square
metre constant to four significant figures against the legacy 100 trees in r=104.7.

    python3 tools/gen_roi_forest.py --mission-radius 85 --scenery-radius 148 \
            --oak 120 --pine 80 --fires 40 --seed 20251007
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

OAK_URI = "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Oak Tree"
PINE_URI = "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Pine Tree"

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

    The legacy forest was r=104.7 of scenery around a 60 m mission ring, with 100
    trees, 153 grass tiles and 36 boundary posts.  Doubling the area scales both
    radii by sqrt(2) and the counts by 2.
    """

    mission_radius_m: float = 85.0        # 60 * sqrt(2) = 84.85; (85/60)^2 = 2.007
    scenery_radius_m: float = 148.0       # (148/104.7)^2 = 1.998
    grass_pitch_m: float = 14.83          # 15 m tiles with 0.17 m overlap: no seams
    grass_tile_m: float = 15.0
    grass_offset_x_m: float = 2.71574     # the legacy grid's phase, kept so the
    grass_offset_y_m: float = -2.80395    # texture does not shift under the trees
    oak_inside: int = 30
    oak_outside: int = 40
    oak_outer_extra: int = 50
    pine_inside: int = 40
    pine_outside: int = 40
    fire_count: int = 40
    fire_z_m: float = 0.40
    boundary_arc_m: float = 10.47         # the post spacing of the legacy 60 m ring
    min_tree_spacing_m: float = 2.5
    min_fire_spacing_m: float = 5.0
    min_fire_tree_m: float = 3.0
    seed: int = 20251007

    @property
    def oak_count(self) -> int:
        return self.oak_inside + self.oak_outside + self.oak_outer_extra

    @property
    def pine_count(self) -> int:
        return self.pine_inside + self.pine_outside

    @property
    def tree_count(self) -> int:
        return self.oak_count + self.pine_count


# -- geometry ------------------------------------------------------------------

def grass_tiles(p: ForestParams) -> list[tuple[str, float, float]]:
    """Grass tile centres on the legacy grid, clipped to a disc.

    Named by Chebyshev ring so all six legacy name families survive -- muscle memory
    in the Gazebo entity tree is worth more than tidier names.
    """
    reach = int(math.ceil(p.scenery_radius_m / p.grass_pitch_m)) + 1
    cells: list[tuple[int, int, int, float, float]] = []
    for i in range(-reach, reach + 1):
        for j in range(-reach, reach + 1):
            x = p.grass_offset_x_m + i * p.grass_pitch_m
            y = p.grass_offset_y_m + j * p.grass_pitch_m
            if math.hypot(x, y) <= p.scenery_radius_m:
                cells.append((max(abs(i), abs(j)), i, j, x, y))
    cells.sort(key=lambda c: (c[0], c[1], c[2]))

    tiles: list[tuple[str, float, float]] = []
    counter = 0
    for ring, i, j, x, y in cells:
        if ring == 0:
            name = "grasspatch"
        elif ring == 1:
            name, counter = f"grasspatch_{counter}", counter + 1
        elif ring == 2:
            name, counter = f"grasspatch_ext_{counter}", counter + 1
        elif ring == 3:
            name = f"grasspatch_fill_{i}_{j}"
        elif ring <= 5:
            name = f"grasspatch_outer_{i}_{j}"
        else:
            name = f"grassfield_{i}_{j}"
        tiles.append((name, x, y))
    return tiles


def ground_extent(p: ForestParams, tiles: list[tuple[str, float, float]]) -> list[float]:
    """The grass bounding box: tile centres grown by half a tile.

    This is what ``terrain_patch()`` clips the GCS ground visual against, so it must
    describe where the grass actually *is*, not where the scenery radius says it is.
    """
    half = p.grass_tile_m / 2.0
    xs = [x for _, x, _ in tiles]
    ys = [y for _, _, y in tiles]
    return [round(min(xs) - half, 2), round(max(xs) + half, 2),
            round(min(ys) - half, 2), round(max(ys) + half, 2)]


def boundary_posts(p: ForestParams) -> list[tuple[str, float, float]]:
    """Posts on the mission ring, keeping the legacy arc spacing."""
    count = max(3, round(2.0 * math.pi * p.mission_radius_m / p.boundary_arc_m))
    posts = [("boundary_origin", 0.0, 0.0)]
    for k in range(count):
        theta = 2.0 * math.pi * k / count
        posts.append((f"boundary_{k:02d}",
                      p.mission_radius_m * math.cos(theta),
                      p.mission_radius_m * math.sin(theta)))
    return posts


def _sample(rng: random.Random, r_inner: float, r_outer: float) -> tuple[float, float]:
    """Area-uniform point in a disc or annulus."""
    r = math.sqrt(rng.random() * (r_outer ** 2 - r_inner ** 2) + r_inner ** 2)
    theta = rng.uniform(0.0, 2.0 * math.pi)
    return r * math.cos(theta), r * math.sin(theta)


def _place(rng: random.Random, count: int, r_inner: float, r_outer: float,
           taken: list[tuple[float, float]], min_spacing_m: float,
           label: str) -> list[tuple[float, float]]:
    out: list[tuple[float, float]] = []
    for _ in range(count):
        for _attempt in range(2000):
            x, y = _sample(rng, r_inner, r_outer)
            if any(math.hypot(x - tx, y - ty) < min_spacing_m for tx, ty in taken):
                continue
            taken.append((x, y))
            out.append((x, y))
            break
        else:
            raise RuntimeError(
                f"could not place {label} {len(out) + 1} of {count} between "
                f"r={r_inner} and r={r_outer} at {min_spacing_m} m spacing; "
                f"lower the count or the spacing")
    return out


def tree_positions(p: ForestParams,
                   rng: random.Random) -> list[tuple[str, str, float, float, float]]:
    """``(uri, name, x, y, yaw)`` in the order they are emitted.

    Emission order matches the legacy fragment, and the YAML trunk list must use the
    same order: ``uavsim/world.py`` reads it to keep placed targets out of canopies.
    """
    taken: list[tuple[float, float]] = []
    inside, outside = (0.0, p.mission_radius_m), (p.mission_radius_m, p.scenery_radius_m)

    pine_in = _place(rng, p.pine_inside, *inside, taken, p.min_tree_spacing_m, "pine")
    oak_out = _place(rng, p.oak_outside, *outside, taken, p.min_tree_spacing_m, "oak")
    pine_out = _place(rng, p.pine_outside, *outside, taken, p.min_tree_spacing_m, "pine")
    oak_in = _place(rng, p.oak_inside, *inside, taken, p.min_tree_spacing_m, "oak")
    oak_extra = _place(rng, p.oak_outer_extra, *outside, taken, p.min_tree_spacing_m, "oak")

    trees: list[tuple[str, str, float, float, float]] = []

    def add(uri: str, name: str, point: tuple[float, float]) -> None:
        trees.append((uri, name, point[0], point[1], rng.uniform(0.0, 2.0 * math.pi)))

    for i, pt in enumerate(pine_in, 1):
        add(PINE_URI, f"Pine_Tree_Inside_{i}", pt)
    for i, (oak, pine) in enumerate(zip(oak_out, pine_out), 1):   # alternating, as before
        add(OAK_URI, f"Oak_Tree_Outside_{i}", oak)
        add(PINE_URI, f"Pine_Tree_Outside_{i}", pine)
    for i, pt in enumerate(oak_in, 1):
        add(OAK_URI, f"Oak_Tree_Inside_{i}", pt)
    for i, pt in enumerate(oak_extra, 1):
        add(OAK_URI, f"Oak_Tree_Outer_Extra_{i}", pt)
    return trees


def fire_positions(p: ForestParams, trees: list[tuple[str, str, float, float, float]],
                   rng: random.Random) -> list[list[float]]:
    """Fire markers inside the mission ring, clear of trunks and of each other.

    These are ground truth, not scenery: the scenario decides whether to render them
    (``targets.fires``), and ``place_casualties`` keeps casualties away from them.
    """
    trunks = [(x, y) for _, _, x, y, _ in trees]
    placed: list[tuple[float, float]] = []
    for _ in range(p.fire_count):
        for _attempt in range(2000):
            x, y = _sample(rng, 0.0, p.mission_radius_m)
            if any(math.hypot(x - tx, y - ty) < p.min_fire_tree_m for tx, ty in trunks):
                continue
            if any(math.hypot(x - px, y - py) < p.min_fire_spacing_m for px, py in placed):
                continue
            placed.append((x, y))
            break
        else:
            raise RuntimeError(
                f"could not place fire {len(placed) + 1} of {p.fire_count} inside "
                f"{p.mission_radius_m} m; lower --fires")
    return [[round(x, 2), round(y, 2), p.fire_z_m] for x, y in placed]


# -- rendering -----------------------------------------------------------------

def _invocation(p: ForestParams) -> str:
    return (f"python3 tools/gen_roi_forest.py --mission-radius {p.mission_radius_m:g} "
            f"--scenery-radius {p.scenery_radius_m:g} \\\n"
            f"               --oak {p.oak_count} --pine {p.pine_count} "
            f"--fires {p.fire_count} --seed {p.seed}")


SCENERY_MODEL_NAME = "forest_scenery"


def render_sdf(p: ForestParams, tiles: list[tuple[str, float, float]],
               posts: list[tuple[str, float, float]],
               trees: list[tuple[str, str, float, float, float]]) -> str:
    """The fragment, with the bulk scenery nested inside one static parent model.

    Why nested.  The grass, trees and posts are ~565 entities, and Gazebo's entity
    tree lists everything the world contains in creation order.  Vehicles are created
    last, at runtime, so a flat fragment buries them under hundreds of near-identical
    `grasspatch_*` and `Oak_Tree_*` rows -- and a user looking for their own drone
    concludes it never spawned.  One collapsible parent turns 565 rows into 1.

    ``axes`` deliberately stays at world level: it carries the world's only navsat
    sensor, and nesting it would change that sensor's topic path, which the PX4
    backend depends on.
    """
    entities = len(tiles) + len(posts) + len(trees) + 1
    out = [f"""\
<!-- Forest region-of-interest scenery, centred on the world origin.
     GENERATED by tools/gen_roi_forest.py. Do not edit by hand. The exact
     regeneration command is in the header of roi_forest.yaml, which can carry the
     flags that an XML comment cannot; the generator's check mode asserts that the
     committed files still match it. Built from: mission_radius={p.mission_radius_m:g} m, scenery_radius={p.scenery_radius_m:g} m,
     oak={p.oak_count}, pine={p.pine_count}, fires={p.fire_count}, seed={p.seed}.
     Included by worlds/templates/base_world.sdf.j2. {entities} entities:
     {len(tiles)} grass tiles, {len(trees)} trees, {len(posts)} boundary posts, 1 axis marker.
     The grass, trees and posts are nested in one static '{SCENERY_MODEL_NAME}' model so
     that Gazebo's entity tree stays navigable: a flat list buries the runtime-spawned
     vehicles under hundreds of scenery rows. Their scoped names therefore read
     '{SCENERY_MODEL_NAME}::Oak_Tree_Inside_1'. The axis marker stays at world level
     because it carries the navsat sensor the PX4 backend needs.
     Declares no plane geometry and no ground plane: there is exactly one plane
     collision per world, in base_world.sdf.j2.
     Fire/casualty/ROI targets are placed by the scenario, see roi_forest.yaml. -->
"""]
    out.append(AXES_SDF)
    out.append(f'    <model name="{SCENERY_MODEL_NAME}">\n')
    out.append("      <static>true</static>\n")
    out.append("      <pose>0 0 0 0 0 0</pose>\n")
    for name, x, y in tiles:
        out.append(f"""\
      <include>
        <uri>model://grasspatch_visual</uri>
        <name>{name}</name>
        <pose>{x:.5f} {y:.5f} 0.00000 0 0 0</pose>
      </include>
""")
    for name, x, y in posts:
        out.append(f"""\
      <model name="{name}">
        <static>true</static>
        <pose>{x:.3f} {y:.3f} 1.0 0 0 0</pose>
        <link name="link">
          <collision name="collision">
            <geometry>
              <cylinder>
                <radius>0.20</radius>
                <length>2.0</length>
              </cylinder>
            </geometry>
          </collision>
          <visual name="visual">
            <geometry>
              <cylinder>
                <radius>0.20</radius>
                <length>2.0</length>
              </cylinder>
            </geometry>
          </visual>
        </link>
      </model>
""")
    for uri, name, x, y, yaw in trees:
        out.append(f"""\
      <include>
        <uri>{uri}</uri>
        <name>{name}</name>
        <pose>{x:.4f} {y:.4f} 0 0 0 {yaw:.4f}</pose>
      </include>
""")
    out.append("    </model>\n")
    return "".join(out)


def render_yaml(p: ForestParams, tiles: list[tuple[str, float, float]],
                posts: list[tuple[str, float, float]],
                trees: list[tuple[str, str, float, float, float]],
                fires: list[list[float]], extent: list[float]) -> str:
    entities = len(tiles) + len(posts) + len(trees) + 1
    lines = [
        "# Forest region-of-interest metadata.",
        "# GENERATED -- do not edit by hand. Regenerate with:",
        f"#   {_invocation(p).replace(chr(10), chr(10) + '#   ')}",
        "id: roi_forest",
        "description: Forest search area, doubled in area from the legacy forest.sdf.",
        "centre: [0.0, 0.0]",
        f"mission_radius_m: {p.mission_radius_m}          # the boundary posts in the scenery",
        f"scenery_radius_m: {p.scenery_radius_m}",
        f"entities: {entities}",
        f"tree_count: {len(trees)}",
        f"ground_extent: [{extent[0]}, {extent[1]}, {extent[2]}, {extent[3]}]"
        "   # xmin, xmax, ymin, ymax",
        "generated_by: tools/gen_roi_forest.py",
        f"generator_seed: {p.seed}",
        "generator_args: {"
        f"mission_radius_m: {p.mission_radius_m}, scenery_radius_m: {p.scenery_radius_m}, "
        f"oak: {p.oak_count},",
        f"                 pine: {p.pine_count}, fires: {p.fire_count}, "
        f"grass_pitch_m: {p.grass_pitch_m}}}",
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

def _split_oak(total: int) -> dict[str, int]:
    """Oak split 1:4:5 across inside / outside / outer-extra, as the legacy forest."""
    inside = round(total * 0.25)
    outside = round(total * 1 / 3)
    return {"oak_inside": inside, "oak_outside": outside,
            "oak_outer_extra": total - inside - outside}


def main(argv: list[str] | None = None) -> int:
    d = ForestParams()
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--mission-radius", type=float, default=d.mission_radius_m)
    ap.add_argument("--scenery-radius", type=float, default=d.scenery_radius_m)
    ap.add_argument("--grass-pitch", type=float, default=d.grass_pitch_m)
    ap.add_argument("--oak", type=int, default=d.oak_count)
    ap.add_argument("--pine", type=int, default=d.pine_count)
    ap.add_argument("--fires", type=int, default=d.fire_count)
    ap.add_argument("--seed", type=int, default=d.seed)
    ap.add_argument("--out-dir", type=Path, default=FRAGMENTS_DIR)
    ap.add_argument("--check", action="store_true",
                    help="regenerate in memory and byte-compare; exit 1 on a mismatch")
    args = ap.parse_args(argv)

    params = replace(
        d, mission_radius_m=args.mission_radius, scenery_radius_m=args.scenery_radius,
        grass_pitch_m=args.grass_pitch, pine_inside=args.pine // 2,
        pine_outside=args.pine - args.pine // 2, fire_count=args.fires, seed=args.seed,
        **_split_oak(args.oak))

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
