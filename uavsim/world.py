"""Builds the scenario world: base setup + forest + GCS site + search-and-rescue sites.

The world deliberately contains no vehicles.  That is the whole point of the
split: the same world file serves any fleet, and changing "four Iris" to "eight"
never touches SDF.
"""

from __future__ import annotations

import math
import random
import secrets
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import jinja2
import yaml

from .adapter import Pose, WorldInfo
from .registry import PKG_ROOT
from .site import DEFAULT_GCS_POSE, DEFAULT_SITE, SiteLayout, compose

WORLDS_DIR = PKG_ROOT / "worlds"
TEMPLATES_DIR = WORLDS_DIR / "templates"
FRAGMENTS_DIR = WORLDS_DIR / "fragments"

# CMAC, the origin the legacy forest world and its parameter files use
DEFAULT_ORIGIN = {"latitude_deg": -35.389138, "longitude_deg": 149.219188, "elevation_m": 580.0}

EARTH_RADIUS_M = 6378137.0

# The search-and-rescue sites an ROI can be. Each is a Jinja macro in rois.sdf.j2.
DEFAULT_ROI_TYPES = ("survivor", "wreckage", "tent", "crate", "signal_fire")

# Top-down footprint of each ROI prop as ``(x, y, radius)`` circles in the prop frame
# (+x is the prop's heading), each enclosing one or more visuals of the macro of the
# same name in rois.sdf.j2. Keep the two in step: the marker offset is derived from
# these, and a visual that pokes out of its circle ends up under the plate -- which is
# how the wreckage's scorch disc came to z-fight the marker.
PROP_FOOTPRINTS: dict[str, tuple[tuple[float, float, float], ...]] = {
    "survivor": ((0.0, 0.0, 0.50), (-0.55, 0.0, 0.42), (0.45, 0.0, 0.12),
                 (0.10, 0.42, 0.33)),
    "wreckage": ((0.0, 0.0, 3.20),                      # scorch; covers fuselage, wings
                 (-2.40, -0.30, 1.38)),                 # charred tail, behind
    "tent": ((0.0, 0.0, 1.72), (1.70, 0.0, 0.98)),      # body + pole, ground sheet
    "crate": ((0.0, 0.0, 0.90), (-2.10, 0.40, 1.90)),   # box + straps, canopy behind
    "signal_fire": ((0.0, 0.0, 1.10), (1.50, 0.0, 0.96),
                    (-0.75, 1.30, 0.96), (-0.75, -1.30, 0.96)),   # embers + 3 logs
}

MARKER_PROP_GAP_M = 0.5        # daylight between a prop's footprint and its plate
OAK_CANOPY_M = 5.0             # radius of the biggest canopy: the oaks in roi_forest
MARKER_CANOPY_GAP_M = 1.0      # daylight between that canopy and the plate's corner
MARKER_Z_M = 0.05              # above every ground-hugging visual (grass at 0, scorch 0.02)


def marker_offset_m(roi_type: str, marker_size_m: float) -> float:
    """How far along the prop's heading the plate's centre has to sit to clear it.

    The plate is axis-aligned in the world while the prop's yaw is random, so relative
    to the prop the plate can be at any rotation: it is cleared by a circle of half its
    diagonal. ``(d, 0)`` must then be that far plus the gap from every footprint circle.
    """
    clear = marker_size_m / math.sqrt(2.0) + MARKER_PROP_GAP_M
    offset = 0.0
    for cx, cy, r in PROP_FOOTPRINTS[roi_type]:
        reach = r + clear
        if abs(cy) < reach:
            offset = max(offset, cx + math.sqrt(reach * reach - cy * cy))
    return math.ceil(offset * 10.0) / 10.0


def marker_min_tree_m(marker_size_m: float) -> float:
    """Plate centre to any trunk: an oak's canopy, half the plate's diagonal, a metre."""
    return math.ceil((OAK_CANOPY_M + marker_size_m / math.sqrt(2.0) + MARKER_CANOPY_GAP_M)
                     * 10.0) / 10.0


@dataclass
class TargetSpec:
    roi_count: int = 0
    roi_seed: int | None = 0             # None: draw one with new_seed() before building
    roi_types: tuple[str, ...] = DEFAULT_ROI_TYPES
    roi_marker_dict: str = "4x4_50"
    roi_marker_size_m: float = 3.0         # 3 m plate, 2.4 m marker: readable to ~25 m
    roi_first_marker_id: int = 0
    roi_min_tree_m: float = 6.0          # prop centre to trunk
    roi_marker_min_tree_m: float | None = None   # None: marker_min_tree_m(size)
    roi_positions: list[list[float]] | None = None


@dataclass
class WorldSpec:
    name: str = "rescue"
    roi: str | None = "roi_forest"
    gcs_pose: Pose = DEFAULT_GCS_POSE
    site: SiteLayout = DEFAULT_SITE
    origin: dict[str, float] = field(default_factory=lambda: dict(DEFAULT_ORIGIN))
    targets: TargetSpec = field(default_factory=TargetSpec)


@dataclass
class RoiPlacement:
    """One search-and-rescue region of interest: a themed prop plus its ArUco plate.

    The plate sits ``marker_offset_m`` along the prop's heading rather than under it,
    so the prop never occludes the marker from a nadir camera (see marker_offset_m()).

    ``site`` is the neutral label the world uses for the entity names. It is a seeded
    shuffle of the indices, so neither the name nor the SDF order gives away which
    marker id or prop type is where.
    """

    id: int
    type: str
    aruco_id: int
    x: float
    y: float
    yaw: float
    marker_offset_m: float = 3.0
    site: int = 0

    @property
    def marker_x(self) -> float:
        return self.x + self.marker_offset_m * math.cos(self.yaw)

    @property
    def marker_y(self) -> float:
        return self.y + self.marker_offset_m * math.sin(self.yaw)


@dataclass
class BuiltWorld:
    info: WorldInfo
    roi: dict[str, Any]
    site_pose: Pose
    site: SiteLayout
    rois: list[RoiPlacement] = field(default_factory=list)
    targets: TargetSpec = field(default_factory=TargetSpec)


def load_roi(roi_id: str) -> dict[str, Any]:
    path = FRAGMENTS_DIR / f"{roi_id}.yaml"
    if not path.exists():
        raise FileNotFoundError(
            f"unknown roi {roi_id!r}: {path} not found "
            f"(expected a fragment in worlds/fragments/)")
    return yaml.safe_load(path.read_text())


def enu_to_geo(x: float, y: float, origin: dict[str, float]) -> tuple[float, float]:
    """Flat-earth ENU -> lat/lon, good to centimetres over a few kilometres."""
    lat0 = math.radians(origin["latitude_deg"])
    dlat = y / EARTH_RADIUS_M
    dlon = x / (EARTH_RADIUS_M * math.cos(lat0))
    return (origin["latitude_deg"] + math.degrees(dlat),
            origin["longitude_deg"] + math.degrees(dlon))


def mission_extent(roi: dict[str, Any]) -> tuple[float, float, float, float]:
    """The search area as ``(xmin, xmax, ymin, ymax)`` in world ENU."""
    xmin, xmax, ymin, ymax = (float(v) for v in roi["mission_extent"])
    return xmin, xmax, ymin, ymax


def gcs_standoff_m(gcs_pose: Pose, site: SiteLayout, roi: dict[str, Any]) -> float | None:
    """Open ground between the GCS's forest-facing edge and the mission extent.

    Measured from the centre of the apron's front edge, which is what faces the
    forest when the site points at it (``uavsim/site.py`` has the arithmetic).
    """
    if not roi:
        return None
    front = compose(gcs_pose, Pose(x=site.front_x_m, y=site.apron_y_m))
    xmin, xmax, ymin, ymax = mission_extent(roi)
    dx = max(xmin - front.x, 0.0, front.x - xmax)
    dy = max(ymin - front.y, 0.0, front.y - ymax)
    return round(math.hypot(dx, dy), 1)


def new_seed() -> int:
    """A fresh layout seed, for a run that should not be predictable."""
    return secrets.randbelow(2 ** 31)


def place_points(roi: dict[str, Any], count: int, seed: int, *,
                 min_tree_m: float = 3.0, min_self_m: float = 8.0,
                 edge_margin_m: float = 5.0,
                 accept: Callable[[float, float, float], bool] | None = None,
                 explicit: list[list[float]] | None = None,
                 attempts: int = 500, label: str = "point") -> list[list[float]]:
    """Deterministic, area-uniform rejection sampling inside the mission extent.

    ``edge_margin_m`` keeps points off the boundary posts.  ``accept(x, y, yaw)`` is
    an extra test run last, so ``True`` means the point is placed.
    Returns ``[x, y, yaw]`` triples; same seed, same layout, every run.
    """
    if explicit:
        return [[p[0], p[1], p[2] if len(p) > 2 else 0.0] for p in explicit]
    if count <= 0:
        return []

    xmin, xmax, ymin, ymax = mission_extent(roi)
    xmin, xmax = xmin + edge_margin_m, xmax - edge_margin_m
    ymin, ymax = ymin + edge_margin_m, ymax - edge_margin_m
    trees = [tuple(t) for t in roi.get("trees", [])]
    rng = random.Random(seed)
    placed: list[list[float]] = []
    for _ in range(count):
        for _attempt in range(attempts):
            x, y = round(rng.uniform(xmin, xmax), 2), round(rng.uniform(ymin, ymax), 2)
            yaw = round(rng.uniform(0, 2 * math.pi), 3)
            if any(math.hypot(x - tx, y - ty) < min_tree_m for tx, ty in trees):
                continue
            if any(math.hypot(x - px, y - py) < min_self_m for px, py, _ in placed):
                continue
            if accept is not None and not accept(x, y, yaw):
                continue
            placed.append([x, y, yaw])
            break
        else:
            raise RuntimeError(
                f"could not place {label} {len(placed) + 1} of {count} inside "
                f"{roi['mission_extent']}; reduce the count or the clearances")
    return placed


ROI_MIN_SEPARATION_M = 15.0      # between props, and between plates


def place_rois(roi: dict[str, Any], spec: TargetSpec) -> list[RoiPlacement]:
    """Search-and-rescue sites: clear of trunks and of each other.

    The plate, not just the prop, is what has to be *seen*, so the canopy clearance
    is checked at the plate: it sits metres out along the prop's heading, and an
    oak's canopy is 5 m in radius. The plate must also stay inside the search area.
    """
    if spec.roi_count > 0 and spec.roi_seed is None and not spec.roi_positions:
        raise ValueError("roi_seed is unresolved; draw one with new_seed() first")
    types = tuple(spec.roi_types) or DEFAULT_ROI_TYPES
    trees = [tuple(t) for t in roi.get("trees", [])]
    xmin, xmax, ymin, ymax = mission_extent(roi)
    half = spec.roi_marker_size_m / 2.0
    min_tree = (spec.roi_marker_min_tree_m if spec.roi_marker_min_tree_m is not None
                else marker_min_tree_m(spec.roi_marker_size_m))
    edge = 5.0
    plates: list[tuple[float, float]] = []

    def plate_fits(x: float, y: float, yaw: float) -> bool:
        offset = marker_offset_m(types[len(plates) % len(types)], spec.roi_marker_size_m)
        mx, my = x + offset * math.cos(yaw), y + offset * math.sin(yaw)
        if not (xmin + edge + half <= mx <= xmax - edge - half
                and ymin + edge + half <= my <= ymax - edge - half):
            return False
        if any(math.hypot(mx - tx, my - ty) < min_tree for tx, ty in trees):
            return False
        if any(math.hypot(mx - px, my - py) < ROI_MIN_SEPARATION_M for px, py in plates):
            return False
        plates.append((mx, my))
        return True

    points = place_points(roi, spec.roi_count, spec.roi_seed or 0,
                          min_tree_m=spec.roi_min_tree_m, min_self_m=ROI_MIN_SEPARATION_M,
                          edge_margin_m=edge,
                          accept=None if spec.roi_positions else plate_fits,
                          explicit=spec.roi_positions, attempts=5000, label="roi")
    sites = list(range(len(points)))
    random.Random(f"sites-{spec.roi_seed}").shuffle(sites)
    return [RoiPlacement(id=i, type=types[i % len(types)],
                         aruco_id=spec.roi_first_marker_id + i,
                         x=point[0], y=point[1], yaw=point[2],
                         marker_offset_m=marker_offset_m(types[i % len(types)],
                                                         spec.roi_marker_size_m),
                         site=sites[i])
            for i, point in enumerate(points)]


def _env() -> jinja2.Environment:
    return jinja2.Environment(
        loader=jinja2.FileSystemLoader([str(TEMPLATES_DIR), str(FRAGMENTS_DIR)]),
        undefined=jinja2.StrictUndefined, trim_blocks=True, keep_trailing_newline=True)


def build(spec: WorldSpec, out_dir: Path, *, resource_paths: list[str] | None = None,
          plugin_paths: list[str] | None = None) -> BuiltWorld:
    """Render the world to ``out_dir/<name>.sdf`` and report what is in it."""
    env = _env()
    roi = load_roi(spec.roi) if spec.roi else {}

    pads = [compose(Pose(), slot) for slot in spec.site.pad_slots()]
    gcs_sdf = env.get_template("gcs_site.sdf.j2").render(
        site=spec.site, site_name="gcs_site", pads=pads, pose=spec.gcs_pose.as_sdf())

    rois = place_rois(roi, spec.targets) if roi else []
    rois_sdf = env.get_template("rois.sdf.j2").render(
        rois=sorted(rois, key=lambda r: r.site), marker_z=MARKER_Z_M,
        marker_size_m=spec.targets.roi_marker_size_m,
        marker_dict=spec.targets.roi_marker_dict) if rois else ""

    world_sdf = env.get_template("base_world.sdf.j2").render(
        world_name=spec.name,
        origin=spec.origin,
        roi_id=spec.roi or "none",
        roi_fragment=f"{spec.roi}.sdf" if spec.roi else None,
        gcs_site=gcs_sdf,
        rois=rois_sdf,
        terrain=terrain_patch(spec.gcs_pose, roi),
        camera_pose=_camera_pose(spec),
    )

    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"{spec.name}.sdf"
    out_path.write_text(world_sdf)

    info = WorldInfo(
        name=spec.name, sdf_path=out_path,
        latitude_deg=spec.origin["latitude_deg"],
        longitude_deg=spec.origin["longitude_deg"],
        elevation_m=spec.origin["elevation_m"],
        resource_paths=list(resource_paths or []),
        plugin_paths=list(plugin_paths or []),
    )
    return BuiltWorld(info=info, roi=roi, site_pose=spec.gcs_pose, site=spec.site, rois=rois,
                      targets=spec.targets)


def terrain_patch(gcs: Pose, roi: dict[str, Any], *, margin_m: float = 150.0,
                  clearance_m: float = 0.0, depth_m: float = 0.1) -> dict[str, float] | None:
    """Ground visual under the GCS and the corridor to the ROI, never over the ROI.

    The ROI's grass tiles have their top face at z=0; a second surface over them
    z-fights, which shows up as tiles flickering and vanishing with zoom. So the
    patch is clipped to end ``clearance_m`` before the ROI's ground extent. The
    default is flush: the ground plane has no visual, so any gap shows the sky
    colour as a bright strip along the forest edge.
    Returns None when the GCS is inside that extent and no clean patch exists.
    """
    xmin, xmax, ymin, ymax = roi.get("ground_extent") or [0.0, 0.0, 0.0, 0.0]
    x0, x1 = gcs.x - margin_m, gcs.x + margin_m
    y0, y1 = gcs.y - margin_m, gcs.y + margin_m
    if gcs.x < xmin:
        x1 = xmin - clearance_m
        y0, y1 = min(y0, ymin), max(y1, ymax)   # x already separates us; widen so the
    elif gcs.x > xmax:                          # corridor meets the ROI without bare
        x0 = xmax + clearance_m                 # wedges at the corners
        y0, y1 = min(y0, ymin), max(y1, ymax)
    elif gcs.y < ymin:
        y1 = ymin - clearance_m
    elif gcs.y > ymax:
        y0 = ymax + clearance_m
    else:
        return None
    if x1 <= x0 or y1 <= y0:
        return None
    return {"cx": round((x0 + x1) / 2, 2), "cy": round((y0 + y1) / 2, 2),
            "sx": round(x1 - x0, 2), "sy": round(y1 - y0, 2), "depth": depth_m}


# Where the GUI camera's eye sits in the site frame, as an offset from the apron
# centre, and the height of what it looks at. Chosen so that all 24 pads and the
# forest edge sit inside a conservative 60x35 degree frame -- see
# test_the_default_camera_frames_the_landing_pads.
CAMERA_EYE_OFFSET_M = (-70.0, -40.0, 28.0)
CAMERA_TARGET_Z_M = 1.5


def camera_eye_and_target(spec: WorldSpec) -> tuple[Pose, Pose]:
    """The GUI camera's eye and the point it looks at, both in world coordinates."""
    dx, dy, dz = CAMERA_EYE_OFFSET_M
    eye = compose(spec.gcs_pose, Pose(x=spec.site.apron_x_m + dx,
                                      y=spec.site.apron_y_m + dy, z=dz))
    target = compose(spec.gcs_pose, Pose(x=spec.site.apron_x_m, y=spec.site.apron_y_m,
                                         z=CAMERA_TARGET_Z_M))
    return eye, target


def _camera_pose(spec: WorldSpec) -> str:
    """Behind and above the apron, framing the pads with the ROI beyond them.

    Aimed at the apron geometrically rather than with a fixed pitch. The pads are
    only ~120 m from the eye but tens of metres below it, so a pitch chosen to look
    at the ROI instead puts them a couple of degrees *below the bottom of the frame*
    -- which is the bug where you bring the world up and cannot find your own
    vehicles. Deriving the aim from ``SiteLayout`` also means it survives moving the
    apron or rotating the whole GCS, which the old fixed pitch did not.
    """
    eye, target = camera_eye_and_target(spec)
    dx, dy, dz = target.x - eye.x, target.y - eye.y, target.z - eye.z
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(-dz, math.hypot(dx, dy))      # positive pitch looks down
    return f"{eye.x:.1f} {eye.y:.1f} {eye.z:.1f} 0 {pitch:.3f} {yaw:.3f}"
