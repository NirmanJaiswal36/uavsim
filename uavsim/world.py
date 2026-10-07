"""Builds the scenario world: base setup + ROI scenery + GCS site + targets.

The world deliberately contains no vehicles.  That is the whole point of the
split: the same world file serves any fleet, and changing "three Iris" to "six
Iris and a VTOL" never touches SDF.
"""

from __future__ import annotations

import math
import random
from collections.abc import Sequence
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


@dataclass
class TargetSpec:
    fires: bool = True
    fire_lights: bool = False
    casualty_count: int = 0
    casualty_seed: int = 0
    casualty_positions: list[list[float]] | None = None
    roi_count: int = 0
    roi_seed: int = 0
    roi_types: tuple[str, ...] = DEFAULT_ROI_TYPES
    roi_marker_dict: str = "4x4_50"
    roi_marker_size_m: float = 2.0
    roi_first_marker_id: int = 0
    roi_min_tree_m: float = 6.0          # clears an oak canopy, so the plate is visible
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

    The plate sits ``MARKER_OFFSET_M`` along the prop's heading rather than under it,
    so the prop never occludes the marker from a nadir camera.
    """

    id: int
    type: str
    aruco_id: int
    x: float
    y: float
    yaw: float

    MARKER_OFFSET_M = 3.0

    @property
    def marker_x(self) -> float:
        return self.x + self.MARKER_OFFSET_M * math.cos(self.yaw)

    @property
    def marker_y(self) -> float:
        return self.y + self.MARKER_OFFSET_M * math.sin(self.yaw)


@dataclass
class BuiltWorld:
    info: WorldInfo
    roi: dict[str, Any]
    fires: list[list[float]]
    casualties: list[list[float]]
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


def place_points(roi: dict[str, Any], count: int, seed: int, *,
                 min_tree_m: float = 3.0, min_self_m: float = 8.0,
                 avoid: Sequence[tuple[Sequence[Sequence[float]], float]] = (),
                 explicit: list[list[float]] | None = None,
                 attempts: int = 500, label: str = "point") -> list[list[float]]:
    """Deterministic ``r = R*sqrt(U)`` rejection sampling inside the mission radius.

    ``avoid`` is a sequence of ``(points, min_distance)`` pairs, so one pass can keep
    clear of fires, casualties and already-placed ROIs without three near-copies of
    this loop. Returns ``[x, y, yaw]`` triples; same seed, same layout, every run.

    The fire markers in the legacy world were themselves drawn from ``r = R*sqrt(U)``
    with a small seed, so without a fire clearance a casualty can land exactly on one
    (seed 42 does).
    """
    if explicit:
        return [[p[0], p[1], p[2] if len(p) > 2 else 0.0] for p in explicit]
    if count <= 0:
        return []

    radius = float(roi.get("mission_radius_m", 60.0))
    trees = [tuple(t) for t in roi.get("trees", [])]
    rng = random.Random(seed)
    placed: list[list[float]] = []
    for _ in range(count):
        for _attempt in range(attempts):
            r = radius * math.sqrt(rng.random())
            theta = rng.uniform(0, 2 * math.pi)
            x, y = r * math.cos(theta), r * math.sin(theta)
            if any(math.hypot(x - tx, y - ty) < min_tree_m for tx, ty in trees):
                continue
            if any(math.hypot(x - ax, y - ay) < keep_out
                   for points, keep_out in avoid for ax, ay, *_ in points):
                continue
            if any(math.hypot(x - px, y - py) < min_self_m for px, py, _ in placed):
                continue
            placed.append([round(x, 2), round(y, 2), round(rng.uniform(0, 2 * math.pi), 3)])
            break
        else:
            raise RuntimeError(
                f"could not place {label} {len(placed) + 1} of {count} inside "
                f"{radius} m; reduce the count or the clearances")
    return placed


def place_casualties(roi: dict[str, Any], spec: TargetSpec,
                     fires: list[list[float]] | None = None) -> list[list[float]]:
    """Casualties: inside the mission radius, 3 m from trunks, 5 m from fires, 8 m apart."""
    return place_points(roi, spec.casualty_count, spec.casualty_seed,
                        min_tree_m=3.0, min_self_m=8.0, avoid=[(fires or [], 5.0)],
                        explicit=spec.casualty_positions, label="casualty")


def place_rois(roi: dict[str, Any], spec: TargetSpec,
               fires: list[list[float]] | None = None,
               casualties: list[list[float]] | None = None) -> list[RoiPlacement]:
    """Search-and-rescue sites: clear of trunks, fires, casualties and each other.

    The trunk clearance is larger than a casualty's because an ROI carries an ArUco
    plate that has to be *seen*: under a canopy it never is. Separation between ROIs
    is larger too, so one camera frame cannot contain two markers and a detection can
    be attributed to a single site.
    """
    points = place_points(roi, spec.roi_count, spec.roi_seed,
                          min_tree_m=spec.roi_min_tree_m, min_self_m=15.0,
                          avoid=[(fires or [], 8.0), (casualties or [], 8.0)],
                          explicit=spec.roi_positions, attempts=2000, label="roi")
    types = tuple(spec.roi_types) or DEFAULT_ROI_TYPES
    return [RoiPlacement(id=i, type=types[i % len(types)],
                         aruco_id=spec.roi_first_marker_id + i,
                         x=point[0], y=point[1], yaw=point[2])
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

    fires = [list(f) for f in roi.get("fires", [])] if (spec.targets.fires and roi) else []
    casualties = place_casualties(roi, spec.targets, fires) if roi else []

    pads = [compose(Pose(), slot) for slot in spec.site.pad_slots()]
    gcs_sdf = env.get_template("gcs_site.sdf.j2").render(
        site=spec.site, site_name="gcs_site", pads=pads, pose=spec.gcs_pose.as_sdf())

    targets_sdf = ""
    if fires or casualties:
        targets_sdf = env.get_template("targets.sdf.j2").render(
            fires=fires, casualties=casualties, fire_lights=spec.targets.fire_lights)

    rois = place_rois(roi, spec.targets, fires, casualties) if roi else []
    rois_sdf = env.get_template("rois.sdf.j2").render(
        rois=rois, marker_size_m=spec.targets.roi_marker_size_m,
        marker_dict=spec.targets.roi_marker_dict) if rois else ""

    world_sdf = env.get_template("base_world.sdf.j2").render(
        world_name=spec.name,
        origin=spec.origin,
        roi_id=spec.roi or "none",
        roi_fragment=f"{spec.roi}.sdf" if spec.roi else None,
        gcs_site=gcs_sdf,
        targets=targets_sdf,
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
    return BuiltWorld(info=info, roi=roi, fires=fires, casualties=casualties,
                      site_pose=spec.gcs_pose, site=spec.site, rois=rois,
                      targets=spec.targets)


def terrain_patch(gcs: Pose, roi: dict[str, Any], *, margin_m: float = 150.0,
                  clearance_m: float = 10.0, depth_m: float = 0.1) -> dict[str, float] | None:
    """Ground visual under the GCS and the corridor to the ROI, never over the ROI.

    The ROI's grass patches are planes at z=0; a second surface over them
    z-fights, which shows up as patches flickering and vanishing with zoom. So
    the patch is clipped to end ``clearance_m`` before the ROI's ground extent.
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


def _camera_pose(spec: WorldSpec) -> str:
    """Behind and above the GCS, looking down the runway toward the ROI."""
    x = spec.gcs_pose.x - 120.0
    y = spec.gcs_pose.y - 90.0
    yaw = math.atan2(spec.gcs_pose.y + 90.0 - y, spec.gcs_pose.x + 120.0 - x)
    return f"{x:.1f} {y:.1f} 90 0 0.32 {yaw:.3f}"
