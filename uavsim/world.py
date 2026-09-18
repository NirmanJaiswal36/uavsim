"""Builds the scenario world: base setup + ROI scenery + GCS site + targets.

The world deliberately contains no vehicles.  That is the whole point of the
split: the same world file serves any fleet, and changing "three Iris" to "six
Iris and a VTOL" never touches SDF.
"""

from __future__ import annotations

import math
import random
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


@dataclass
class TargetSpec:
    fires: bool = True
    fire_lights: bool = False
    casualty_count: int = 0
    casualty_seed: int = 0
    casualty_positions: list[list[float]] | None = None


@dataclass
class WorldSpec:
    name: str = "rescue"
    roi: str | None = "roi_forest"
    gcs_pose: Pose = DEFAULT_GCS_POSE
    site: SiteLayout = DEFAULT_SITE
    origin: dict[str, float] = field(default_factory=lambda: dict(DEFAULT_ORIGIN))
    targets: TargetSpec = field(default_factory=TargetSpec)


@dataclass
class BuiltWorld:
    info: WorldInfo
    roi: dict[str, Any]
    fires: list[list[float]]
    casualties: list[list[float]]
    site_pose: Pose
    site: SiteLayout


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


def place_casualties(roi: dict[str, Any], spec: TargetSpec,
                     fires: list[list[float]] | None = None) -> list[list[float]]:
    """Deterministic placement inside the mission radius, clear of trees and fires.

    Returns ``[x, y, yaw]`` triples. Same seed, same layout, every run.

    The fire markers in the legacy world were themselves drawn from
    ``r = R*sqrt(U)`` with a small seed, so without the fire clearance below a
    casualty can land exactly on one (seed 42 does).
    """
    if spec.casualty_positions:
        return [[p[0], p[1], p[2] if len(p) > 2 else 0.0] for p in spec.casualty_positions]
    if spec.casualty_count <= 0:
        return []

    radius = float(roi.get("mission_radius_m", 60.0))
    trees = [tuple(t) for t in roi.get("trees", [])]
    rng = random.Random(spec.casualty_seed)
    placed: list[list[float]] = []
    for _ in range(spec.casualty_count):
        for _attempt in range(500):
            r = radius * math.sqrt(rng.random())
            theta = rng.uniform(0, 2 * math.pi)
            x, y = r * math.cos(theta), r * math.sin(theta)
            if any(math.hypot(x - tx, y - ty) < 3.0 for tx, ty in trees):
                continue
            if any(math.hypot(x - fx, y - fy) < 5.0 for fx, fy, *_ in (fires or [])):
                continue
            if any(math.hypot(x - px, y - py) < 8.0 for px, py, _ in placed):
                continue
            placed.append([round(x, 2), round(y, 2), round(rng.uniform(0, 2 * math.pi), 3)])
            break
        else:
            raise RuntimeError(
                f"could not place casualty {len(placed) + 1} of {spec.casualty_count} "
                f"inside {radius} m; reduce the count")
    return placed


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

    world_sdf = env.get_template("base_world.sdf.j2").render(
        world_name=spec.name,
        origin=spec.origin,
        roi_id=spec.roi or "none",
        roi_fragment=f"{spec.roi}.sdf" if spec.roi else None,
        gcs_site=gcs_sdf,
        targets=targets_sdf,
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
                      site_pose=spec.gcs_pose, site=spec.site)


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
    elif gcs.x > xmax:
        x0 = xmax + clearance_m
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
