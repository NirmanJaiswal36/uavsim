"""The run manifest: the contract between this environment and a user's framework.

Everything a mission planner needs is here -- which vehicles exist, what they can
do, where to reach them, where the GCS and the ROI are in both ENU and WGS84, and
the ground-truth positions of the targets -- so that nobody has to parse SDF or
guess port numbers.
"""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any

from .adapter import Instance
from .world import BuiltWorld, enu_to_geo

SCHEMA_VERSION = 1


def build(world: BuiltWorld, instances: list[Instance],
          endpoints: dict[str, dict[str, Any]], *,
          bridge_config: Path | None = None, run_dir: Path | None = None) -> dict[str, Any]:
    origin = {"latitude_deg": world.info.latitude_deg,
              "longitude_deg": world.info.longitude_deg,
              "elevation_m": world.info.elevation_m}
    gcs_lat, gcs_lon = enu_to_geo(world.site_pose.x, world.site_pose.y, origin)

    return {
        "schema_version": SCHEMA_VERSION,
        "created": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "world": {
            "name": world.info.name,
            "sdf": str(world.info.sdf_path),
            "origin": origin,
            "note": "world ENU origin is the centre of the region of interest",
        },
        "roi": {
            "id": world.roi.get("id"),
            "centre_enu": [0.0, 0.0],
            "centre_geo": [origin["latitude_deg"], origin["longitude_deg"]],
            "mission_radius_m": world.roi.get("mission_radius_m"),
            "scenery_radius_m": world.roi.get("scenery_radius_m"),
        },
        "gcs": {
            "pose_enu": world.site_pose.as_list(),
            "geo": [gcs_lat, gcs_lon],
            "distance_to_roi_m": round((world.site_pose.x ** 2 + world.site_pose.y ** 2) ** 0.5, 1),
            "runway": {
                "length_m": world.site.runway_length_m,
                "width_m": world.site.runway_width_m,
                "heading": "toward the ROI (+X)",
            },
            "pads": world.site.pad_capacity,
        },
        "targets": {
            "fires": [_target(f[0], f[1], origin) for f in world.fires],
            "casualties": [_target(c[0], c[1], origin) for c in world.casualties],
        },
        "vehicles": [_vehicle(inst, world, endpoints.get(inst.name, {})) for inst in instances],
        "ros2": {
            "bridge_config": str(bridge_config) if bridge_config else None,
            "clock": f"/world/{world.info.name}/clock",
        },
        "run_dir": str(run_dir) if run_dir else None,
    }


def _target(x: float, y: float, origin: dict[str, float]) -> dict[str, Any]:
    lat, lon = enu_to_geo(x, y, origin)
    return {"enu": [x, y], "geo": [round(lat, 8), round(lon, 8)]}


def _vehicle(inst: Instance, world: BuiltWorld, endpoints: dict[str, Any]) -> dict[str, Any]:
    origin = {"latitude_deg": world.info.latitude_deg,
              "longitude_deg": world.info.longitude_deg,
              "elevation_m": world.info.elevation_m}
    lat, lon = enu_to_geo(inst.pose.x, inst.pose.y, origin)
    caps = inst.vehicle.capabilities(name=inst.name, world=world.info.name)
    return {
        "name": inst.name,
        "type": inst.vehicle.id,
        "airframe": inst.vehicle.airframe.id,
        "backend": inst.vehicle.backend.id,
        "index": inst.index,
        "sysid": inst.sysid,
        "spawn": {"enu": inst.pose.as_list(), "geo": [round(lat, 8), round(lon, 8)]},
        "capabilities": caps,
        "endpoints": endpoints,
        "run_dir": str(inst.run_dir),
    }


def write(manifest: dict[str, Any], path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(manifest, indent=2) + "\n")
    return path
