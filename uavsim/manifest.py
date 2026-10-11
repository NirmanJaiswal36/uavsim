"""The run manifest: the contract between this environment and a user's framework.

Everything a mission planner needs is here -- which vehicles exist, what they can
do, where to reach them, where the GCS and the search area are in both ENU and
WGS84, and what the markers to look for are -- so that nobody has to parse SDF or
guess port numbers.

What is *not* here is where the targets are: finding them is the challenge. Their
ground truth goes to a separate judge-only file, built by ``ground_truth()``.
"""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any

from .adapter import Instance
from .world import BuiltWorld, RoiPlacement, enu_to_geo, gcs_standoff_m, mission_extent

# 2 added targets.rois; 3 made the ROI a square, dropped the runway; 4 moved every
# target position out to ground_truth()
SCHEMA_VERSION = 4


def build(world: BuiltWorld, instances: list[Instance],
          endpoints: dict[str, dict[str, Any]], *,
          bridge_config: Path | None = None, run_dir: Path | None = None) -> dict[str, Any]:
    origin = {"latitude_deg": world.info.latitude_deg,
              "longitude_deg": world.info.longitude_deg,
              "elevation_m": world.info.elevation_m}
    gcs_lat, gcs_lon = enu_to_geo(world.site_pose.x, world.site_pose.y, origin)
    xmin, xmax, ymin, ymax = mission_extent(world.roi) if world.roi else (0.0,) * 4
    corners = [(xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax)]

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
            "size_m": world.roi.get("size_m"),
            "mission_extent_enu": [xmin, xmax, ymin, ymax],
            "boundary_geo": [[round(v, 8) for v in enu_to_geo(x, y, origin)]
                             for x, y in corners],
        },
        "gcs": {
            "pose_enu": world.site_pose.as_list(),
            "geo": [gcs_lat, gcs_lon],
            "distance_to_roi_m": round((world.site_pose.x ** 2 + world.site_pose.y ** 2) ** 0.5, 1),
            "standoff_m": gcs_standoff_m(world.site_pose, world.site, world.roi),
            "pads": world.site.pad_capacity,
        },
        "targets": {
            "note": "positions are not published; finding the sites is the task",
            "roi_count": len(world.rois),
            "marker": {
                "dictionary": world.targets.roi_marker_dict,
                "size_m": world.targets.roi_marker_size_m,
                "ids": sorted(r.aruco_id for r in world.rois),
                "normal": "+Z (horizontal, faces a nadir camera)",
            },
        },
        "vehicles": [_vehicle(inst, world, endpoints.get(inst.name, {})) for inst in instances],
        "ros2": {
            "bridge_config": str(bridge_config) if bridge_config else None,
            "clock": f"/world/{world.info.name}/clock",
        },
        "run_dir": str(run_dir) if run_dir else None,
    }


def ground_truth(world: BuiltWorld) -> dict[str, Any]:
    """Where everything a team is meant to find actually is. For the judges only.

    Carries the layout seed too, so ``uavsim up --seed`` can replay the run.
    """
    origin = {"latitude_deg": world.info.latitude_deg,
              "longitude_deg": world.info.longitude_deg,
              "elevation_m": world.info.elevation_m}
    return {
        "schema_version": SCHEMA_VERSION,
        "world": world.info.name,
        "origin": origin,
        "roi_seed": world.targets.roi_seed,
        "rois": [_roi(r, origin, world.targets) for r in world.rois],
    }


def _roi(roi: RoiPlacement, origin: dict[str, float], spec: Any) -> dict[str, Any]:
    """One ROI and the marker that identifies it, in ENU and WGS84.

    ``aruco_id`` is reported separately from ``index`` on purpose: a detector reports
    the marker it read, not the ROI it belongs to, and ``first_marker_id`` lets two
    scenarios in the same world use disjoint marker sets. The mapping has to be
    explicit or a scoring script has to guess it.
    """
    lat, lon = enu_to_geo(roi.x, roi.y, origin)
    marker_lat, marker_lon = enu_to_geo(roi.marker_x, roi.marker_y, origin)
    return {
        "id": f"roi_{roi.id}",
        "index": roi.id,
        "entity": f"site_{roi.site}",
        "type": roi.type,
        "enu": [roi.x, roi.y],
        "yaw_rad": roi.yaw,
        "geo": [round(lat, 8), round(lon, 8)],
        "marker": {
            "dictionary": spec.roi_marker_dict,
            "aruco_id": roi.aruco_id,
            "size_m": spec.roi_marker_size_m,
            "model": f"aruco_marker_{roi.aruco_id}",
            "entity": f"site_{roi.site}_plate",
            "enu": [round(roi.marker_x, 2), round(roi.marker_y, 2)],
            "geo": [round(marker_lat, 8), round(marker_lon, 8)],
            "normal": "+Z (horizontal, faces a nadir camera)",
        },
    }


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
