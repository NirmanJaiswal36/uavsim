"""Scenario files: the one thing a user of this environment edits."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

from .adapter import Pose
from .allocator import FleetRequest
from .registry import PKG_ROOT, Registry
from .site import DEFAULT_GCS_POSE, DEFAULT_SITE, SiteLayout
from .world import DEFAULT_ORIGIN, DEFAULT_ROI_TYPES, TargetSpec, WorldSpec

SCENARIOS_DIR = PKG_ROOT / "scenarios"


@dataclass
class Scenario:
    path: Path
    name: str
    world: WorldSpec
    fleet: list[FleetRequest]
    ros2: dict[str, Any] = field(default_factory=lambda: {"bridge": True, "clock": True})
    backends: dict[str, dict[str, Any]] = field(default_factory=dict)
    supervisor: str = "tmux"
    gazebo: dict[str, Any] = field(default_factory=lambda: {"headless": False, "verbosity": 3,
                                                            "start": True})
    logging: dict[str, Any] = field(default_factory=lambda: {"enabled": True, "team": None,
                                                             "rate_hz": 10.0})


def resolve_path(value: str | Path) -> Path:
    """Accept a path, or the bare name of a file in scenarios/."""
    path = Path(value)
    if path.exists():
        return path
    for candidate in (SCENARIOS_DIR / path.name, SCENARIOS_DIR / f"{path.name}.yaml"):
        if candidate.exists():
            return candidate
    raise FileNotFoundError(f"no scenario at {value!r}; looked in {SCENARIOS_DIR}")


def _seed(value: Any) -> int | None:
    """A fixed layout seed, or None for "a fresh one every run" (absent or 'random')."""
    if value is None or value == "random":
        return None
    return int(value)


def load(path: str | Path, registry: Registry | None = None) -> Scenario:
    path = resolve_path(path)
    data = yaml.safe_load(path.read_text()) or {}
    (registry or Registry()).validate(data, "scenario", path)

    world_data = data.get("world") or {}
    spacing = (data.get("spawn") or {}).get("spacing_m")
    site = SiteLayout(**{**DEFAULT_SITE.__dict__, "pad_spacing_m": spacing}) if spacing \
        else DEFAULT_SITE

    gcs_pose = DEFAULT_GCS_POSE
    if world_data.get("gcs_pose"):
        x, y, z, roll, pitch, yaw = world_data["gcs_pose"]
        gcs_pose = Pose(x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw)

    targets_data = data.get("targets") or {}
    rois = targets_data.get("rois") or {}
    targets = TargetSpec(
        roi_count=int(rois.get("count", 0)),
        roi_seed=_seed(rois.get("seed")),
        roi_types=tuple(rois.get("types") or DEFAULT_ROI_TYPES),
        roi_marker_dict=rois.get("marker_dict", "4x4_50"),
        roi_marker_size_m=float(rois.get("marker_size_m", 3.0)),
        roi_first_marker_id=int(rois.get("first_marker_id", 0)),
        roi_min_tree_m=float(rois.get("min_tree_m", 6.0)),
        roi_marker_min_tree_m=(float(rois["marker_min_tree_m"])
                               if rois.get("marker_min_tree_m") is not None else None),
        roi_positions=rois.get("positions"),
    )

    world = WorldSpec(
        name=world_data.get("name", path.stem.replace("-", "_")),
        roi=world_data.get("roi", "roi_forest"),
        gcs_pose=gcs_pose,
        site=site,
        origin={**DEFAULT_ORIGIN, **(world_data.get("origin") or {})},
        targets=targets,
    )

    fleet = [FleetRequest(
        vehicle_id=entry["type"],
        count=int(entry.get("count", 1)),
        name_prefix=entry.get("name_prefix"),
        backend_config=entry.get("backend_config"),
        payload_params=entry.get("payload_params"),
    ) for entry in data.get("fleet", [])]

    defaults = Scenario(path=path, name=data.get("name", path.stem), world=world, fleet=fleet)
    return Scenario(
        path=path,
        name=data.get("name", path.stem),
        world=world,
        fleet=fleet,
        ros2={**defaults.ros2, **(data.get("ros2") or {})},
        backends=data.get("backends") or {},
        supervisor=data.get("supervisor", "tmux"),
        gazebo={**defaults.gazebo, **(data.get("gazebo") or {})},
        logging={**defaults.logging, **(data.get("logging") or {})},
    )
