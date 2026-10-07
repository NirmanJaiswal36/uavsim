"""Reads a run manifest into plain dataclasses the example controllers can use.

Why this exists.  Everything a controller needs -- which vehicles are up, their
sysids, where to reach each one over MAVLink, and the ground-truth position of every
region of interest in both ENU and WGS84 -- is already written to
``/tmp/uavsim/<world>/manifest.json`` by ``uavsim up``.  Hardcoding a port table is
the one thing that reliably breaks the moment the fleet changes, so nothing here does.

This module deliberately imports neither pymavlink nor rclpy: both example
controllers share it, and so do the tests, which must run in CI with no simulator and
no ROS.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

RUNS_DIR = Path(os.environ.get("UAVSIM_RUN_DIR", "/tmp/uavsim"))


@dataclass(frozen=True)
class Marker:
    """The fiducial that identifies one ROI."""

    aruco_id: int
    dictionary: str
    size_m: float
    enu: tuple[float, float]
    geo: tuple[float, float]


@dataclass(frozen=True)
class Roi:
    """One search-and-rescue site. ``geo`` is what a controller flies to."""

    index: int
    id: str
    type: str
    enu: tuple[float, float]
    geo: tuple[float, float]
    yaw_rad: float
    marker: Marker

    @property
    def lat(self) -> float:
        return self.geo[0]

    @property
    def lon(self) -> float:
        return self.geo[1]


@dataclass(frozen=True)
class Vehicle:
    name: str
    sysid: int
    backend: str                        # "ardupilot" | "px4" | "static"
    index: int
    mavlink: str | None                 # pymavlink connection string, or None
    extra_out: tuple[str, ...] = ()     # spare MAVProxy outputs; MAVROS takes one
    spawn_enu: tuple[float, float, float] = (0.0, 0.0, 0.0)
    spawn_geo: tuple[float, float] = (0.0, 0.0)


@dataclass(frozen=True)
class Run:
    """One running simulation, as the manifest describes it."""

    world: str
    origin: tuple[float, float, float]      # lat, lon, elevation -- the ROI centre
    mission_radius_m: float
    vehicles: tuple[Vehicle, ...]
    rois: tuple[Roi, ...]
    manifest_path: Path
    schema_version: int = 0
    raw: dict[str, Any] = field(default_factory=dict, repr=False)


def manifest_path(world_or_path: str | Path) -> Path:
    """A world name, a run directory, or a path straight to the manifest."""
    candidate = Path(world_or_path)
    if candidate.is_file():
        return candidate
    if (candidate / "manifest.json").is_file():
        return candidate / "manifest.json"
    return RUNS_DIR / str(world_or_path) / "manifest.json"


def parse(data: dict[str, Any], path: Path | None = None) -> Run:
    """Build a ``Run`` from already-loaded manifest data (what the tests use)."""
    world = data.get("world") or {}
    origin = world.get("origin") or {}
    roi = data.get("roi") or {}

    vehicles = []
    for entry in data.get("vehicles", []):
        endpoints = entry.get("endpoints") or {}
        spawn = entry.get("spawn") or {}
        enu = list(spawn.get("enu") or [0.0, 0.0, 0.0])[:3]
        enu += [0.0] * (3 - len(enu))
        geo = list(spawn.get("geo") or [0.0, 0.0])[:2]
        vehicles.append(Vehicle(
            name=entry["name"],
            sysid=int(entry.get("sysid", 0)),
            backend=str(entry.get("backend", "")),
            index=int(entry.get("index", 0)),
            # PX4 reports only mavlink_udp; ArduPilot reports both
            mavlink=endpoints.get("mavlink") or endpoints.get("mavlink_udp"),
            extra_out=tuple(endpoints.get("extra_out") or ()),
            spawn_enu=(enu[0], enu[1], enu[2]),
            spawn_geo=(geo[0], geo[1]),
        ))

    rois = []
    for entry in (data.get("targets") or {}).get("rois", []):
        marker = entry.get("marker") or {}
        rois.append(Roi(
            index=int(entry.get("index", 0)),
            id=str(entry.get("id", "")),
            type=str(entry.get("type", "")),
            enu=tuple(entry.get("enu") or (0.0, 0.0)),            # type: ignore[arg-type]
            geo=tuple(entry.get("geo") or (0.0, 0.0)),            # type: ignore[arg-type]
            yaw_rad=float(entry.get("yaw_rad", 0.0)),
            marker=Marker(
                aruco_id=int(marker.get("aruco_id", -1)),
                dictionary=str(marker.get("dictionary", "")),
                size_m=float(marker.get("size_m", 0.0)),
                enu=tuple(marker.get("enu") or (0.0, 0.0)),       # type: ignore[arg-type]
                geo=tuple(marker.get("geo") or (0.0, 0.0)),       # type: ignore[arg-type]
            ),
        ))

    return Run(
        world=str(world.get("name", "")),
        origin=(float(origin.get("latitude_deg", 0.0)),
                float(origin.get("longitude_deg", 0.0)),
                float(origin.get("elevation_m", 0.0))),
        mission_radius_m=float(roi.get("mission_radius_m", 0.0)),
        vehicles=tuple(vehicles),
        rois=tuple(rois),
        manifest_path=path or Path(),
        schema_version=int(data.get("schema_version", 0)),
        raw=data,
    )


def load_run(world_or_path: str | Path) -> Run:
    """Read the manifest of a running simulation."""
    path = manifest_path(world_or_path)
    if not path.is_file():
        raise FileNotFoundError(
            f"no run manifest at {path}. Start the simulation first:\n"
            f"  ./bin/uavsim up scenarios/forest_rescue.yaml\n"
            f"or point UAVSIM_RUN_DIR at the run directory you want.")
    return parse(json.loads(path.read_text()), path)


def ardupilot(run: Run) -> tuple[Vehicle, ...]:
    return tuple(v for v in run.vehicles if v.backend == "ardupilot" and v.mavlink)


def px4(run: Run) -> tuple[Vehicle, ...]:
    return tuple(v for v in run.vehicles if v.backend == "px4")


def fcu_url(vehicle: Vehicle) -> str:
    """A pymavlink endpoint translated into MAVROS's ``fcu_url`` spelling.

    MAVROS binds the local port and learns the remote address from the first packet it
    receives, so a MAVProxy ``--out host:port`` becomes ``udp://:port@``.  Prefer a
    dedicated ``extra_out``: the main ``mavlink`` port is a single UDP socket that only
    one process can bind, so sharing it with the pymavlink example means one of the two
    silently gets nothing.  Without MAVProxy (headless runs) there is only SITL's own
    TCP port, and that too admits a single client.
    """
    if vehicle.extra_out:
        return f"udp://:{vehicle.extra_out[0].rsplit(':', 1)[1]}@"
    if vehicle.mavlink and vehicle.mavlink.startswith(("udp:", "udpin:", "udpout:")):
        return f"udp://:{vehicle.mavlink.rsplit(':', 1)[1]}@"
    if vehicle.mavlink and vehicle.mavlink.startswith("tcp:"):
        return "tcp://" + vehicle.mavlink[len("tcp:"):]
    raise RuntimeError(
        f"{vehicle.name}: no usable MAVLink endpoint in the manifest; add "
        f"backends.ardupilot.extra_out to the scenario")
