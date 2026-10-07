"""Splits the regions of interest across the fleet, and deconflicts the fleet.

Shared by both example controllers on purpose: the assignment is the interesting part
of a swarm mission, and having one copy of it means the pymavlink and MAVROS examples
provably fly the same plan.  Like ``manifest_io``, this imports neither pymavlink nor
rclpy, so it is testable in CI with no simulator.

Collision avoidance here is *altitude layering and nothing else*.  Vehicles share
horizontal space freely, so the layers are the only thing keeping them apart and must
never be collapsed into a single cruise altitude.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

from manifest_io import Roi, Vehicle

STRATEGIES = ("nearest", "round_robin")


@dataclass(frozen=True)
class Leg:
    """One ROI visit: fly to it at cruise, drop to ``observe_alt_m``, hold, climb."""

    roi: Roi
    lat: float
    lon: float
    observe_alt_m: float


@dataclass(frozen=True)
class Tasking:
    vehicle: Vehicle
    cruise_alt_m: float
    legs: tuple[Leg, ...]


def cruise_altitudes(vehicles: Sequence[Vehicle], *, base_m: float = 15.0,
                     separation_m: float = 3.0) -> dict[str, float]:
    """One cruise level per vehicle: ``base + separation * i``.

    Keep ``separation_m`` small.  The ArUco plate is 2 m and the gimbal camera is
    640x480 with a 2.0 rad FOV, so the marker is 27 px across at 15 m and drops below
    OpenCV's ~3 px-per-module floor by 25 m.  Layers 10 m apart would put the top
    vehicle too high to read anything -- which is why the legs descend to a common
    observation altitude on station and only cruise in their own layer.
    """
    return {v.name: base_m + separation_m * i for i, v in enumerate(vehicles)}


def _nearest_order(vehicles: Sequence[Vehicle], rois: Sequence[Roi]) -> list[list[Roi]]:
    """Greedy sweep: each ROI goes to the closest vehicle that still has room.

    Capped at ``ceil(len(rois) / len(vehicles))`` each so one well-placed vehicle
    cannot take the whole forest, which keeps the legs short and the tracks from
    crossing.  Distance is measured from where the vehicle currently *is* in the plan:
    its spawn point first, then its last assigned ROI.
    """
    cap = math.ceil(len(rois) / len(vehicles))
    heads = [(v.spawn_enu[0], v.spawn_enu[1]) for v in vehicles]
    out: list[list[Roi]] = [[] for _ in vehicles]
    for roi in sorted(rois, key=lambda r: r.index):
        best = min(
            (i for i in range(len(vehicles)) if len(out[i]) < cap),
            key=lambda i: math.hypot(roi.enu[0] - heads[i][0], roi.enu[1] - heads[i][1]))
        out[best].append(roi)
        heads[best] = (roi.enu[0], roi.enu[1])
    return out


def assign(vehicles: Sequence[Vehicle], rois: Sequence[Roi], *,
           strategy: str = "nearest", base_alt_m: float = 15.0,
           alt_separation_m: float = 3.0, observe_alt_m: float = 15.0) -> list[Tasking]:
    """Give every ROI to exactly one vehicle.

    ``round_robin`` is ``roi[i] -> vehicle[i % n]``: fair, and trivially reproducible
    when you are comparing two runs.  ``nearest`` produces shorter missions.
    """
    if strategy not in STRATEGIES:
        raise ValueError(f"unknown strategy {strategy!r}; expected one of {STRATEGIES}")
    if not vehicles:
        raise ValueError("no vehicles to assign to")

    if strategy == "round_robin":
        buckets: list[list[Roi]] = [[] for _ in vehicles]
        for i, roi in enumerate(sorted(rois, key=lambda r: r.index)):
            buckets[i % len(vehicles)].append(roi)
    else:
        buckets = _nearest_order(vehicles, rois)

    levels = cruise_altitudes(vehicles, base_m=base_alt_m, separation_m=alt_separation_m)
    return [Tasking(vehicle=v, cruise_alt_m=levels[v.name],
                    legs=tuple(Leg(roi=r, lat=r.lat, lon=r.lon,
                                   observe_alt_m=observe_alt_m) for r in bucket))
            for v, bucket in zip(vehicles, buckets)]


def describe(taskings: Sequence[Tasking]) -> str:
    """The table both controllers print before they arm anything."""
    rows = [f"{'VEHICLE':<26} {'SYSID':>5} {'CRUISE':>7}  ROIS"]
    for tasking in taskings:
        legs = ", ".join(
            f"{leg.roi.id}({leg.roi.type}/aruco {leg.roi.marker.aruco_id})"
            for leg in tasking.legs) or "-"
        rows.append(f"{tasking.vehicle.name:<26} {tasking.vehicle.sysid:>5} "
                    f"{tasking.cruise_alt_m:>6.1f}m  {legs}")
    total = sum(len(t.legs) for t in taskings)
    rows.append(f"\n{total} ROIs across {len(taskings)} vehicles")
    return "\n".join(rows)
