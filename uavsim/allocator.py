"""Turns a fleet request into concrete, conflict-free instances.

Everything that must be unique across a run is decided here -- model names, ROS
namespaces, MAVLink system ids, per-backend instance numbers and spawn slots --
so that no adapter has to know what the other backends are doing.  Port numbers
are *derived* from ``backend_index`` inside each adapter, which is why the
per-backend counters are handed out sequentially from zero.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

from .adapter import Instance, Pose
from .registry import Registry, RegistryError
from .site import SiteLayout, compose, heading_to
from .spec import ResolvedVehicle


@dataclass(frozen=True)
class FleetRequest:
    vehicle_id: str
    count: int = 1
    name_prefix: str | None = None
    backend_config: dict[str, Any] | None = None
    payload_params: dict[str, Any] | None = None


class AllocationError(Exception):
    pass


def allocate(requests: Iterable[FleetRequest], *, registry: Registry, site: SiteLayout,
             gcs_pose: Pose, run_dir: Path, spacing_m: float | None = None,
             first_sysid: int = 1) -> list[Instance]:
    """Resolve every request and place each vehicle at the GCS.

    Vertical take-off airframes get apron pads, rolling ones get runway slots,
    and everyone is turned to face the ROI (the world origin).
    """
    resolved: list[tuple[FleetRequest, ResolvedVehicle]] = []
    for req in requests:
        if req.count < 1:
            raise AllocationError(f"{req.vehicle_id}: count must be >= 1")
        try:
            vehicle = registry.resolve(req.vehicle_id,
                                       backend_config=req.backend_config,
                                       payload_params=req.payload_params)
        except RegistryError as exc:
            raise AllocationError(str(exc)) from None
        resolved.extend([(req, vehicle)] * req.count)

    if spacing_m is not None:
        site = SiteLayout(**{**site.__dict__, "pad_spacing_m": spacing_m})

    singletons: dict[str, str] = {}
    for _, vehicle in resolved:
        for rp in vehicle.payloads:
            if not rp.spec.singleton:
                continue
            if rp.spec.id in singletons:
                raise AllocationError(
                    f"payload {rp.spec.id!r} is singleton (it uses a global topic or port) but "
                    f"is requested by more than one vehicle ({singletons[rp.spec.id]}, {vehicle.id})")
            singletons[rp.spec.id] = vehicle.id

    pads = iter(site.pad_slots())
    runway = iter(site.runway_slots())
    backend_counter: dict[str, int] = {}
    name_counter: dict[str, int] = {}
    used_names: set[str] = set()
    instances: list[Instance] = []

    for index, (req, vehicle) in enumerate(resolved):
        prefix = req.name_prefix or vehicle.id
        n = name_counter.get(prefix, 0)
        name_counter[prefix] = n + 1
        name = f"{prefix}_{n}"
        if name in used_names:
            raise AllocationError(f"duplicate model name {name!r}: use distinct name_prefix values")
        used_names.add(name)

        backend_index = backend_counter.get(vehicle.backend.id, 0)
        backend_counter[vehicle.backend.id] = backend_index + 1

        radius = vehicle.airframe.footprint_radius_m
        if vehicle.airframe.launch == "runway":
            local = next(runway, None)
            if local is None:
                raise AllocationError(
                    f"{name}: out of runway slots (site offers {site.runway_slot_count})")
        else:
            if not site.min_pad_spacing_ok(radius):
                raise AllocationError(
                    f"{name}: pad spacing {site.pad_spacing_m} m is below 2x the "
                    f"{vehicle.airframe.id} footprint ({radius} m)")
            local = next(pads, None)
            if local is None:
                raise AllocationError(
                    f"{name}: out of landing pads (site offers {site.pad_capacity})")

        world_pose = compose(gcs_pose, local)
        yaw = heading_to(world_pose)
        pose = Pose(x=world_pose.x, y=world_pose.y,
                    z=world_pose.z + vehicle.airframe.spawn_z_m, yaw=yaw)

        instances.append(Instance(
            name=name, vehicle=vehicle, index=index, backend_index=backend_index,
            sysid=first_sysid + index, pose=pose, run_dir=run_dir / name,
        ))

    if len(used_names) != len(instances):          # unreachable, kept as a guard
        raise AllocationError("internal: duplicate instance names")
    sysids = {i.sysid for i in instances}
    if len(sysids) != len(instances):
        raise AllocationError("internal: duplicate sysids")
    return instances
