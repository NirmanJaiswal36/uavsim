"""Registry data types.

Four independent things, deliberately kept apart:

    airframe   physical vehicle (links, rotors, onboard sensors, envelope)
    backend    autopilot software (ArduPilot SITL, PX4 SITL, none)
    binding    airframe x backend glue (joint names + motor constants + autopilot
               plugin) -- lives with the airframe because the joint names do
    payload    mountable sensor/actuator (gimbal camera, ...)

A *vehicle type* is the named combination a user asks for by name, e.g.
``ardupilot_iris_gimbal``.  ``ResolvedVehicle`` is that combination after the
registry has been read and merged; nothing downstream of the resolver looks at
raw YAML again.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


# ---------------------------------------------------------------------------
# raw registry entries
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class Binding:
    """How one airframe is driven by one backend."""

    backend: str
    overlay: Path | None = None
    model_uri: str | None = None
    vars: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class Airframe:
    id: str
    cls: str                      # multirotor | fixed_wing | vtol | helicopter
    launch: str                   # pad | runway
    dir: Path
    model_uri: str | None = None
    base_link: str | None = None
    imu_sensor: str | None = None
    geometry: dict[str, Any] = field(default_factory=dict)
    performance: dict[str, Any] = field(default_factory=dict)
    mounts: dict[str, Any] = field(default_factory=dict)
    bindings: dict[str, Binding] = field(default_factory=dict)
    description: str = ""

    @property
    def footprint_radius_m(self) -> float:
        return float(self.geometry.get("footprint_radius_m", 0.5))

    @property
    def spawn_z_m(self) -> float:
        return float(self.geometry.get("spawn_z_m", 0.0))


@dataclass(frozen=True)
class Backend:
    id: str
    dir: Path
    adapter: str                  # "<module>:<class>" relative to dir
    requires: dict[str, Any] = field(default_factory=dict)
    settings: dict[str, Any] = field(default_factory=dict)
    capabilities: dict[str, Any] = field(default_factory=dict)
    description: str = ""


@dataclass(frozen=True)
class Payload:
    id: str
    dir: Path
    fragment: Path | None = None
    mount: str | None = None
    params: dict[str, Any] = field(default_factory=dict)
    backend_fragments: dict[str, Path] = field(default_factory=dict)
    provides: list[dict[str, Any]] = field(default_factory=list)
    singleton: bool = False       # uses a global resource: at most one per run
    description: str = ""


@dataclass(frozen=True)
class VehicleType:
    id: str
    airframe: str
    backend: str
    payloads: list[dict[str, Any]] = field(default_factory=list)
    backend_config: dict[str, Any] = field(default_factory=dict)
    binding_vars: dict[str, Any] = field(default_factory=dict)
    description: str = ""


# ---------------------------------------------------------------------------
# resolved (merged) view
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class ResolvedPayload:
    id: str                       # instance-local id, e.g. "gimbal_cam"
    spec: Payload
    mount: str | None
    params: dict[str, Any]


@dataclass(frozen=True)
class ResolvedVehicle:
    """Everything about a vehicle *type*, with no per-instance values in it."""

    id: str
    airframe: Airframe
    backend: Backend
    binding: Binding
    payloads: list[ResolvedPayload] = field(default_factory=list)
    backend_config: dict[str, Any] = field(default_factory=dict)
    binding_vars: dict[str, Any] = field(default_factory=dict)

    # -- capability view -----------------------------------------------------

    def capabilities(self, name: str = "{name}", world: str = "{world}") -> dict[str, Any]:
        """Normalised capability dict.

        ``name``/``world`` are substituted into topic templates; leave them at
        their defaults to get the templates themselves.
        """
        af, be = self.airframe, self.backend
        perf = af.performance
        caps: dict[str, Any] = {
            "platform": {
                "class": af.cls,
                "hover": af.cls in ("multirotor", "vtol", "helicopter"),
                "vtol": af.cls == "vtol",          # transitions between hover and wing-borne
                "launch": af.launch,
                "cruise_mps": perf.get("cruise_mps"),
                "max_mps": perf.get("max_mps"),
                "endurance_min": perf.get("endurance_min"),
                "mass_kg": perf.get("mass_kg"),
                "max_payload_kg": perf.get("max_payload_kg"),
                "footprint_radius_m": af.footprint_radius_m,
            },
            "autopilot": {
                "backend": be.id,
                "protocols": be.capabilities.get("protocols", []),
                "ros2_native": be.capabilities.get("ros2_native", False),
                "offboard": be.capabilities.get("offboard", []),
                "sim_time": be.capabilities.get("sim_time", "none"),
            },
            "sensors": [],
            "actuators": [],
        }
        for rp in self.payloads:
            for item in rp.spec.provides:
                entry = _render_topics(item, name=name, payload=rp.id, world=world)
                entry["payload"] = rp.id
                role = entry.pop("role", None) or (
                    "actuator" if entry["kind"].split(".")[0] in _ACTUATOR_KINDS else "sensor")
                caps["actuators" if role == "actuator" else "sensors"].append(entry)
        if af.imu_sensor:
            caps["sensors"].append({"id": "imu", "kind": "imu", "payload": None,
                                    "gz_topic": None, "ros": None})
        return caps


_ACTUATOR_KINDS = {"gimbal", "gripper", "winch", "dropper", "servo"}


def _render_topics(item: dict[str, Any], *, name: str, payload: str, world: str) -> dict[str, Any]:
    """Substitute {name}/{payload}/{world} in every string of a ``provides`` entry."""

    def sub(value: Any) -> Any:
        if isinstance(value, str):
            return value.format(name=name, payload=payload, world=world, axis="{axis}")
        if isinstance(value, dict):
            return {k: sub(v) for k, v in value.items()}
        if isinstance(value, list):
            return [sub(v) for v in value]
        return value

    return {k: sub(v) for k, v in item.items()}
