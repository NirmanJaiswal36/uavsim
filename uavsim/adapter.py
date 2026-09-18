"""The interface every autopilot backend implements.

An adapter *describes* work; it never performs it.  It returns ProcessSpecs that
the supervisor runs, SDF variables that the renderer substitutes, and endpoints
that land in the run manifest.  That is what keeps vehicle-specific logic out of
the core: the orchestrator only ever calls the methods below.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from .spec import ResolvedVehicle


@dataclass(frozen=True)
class Pose:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0          # radians

    def as_sdf(self) -> str:
        return (f"{self.x:.4f} {self.y:.4f} {self.z:.4f} "
                f"{self.roll:.4f} {self.pitch:.4f} {self.yaw:.4f}")

    def as_list(self) -> list[float]:
        return [self.x, self.y, self.z, self.roll, self.pitch, self.yaw]


@dataclass(frozen=True)
class WorldInfo:
    """What a backend needs to know about the world it is joining."""

    name: str
    sdf_path: Path
    latitude_deg: float
    longitude_deg: float
    elevation_m: float
    resource_paths: list[str] = field(default_factory=list)
    plugin_paths: list[str] = field(default_factory=list)

    @property
    def home_string(self) -> str:
        """``lat,lon,alt,heading`` -- ArduPilot's ``-l``.

        Always the world origin: ArduPilotPlugin reports WorldPose (NED relative
        to the world origin), so a per-vehicle home would double-count the spawn
        offset and put the EKF tens of metres out.
        """
        return f"{self.latitude_deg},{self.longitude_deg},{self.elevation_m},0"


@dataclass
class Instance:
    """One concrete vehicle in a run."""

    name: str                     # gz model name, also the ROS namespace
    vehicle: ResolvedVehicle
    index: int                    # global index across the whole fleet
    backend_index: int            # per-backend index -> SITL instance number
    sysid: int                    # globally unique MAVLink system id
    pose: Pose
    run_dir: Path
    extra: dict[str, Any] = field(default_factory=dict)


@dataclass
class ProcessSpec:
    """A process the supervisor should run. ``cmd`` is argv, never a shell string."""

    name: str
    cmd: list[str]
    env: dict[str, str] = field(default_factory=dict)
    unset_env: list[str] = field(default_factory=list)
    cwd: Path | None = None
    delay_s: float = 0.0
    interactive: bool = False     # wants its own visible window (MAVProxy console)


@dataclass
class SpawnRequest:
    """Ask the orchestrator to create an entity in the running world."""

    name: str
    sdf: str | None = None        # rendered SDF string
    model_uri: str | None = None  # or a uri resolved through GZ_SIM_RESOURCE_PATH
    pose: Pose = Pose()


class AutopilotAdapter(ABC):
    """Base class for backend adapters.

    Subclasses live in ``registry/backends/<id>/adapter.py`` and are named in
    that backend's ``backend.yaml``.
    """

    #: backend id, must match the directory name
    id: str = ""

    def __init__(self, settings: dict[str, Any], world: WorldInfo):
        self.settings = settings
        self.world = world

    # -- checks --------------------------------------------------------------

    def preflight(self) -> list[str]:
        """Return human-readable problems (missing binaries, unbuilt SITL)."""
        return []

    def validate(self, vehicle: ResolvedVehicle) -> list[str]:
        """Return reasons this vehicle type cannot run on this backend."""
        return []

    # -- model ---------------------------------------------------------------

    def sdf_vars(self, inst: Instance) -> dict[str, Any]:
        """Per-instance values injected into the airframe's binding template."""
        return {}

    def spawn_request(self, inst: Instance, sdf: str | None) -> SpawnRequest | None:
        """Entity to create. Return None if the backend spawns its own model."""
        if sdf is None:
            uri = inst.vehicle.binding.model_uri
            return SpawnRequest(name=inst.name, model_uri=uri, pose=inst.pose)
        return SpawnRequest(name=inst.name, sdf=sdf, pose=inst.pose)

    # -- processes -----------------------------------------------------------

    def shared_processes(self, instances: list[Instance]) -> list[ProcessSpec]:
        """Processes started once per run, whatever the vehicle count."""
        return []

    @abstractmethod
    def processes(self, inst: Instance) -> list[ProcessSpec]:
        """Processes for one vehicle."""

    def wait_ready(self, inst: Instance, timeout_s: float) -> bool:
        """Block until the autopilot is talking, or time out."""
        return True

    def stop_hints(self, inst: Instance) -> list[str]:
        """Exact process names (``pkill -x``) belonging to this instance."""
        return []

    # -- reporting -----------------------------------------------------------

    def endpoints(self, inst: Instance) -> dict[str, Any]:
        """Connection info for the manifest (MAVLink ports, DDS namespace, ...)."""
        return {}
