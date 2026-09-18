"""Ties the pieces together: world -> allocation -> spawn -> autopilots -> ROS 2.

The orchestrator is deliberately vehicle-agnostic.  It never asks "is this
ArduPilot?"; it asks the adapter for SDF variables, processes, readiness and
endpoints.  Adding a backend therefore means adding a directory under
registry/backends/, not editing this file.
"""

from __future__ import annotations

import os
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from . import bridge as bridge_mod
from . import manifest as manifest_mod
from . import world as world_mod
from .adapter import AutopilotAdapter, Instance
from .allocator import allocate
from .registry import PKG_ROOT, Registry
from .scenario import Scenario
from .sdf import render_model, render_string
from .spawner import Spawner
from .supervisor import Supervisor

RUNS_DIR = Path(os.environ.get("UAVSIM_RUN_DIR", "/tmp/uavsim"))
BUILD_WORLDS = PKG_ROOT / "build" / "worlds"


class OrchestratorError(Exception):
    pass


@dataclass
class Plan:
    """Everything decided before a single process starts."""

    scenario: Scenario
    world: world_mod.BuiltWorld
    instances: list[Instance]
    adapters: dict[str, AutopilotAdapter]
    run_dir: Path
    problems: list[str] = field(default_factory=list)

    @property
    def resource_paths(self) -> list[str]:
        return self.world.info.resource_paths


def _run_dir(scenario: Scenario) -> Path:
    return RUNS_DIR / scenario.world.name


def plan(scenario: Scenario, registry: Registry | None = None, *,
         check_environment: bool = True) -> Plan:
    """Resolve, allocate and render everything, touching nothing outside build/."""
    registry = registry or Registry()
    run_dir = _run_dir(scenario)

    resource_paths = [str(PKG_ROOT / "models"), str(PKG_ROOT / "worlds"), str(BUILD_WORLDS)]
    built = world_mod.build(scenario.world, BUILD_WORLDS,
                            resource_paths=resource_paths,
                            plugin_paths=[str(PKG_ROOT / "build")])

    adapters: dict[str, AutopilotAdapter] = {}
    problems: list[str] = []
    for backend_id, backend in registry.backends.items():
        settings = {
            # whether the supervisor can give a process its own console, which
            # decides e.g. whether ArduPilot gets a MAVProxy at all
            "interactive": scenario.supervisor == "tmux",
            **backend.settings,
            **(scenario.backends.get(backend_id) or {}),
        }
        adapters[backend_id] = registry.adapter_class(backend_id)(settings, built.info)

    instances = allocate(scenario.fleet, registry=registry, site=scenario.world.site,
                         gcs_pose=scenario.world.gcs_pose, run_dir=run_dir)

    used_backends = {inst.vehicle.backend.id for inst in instances}
    for backend_id in sorted(used_backends):
        adapter = adapters[backend_id]
        extra = getattr(adapter, "resource_paths", None)
        if callable(extra):
            for path in extra():
                if path not in built.info.resource_paths:
                    built.info.resource_paths.append(path)
        if check_environment:
            problems.extend(adapter.preflight())

    seen_types: set[str] = set()
    for inst in instances:
        if inst.vehicle.id in seen_types:
            continue
        seen_types.add(inst.vehicle.id)
        problems.extend(adapters[inst.vehicle.backend.id].validate(inst.vehicle))

    return Plan(scenario=scenario, world=built, instances=instances, adapters=adapters,
                run_dir=run_dir, problems=problems)


def gazebo_env(plan_: Plan) -> dict[str, str]:
    """Resource and plugin paths, appended to whatever the user already exports."""
    def joined(key: str, values: list[str]) -> str:
        existing = os.environ.get(key, "")
        parts = [p for p in values if p]
        if existing:
            parts.append(existing)
        return os.pathsep.join(dict.fromkeys(parts))

    return {
        "GZ_SIM_RESOURCE_PATH": joined("GZ_SIM_RESOURCE_PATH", plan_.world.info.resource_paths),
        "GZ_SIM_SYSTEM_PLUGIN_PATH": joined("GZ_SIM_SYSTEM_PLUGIN_PATH",
                                            plan_.world.info.plugin_paths),
    }


def render_all(plan_: Plan) -> dict[str, str | None]:
    """Instance name -> rendered SDF (None when the backend spawns a plain uri)."""
    out: dict[str, str | None] = {}
    for inst in plan_.instances:
        adapter = plan_.adapters[inst.vehicle.backend.id]
        out[inst.name] = render_model(inst, world_name=plan_.world.info.name,
                                      backend_vars=adapter.sdf_vars(inst))
    return out


def up(plan_: Plan, *, wait_ready: bool = True, start_bridge: bool = True) -> dict[str, Any]:
    """Start Gazebo (unless already running), spawn the fleet, start the autopilots."""
    if plan_.problems:
        raise OrchestratorError("environment is not ready:\n  - " + "\n  - ".join(plan_.problems))

    scenario = plan_.scenario
    env = gazebo_env(plan_)
    supervisor = Supervisor(session=f"uavsim-{plan_.world.info.name}",
                            log_dir=plan_.run_dir / "logs", kind=scenario.supervisor)
    supervisor.start_session()

    spawner = Spawner(world=plan_.world.info.name)
    if scenario.gazebo.get("start", True):
        supervisor.start_gazebo(plan_.world.info.sdf_path,
                                headless=bool(scenario.gazebo.get("headless", False)),
                                verbosity=int(scenario.gazebo.get("verbosity", 3)), env=env)
    if not spawner.wait_for_world(timeout_s=120.0):
        raise OrchestratorError(
            f"world {plan_.world.info.name!r} never came up; check the gazebo window/log in "
            f"{supervisor.log_dir}")

    sdfs = render_all(plan_)
    for inst in plan_.instances:
        adapter = plan_.adapters[inst.vehicle.backend.id]
        sdf = sdfs[inst.name]
        if sdf is None and inst.vehicle.binding.model_uri:
            uri = render_string(inst.vehicle.binding.model_uri, {"vars": inst.vehicle.binding_vars})
            request = adapter.spawn_request(inst, None)
            if request is not None:
                request.model_uri = uri
        else:
            request = adapter.spawn_request(inst, sdf)
        if request is not None:
            spawner.spawn(request)

    endpoints: dict[str, dict[str, Any]] = {}
    for backend_id in sorted({i.vehicle.backend.id for i in plan_.instances}):
        adapter = plan_.adapters[backend_id]
        members = [i for i in plan_.instances if i.vehicle.backend.id == backend_id]
        for spec in adapter.shared_processes(members):
            supervisor.run(spec)
        for inst in members:
            inst.run_dir.mkdir(parents=True, exist_ok=True)
            for spec in adapter.processes(inst):
                supervisor.run(spec)
            endpoints[inst.name] = adapter.endpoints(inst)

    ready: dict[str, bool] = {}
    if wait_ready:
        for inst in plan_.instances:
            adapter = plan_.adapters[inst.vehicle.backend.id]
            timeout = float(adapter.settings.get("ready_timeout_s", 60.0))
            ready[inst.name] = adapter.wait_ready(inst, timeout)

    bridge_path = None
    if scenario.ros2.get("bridge", True):
        bridge_path = bridge_mod.write_config(
            plan_.instances, plan_.world.info.name, plan_.run_dir / "bridge.yaml",
            clock=scenario.ros2.get("clock", True))
        if start_bridge:
            supervisor.run(_bridge_process(bridge_path))

    data = manifest_mod.build(plan_.world, plan_.instances, endpoints,
                              bridge_config=bridge_path, run_dir=plan_.run_dir)
    data["ready"] = ready
    manifest_path = manifest_mod.write(data, plan_.run_dir / "manifest.json")

    return {"manifest": data, "manifest_path": manifest_path, "supervisor": supervisor,
            "bridge_config": bridge_path, "ready": ready}


def add(plan_: Plan, requests, *, wait_ready: bool = True) -> dict[str, Any]:
    """Add vehicles to a run that is already up.

    Existing vehicles are read back from the manifest so the new ones continue
    the same name, sysid and per-backend instance sequences -- otherwise the
    newcomer would bind an FDM port that is already carrying someone's traffic.
    """
    manifest_path = plan_.run_dir / "manifest.json"
    if not manifest_path.exists():
        raise OrchestratorError(
            f"no run to add to: {manifest_path} does not exist (use 'uavsim up' first)")
    import json

    existing = json.loads(manifest_path.read_text())
    taken_names = {v["name"] for v in existing["vehicles"]}
    first_sysid = max((v["sysid"] for v in existing["vehicles"]), default=0) + 1
    backend_offset: dict[str, int] = {}
    prefix_offset: dict[str, int] = {}
    for vehicle in existing["vehicles"]:
        backend = vehicle["backend"]
        backend_offset[backend] = backend_offset.get(backend, 0) + 1
        prefix = vehicle["name"].rsplit("_", 1)[0]
        prefix_offset[prefix] = prefix_offset.get(prefix, 0) + 1

    # keep the already-placed vehicles in the allocation so their pads stay taken
    from .allocator import FleetRequest, allocate
    previous = [FleetRequest(v["type"], 1) for v in existing["vehicles"]]
    everyone = allocate(previous + list(requests), registry=Registry(),
                        site=plan_.scenario.world.site, gcs_pose=plan_.scenario.world.gcs_pose,
                        run_dir=plan_.run_dir)
    newcomers = everyone[len(previous):]
    for inst in newcomers:
        if inst.name in taken_names:
            raise OrchestratorError(f"{inst.name} already exists in this run")

    spawner = Spawner(world=plan_.world.info.name)
    endpoints: dict[str, dict[str, Any]] = {}
    ready: dict[str, bool] = {}
    supervisor = Supervisor(session=f"uavsim-{plan_.world.info.name}",
                            log_dir=plan_.run_dir / "logs", kind=plan_.scenario.supervisor)
    for inst in newcomers:
        adapter = plan_.adapters[inst.vehicle.backend.id]
        sdf = render_model(inst, world_name=plan_.world.info.name,
                           backend_vars=adapter.sdf_vars(inst))
        request = adapter.spawn_request(inst, sdf)
        if request is not None:
            spawner.spawn(request)
        inst.run_dir.mkdir(parents=True, exist_ok=True)
        for spec in adapter.processes(inst):
            supervisor.run(spec)
        endpoints[inst.name] = adapter.endpoints(inst)
        if wait_ready:
            ready[inst.name] = adapter.wait_ready(
                inst, float(adapter.settings.get("ready_timeout_s", 60.0)))

    merged = manifest_mod.build(plan_.world, everyone,
                                {**{v["name"]: v["endpoints"] for v in existing["vehicles"]},
                                 **endpoints},
                                bridge_config=existing.get("ros2", {}).get("bridge_config"),
                                run_dir=plan_.run_dir)
    merged["ready"] = {**existing.get("ready", {}), **ready}
    manifest_mod.write(merged, manifest_path)
    return {"instances": newcomers, "ready": ready, "manifest_path": manifest_path}


def _bridge_process(config: Path):
    from .adapter import ProcessSpec

    return ProcessSpec(
        name="ros_gz_bridge",
        cmd=["ros2", "run", "ros_gz_bridge", "parameter_bridge",
             "--ros-args", "-p", f"config_file:={config}"],
        delay_s=2.0,
    )


def down(plan_: Plan, *, stop_gazebo: bool = False) -> list[str]:
    """Stop what we started. Gazebo is left alone unless asked."""
    supervisor = Supervisor(session=f"uavsim-{plan_.world.info.name}",
                            log_dir=plan_.run_dir / "logs", kind=plan_.scenario.supervisor)
    names: list[str] = []
    for inst in plan_.instances:
        names.extend(plan_.adapters[inst.vehicle.backend.id].stop_hints(inst))
    if any(i.vehicle.backend.id == "px4" for i in plan_.instances):
        names.append("MicroXRCEAgent")
    report = supervisor.stop(names)
    if stop_gazebo:
        # SIGINT, never SIGKILL: ArduPilotPlugin's lock-step wait only breaks on a signal.
        for name in ("gz", "ruby", "gz-sim-server", "gz-sim-gui"):
            report.extend(Supervisor._kill_group(name))
    return report


def wait_for(condition, timeout_s: float, interval_s: float = 1.0) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if condition():
            return True
        time.sleep(interval_s)
    return False
