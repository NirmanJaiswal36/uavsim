"""Ties the pieces together: world -> allocation -> spawn -> autopilots -> ROS 2.

The orchestrator is deliberately vehicle-agnostic.  It never asks "is this
ArduPilot?"; it asks the adapter for SDF variables, processes, readiness and
endpoints.  Adding a backend therefore means adding a directory under
registry/backends/, not editing this file.
"""

from __future__ import annotations

import json
import os
import time
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any

from . import bridge as bridge_mod
from . import manifest as manifest_mod
from . import runlog
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


RUN_STATE = "run_state.json"


def read_run_state(run_dir: Path) -> dict[str, Any]:
    """What ``up`` decided for the run in ``run_dir``; empty when nothing is up."""
    try:
        return json.loads((run_dir / RUN_STATE).read_text())
    except (FileNotFoundError, json.JSONDecodeError):
        return {}


def _resolve_roi_seed(scenario: Scenario, run_dir: Path, roi_seed: int | None,
                      attach: bool) -> int:
    """The layout seed: explicit, else the running run's, else the scenario's, else new.

    A scenario normally leaves the seed out so every run is a fresh layout. Anything
    that talks to a run that is already up (spawn, down, verify) must rebuild *that*
    layout, though -- ``add`` re-plans and rewrites the manifest -- so ``up`` records
    the seed it drew and ``attach`` reads it back.
    """
    if roi_seed is not None:
        return int(roi_seed)
    if attach and "roi_seed" in (state := read_run_state(run_dir)):
        return int(state["roi_seed"])
    if scenario.world.targets.roi_seed is not None:
        return scenario.world.targets.roi_seed
    return world_mod.new_seed()


def plan(scenario: Scenario, registry: Registry | None = None, *,
         check_environment: bool = True, roi_seed: int | None = None,
         attach: bool = False) -> Plan:
    """Resolve, allocate and render everything, touching nothing outside build/.

    ``attach`` plans for the run that is already up, reusing its ROI layout;
    without it a scenario that leaves the seed out gets a fresh one.
    """
    registry = registry or Registry()
    run_dir = _run_dir(scenario)

    # gazebo.start: false means 'up' joins a server that already has its ROIs
    attach = attach or not scenario.gazebo.get("start", True)
    seed = _resolve_roi_seed(scenario, run_dir, roi_seed, attach)
    world_spec = replace(scenario.world,
                         targets=replace(scenario.world.targets, roi_seed=seed))

    resource_paths = [str(PKG_ROOT / "models"), str(PKG_ROOT / "worlds"), str(BUILD_WORLDS)]
    built = world_mod.build(world_spec, BUILD_WORLDS,
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

    if check_environment and scenario.logging.get("enabled", True):
        problems.extend(runlog.preflight())

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


def up(plan_: Plan, *, wait_ready: bool = True, start_bridge: bool = True,
       team: str | None = None) -> dict[str, Any]:
    """Start Gazebo (unless already running), spawn the fleet, start the autopilots.

    Unless the scenario turns logging off, the run is also recorded: a log
    directory with the ground truth is opened first, and the pose recorder is
    running before the first vehicle is spawned. ``down`` closes and packs it.
    """
    if plan_.problems:
        raise OrchestratorError("environment is not ready:\n  - " + "\n  - ".join(plan_.problems))

    scenario = plan_.scenario
    env = gazebo_env(plan_)
    truth = manifest_mod.ground_truth(plan_.world)
    rate_hz = float(scenario.logging.get("rate_hz", 10.0))
    log_dir = None
    if scenario.logging.get("enabled", True):
        log_dir = runlog.create(team=runlog.team_name(team or scenario.logging.get("team")),
                                scenario_path=scenario.path, world=plan_.world.info.name,
                                roi_seed=plan_.world.targets.roi_seed, ground_truth=truth,
                                rate_hz=rate_hz)
        # the answers go to the judges' log, never next to the participants' manifest
        (plan_.run_dir / runlog.GROUND_TRUTH).unlink(missing_ok=True)
    else:
        _write_json(plan_.run_dir / runlog.GROUND_TRUTH, truth)
    # first, so that a run which dies half-way up can still be brought down cleanly
    _write_json(plan_.run_dir / RUN_STATE, {
        "scenario": str(scenario.path),
        "world": plan_.world.info.name,
        "roi_seed": plan_.world.targets.roi_seed,
        "started": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "log_dir": str(log_dir) if log_dir else None,
    })
    supervisor = Supervisor(session=f"uavsim-{plan_.world.info.name}",
                            log_dir=plan_.run_dir / "logs", kind=scenario.supervisor)
    supervisor.start_session()

    spawner = Spawner(world=plan_.world.info.name)
    warnings: list[str] = []
    started_gui = scenario.gazebo.get("start", True) and not scenario.gazebo.get("headless", False)
    if scenario.gazebo.get("start", True):
        supervisor.start_gazebo(plan_.world.info.sdf_path,
                                headless=bool(scenario.gazebo.get("headless", False)),
                                verbosity=int(scenario.gazebo.get("verbosity", 3)), env=env)
    if not spawner.wait_for_world(timeout_s=120.0):
        raise OrchestratorError(
            f"world {plan_.world.info.name!r} never came up; check the gazebo window/log in "
            f"{supervisor.log_dir}")
    # Spawning before the GUI's scene exists leaves those vehicles undrawn for the
    # whole run -- see Spawner.wait_for_gui.
    if started_gui and not spawner.wait_for_gui(timeout_s=120.0):
        warnings.append("the Gazebo GUI never reported its 3D scene ready; vehicles were "
                        "spawned anyway and may be missing from the GUI (they still fly)")
    if log_dir:
        supervisor.run(runlog.recorder_process(plan_.world.info.name, log_dir, rate_hz))
        if wait_ready and not runlog.wait_recording(log_dir):
            warnings.append(f"the pose recorder has not started; this run may not be logged "
                            f"(see the '{runlog.RECORDER}' window or {supervisor.log_dir})")

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
            "bridge_config": bridge_path, "ready": ready, "warnings": warnings,
            "log_dir": log_dir}


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

    existing = json.loads(manifest_path.read_text())
    taken_names = {v["name"] for v in existing["vehicles"]}
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


def _write_json(path: Path, data: dict[str, Any]) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2) + "\n")
    return path


def _bridge_process(config: Path):
    from .adapter import ProcessSpec

    return ProcessSpec(
        name="ros_gz_bridge",
        cmd=["ros2", "run", "ros_gz_bridge", "parameter_bridge",
             "--ros-args", "-p", f"config_file:={config}"],
        delay_s=2.0,
    )


def down(plan_: Plan, *, stop_gazebo: bool = True) -> list[str]:
    """Stop what we started, Gazebo included, and close the run's log.

    ``stop_gazebo=False`` keeps the server for a quick re-'up'. It is not the default:
    a server whose GUI window was closed keeps running invisibly, holds the world's
    gz topics and CPU, and is easily forgotten.

    The pose recorder goes first, while every vehicle is still where it ended up,
    and is waited for: the bag is only complete once it has written metadata.yaml.
    """
    state = read_run_state(plan_.run_dir)
    log_dir = Path(state["log_dir"]) if state.get("log_dir") else None
    report: list[str] = []
    if log_dir and log_dir.is_dir():
        report.append(runlog.stop_recorder(log_dir))

    supervisor = Supervisor(session=f"uavsim-{plan_.world.info.name}",
                            log_dir=plan_.run_dir / "logs", kind=plan_.scenario.supervisor)
    names: list[str] = []
    for inst in plan_.instances:
        names.extend(plan_.adapters[inst.vehicle.backend.id].stop_hints(inst))
    report.extend(supervisor.stop(names))

    if log_dir and log_dir.is_dir():
        packed = runlog.finish(log_dir, plan_.run_dir / "manifest.json")
        report.append(f"run log: {packed['log_dir']} ({packed['samples']} pose samples)")
        report.append(f"hand in: {packed['archive']}")
        report.append(f"sha256:  {packed['sha256']}")
        # closed: a second 'down' must not repack it
        _write_json(plan_.run_dir / RUN_STATE, {**state, "log_dir": None})
    if stop_gazebo:
        # SIGINT, never SIGKILL: ArduPilotPlugin's lock-step wait only breaks on a signal.
        for name in ("gz", "ruby", "gz-sim-server", "gz-sim-gui"):
            report.extend(supervisor._kill_group(name))
        # the layout dies with the world; the next 'up' draws a fresh one
        (plan_.run_dir / RUN_STATE).unlink(missing_ok=True)
    return report


def wait_for(condition, timeout_s: float, interval_s: float = 1.0) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if condition():
            return True
        time.sleep(interval_s)
    return False
