"""Checks that catch the failures this simulation actually has.

Every static check here is one that has actually bitten a run in this world, and
the live ones keep its hard-won lessons: a second plane collision hangs the
physics step, two servers can both bind the FDM ports without any error, and one
stalled SITL freezes every vehicle because lock-step has no timeout.
"""

from __future__ import annotations

import re
import subprocess
from typing import Any

from .orchestrator import Plan, render_all


class _Report:
    def __init__(self) -> None:
        self.passed = 0
        self.failed = 0

    def ok(self, message: str) -> None:
        self.passed += 1
        print(f"  PASS  {message}")

    def bad(self, message: str) -> None:
        self.failed += 1
        print(f"  FAIL  {message}")

    def check(self, condition: bool, good: str, bad: str) -> None:
        self.ok(good) if condition else self.bad(bad)


def run(plan: Plan, *, live: bool = False) -> int:
    report = _Report()

    print("== world ==")
    world_text = plan.world.info.sdf_path.read_text()
    planes = len(re.findall(r"<plane>", re.sub(r"<!--.*?-->", "", world_text, flags=re.S)))
    report.check(planes == 1, "exactly one plane collision in the world",
                 f"{planes} plane collisions (must be 1 -- coincident infinite planes "
                 f"make the contact LCP singular and freeze the physics step)")
    report.check("<model name=\"drone" not in world_text and
                 all(f'name="{i.name}"' not in world_text for i in plan.instances),
                 "world contains no vehicles (they are spawned at runtime)",
                 "world file still declares vehicles")

    print("== models ==")
    try:
        rendered = render_all(plan)
        report.ok(f"{len(rendered)} instance models render")
    except Exception as exc:                                   # noqa: BLE001
        report.bad(f"rendering failed: {exc}")
        rendered = {}

    ports: dict[int, str] = {}
    for inst in plan.instances:
        adapter = plan.adapters[inst.vehicle.backend.id]
        port = adapter.endpoints(inst).get("fdm_udp")
        if port is None:
            continue
        if port in ports:
            report.bad(f"fdm port {port} used by both {ports[port]} and {inst.name}")
        ports[port] = inst.name
    if ports:
        report.ok(f"{len(ports)} unique FDM ports")

    sysids = [i.sysid for i in plan.instances]
    report.check(len(set(sysids)) == len(sysids), f"{len(sysids)} unique MAVLink sysids",
                 "duplicate sysids across the fleet")

    for name, sdf in rendered.items():
        if sdf and "<plane>" in sdf:
            report.bad(f"{name} adds a plane collision")

    print("== environment ==")
    if plan.problems:
        for problem in plan.problems:
            report.bad(problem)
    else:
        report.ok("backend preflight clean (or skipped)")

    if not live:
        print(f"\nstatic: {report.passed} passed, {report.failed} failed  "
              f"(--live to check a running simulation)")
        return 1 if report.failed else 0

    print("== gazebo ==")
    stats = _gz_stats(plan.world.info.name)
    report.check(stats is not None and stats > 0,
                 f"sim advancing (iterations={stats})",
                 "sim not advancing -- one stalled SITL wedges every vehicle under lock_step")

    models = _gz_models(plan.world.info.name)
    missing = [i.name for i in plan.instances if i.name not in models]
    report.check(not missing, f"all {len(plan.instances)} vehicles present in the world",
                 f"missing from the world: {', '.join(missing)}")

    print("== ports ==")
    listening = _udp_listeners()
    for port, name in sorted(ports.items()):
        owners = listening.get(port, set())
        if not owners:
            report.bad(f"{name}: nothing listening on UDP/{port}")
        elif len(owners) > 1:
            report.bad(f"{name}: UDP/{port} shared by {len(owners)} processes "
                       f"(SO_REUSEADDR splits the FDM stream silently)")
        else:
            report.ok(f"{name}: UDP/{port} owned by one server")

    print("== autopilots ==")
    for inst in plan.instances:
        adapter = plan.adapters[inst.vehicle.backend.id]
        report.check(adapter.wait_ready(inst, 20.0),
                     f"{inst.name}: autopilot responding (sysid {inst.sysid})",
                     f"{inst.name}: no response from the autopilot")

    print(f"\n{report.passed} passed, {report.failed} failed")
    return 1 if report.failed else 0


def _gz_stats(world: str) -> int | None:
    try:
        out = subprocess.run(["gz", "topic", "-e", "-t", f"/world/{world}/stats", "-n", "1"],
                             capture_output=True, text=True, timeout=15)
    except (OSError, subprocess.TimeoutExpired):
        return None
    match = re.search(r"iterations:\s*(\d+)", out.stdout)
    return int(match.group(1)) if match else None


def _gz_models(world: str) -> set[str]:
    try:
        out = subprocess.run(["gz", "model", "--list"], capture_output=True, text=True, timeout=20)
    except (OSError, subprocess.TimeoutExpired):
        return set()
    return {line.strip("- ").strip() for line in out.stdout.splitlines() if line.strip()}


def _udp_listeners() -> dict[int, set[str]]:
    """port -> set of owning pids, from ss."""
    try:
        out = subprocess.run(["ss", "-lunp"], capture_output=True, text=True, timeout=10)
    except (OSError, subprocess.TimeoutExpired):
        return {}
    listeners: dict[int, set[str]] = {}
    for line in out.stdout.splitlines():
        match = re.search(r":(\d+)\s", line)
        if not match:
            continue
        pids = set(re.findall(r"pid=(\d+)", line))
        listeners.setdefault(int(match.group(1)), set()).update(pids or {"?"})
    return listeners


def summary(plan: Plan) -> dict[str, Any]:
    return {"vehicles": len(plan.instances), "problems": plan.problems}
