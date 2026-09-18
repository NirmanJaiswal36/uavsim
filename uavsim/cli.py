#!/usr/bin/env python3
"""uavsim -- spawn any mix of UAVs into the rescue environment.

    uavsim list [vehicles|airframes|backends|payloads]
    uavsim plan   scenarios/forest_rescue.yaml
    uavsim world  scenarios/forest_rescue.yaml      # build the world sdf only
    uavsim up     scenarios/forest_rescue.yaml
    uavsim down   scenarios/forest_rescue.yaml
    uavsim verify scenarios/forest_rescue.yaml [--live]
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

if __package__ in (None, ""):                       # allow ./uavsim/cli.py
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from uavsim import orchestrator, scenario as scenario_mod, verify as verify_mod
from uavsim.allocator import AllocationError
from uavsim.registry import Registry, RegistryError
from uavsim.sdf import RenderError


def _fleet_table(plan) -> str:
    rows = [("NAME", "TYPE", "BACKEND", "SYSID", "LAUNCH", "SPAWN X,Y", "ENDPOINT")]
    for inst in plan.instances:
        adapter = plan.adapters[inst.vehicle.backend.id]
        ep = adapter.endpoints(inst)
        endpoint = ep.get("mavlink") or ep.get("mavlink_udp") or ep.get("dds_namespace") or "-"
        rows.append((inst.name, inst.vehicle.id, inst.vehicle.backend.id, str(inst.sysid),
                     inst.vehicle.airframe.launch,
                     f"{inst.pose.x:.1f},{inst.pose.y:.1f}", str(endpoint)))
    widths = [max(len(r[i]) for r in rows) for i in range(len(rows[0]))]
    return "\n".join("  ".join(cell.ljust(widths[i]) for i, cell in enumerate(row)).rstrip()
                     for row in rows)


def cmd_list(args) -> int:
    reg = Registry()
    what = args.what or "vehicles"
    table = {"vehicles": reg.vehicles, "airframes": reg.airframes,
             "backends": reg.backends, "payloads": reg.payloads}
    if what not in table:
        print(f"unknown listing {what!r}; try {', '.join(table)}", file=sys.stderr)
        return 2
    for key in sorted(table[what]):
        item = table[what][key]
        extra = ""
        if what == "vehicles":
            extra = f"  [{item.airframe} + {item.backend}]"
        elif what == "airframes":
            extra = f"  [{item.cls}, {item.launch}, bindings: {', '.join(sorted(item.bindings))}]"
        desc = (item.description or "").strip().splitlines()
        print(f"{key}{extra}")
        if desc:
            print(f"    {desc[0]}")
    return 0


def cmd_plan(args) -> int:
    scn = scenario_mod.load(args.scenario)
    plan = orchestrator.plan(scn, check_environment=not args.no_check)
    print(f"scenario   {scn.path}")
    print(f"world      {plan.world.info.sdf_path}  (origin "
          f"{plan.world.info.latitude_deg}, {plan.world.info.longitude_deg}, "
          f"{plan.world.info.elevation_m} m)")
    print(f"roi        {plan.world.roi.get('id')}  radius "
          f"{plan.world.roi.get('mission_radius_m')} m, "
          f"{len(plan.world.fires)} fires, {len(plan.world.casualties)} casualties")
    print(f"gcs        {plan.world.site_pose.as_list()[:2]}  "
          f"{abs(plan.world.site_pose.x):.0f} m from the ROI")
    print(f"fleet      {len(plan.instances)} vehicles\n")
    print(_fleet_table(plan))
    if args.sdf:
        for name, sdf in orchestrator.render_all(plan).items():
            print(f"\n----- {name} -----\n{sdf if sdf else '(spawned from a model uri)'}")
    if plan.problems:
        print("\nproblems:")
        for problem in plan.problems:
            print(f"  - {problem}")
        return 1
    print("\nready: run 'uavsim up' to start it")
    return 0


def cmd_world(args) -> int:
    scn = scenario_mod.load(args.scenario)
    plan = orchestrator.plan(scn, check_environment=False)
    print(plan.world.info.sdf_path)
    env = orchestrator.gazebo_env(plan)
    for key, value in env.items():
        print(f"export {key}={value}")
    return 0


def cmd_up(args) -> int:
    scn = scenario_mod.load(args.scenario)
    plan = orchestrator.plan(scn, check_environment=not args.no_check)
    if plan.problems and not args.no_check:
        print("environment is not ready:", file=sys.stderr)
        for problem in plan.problems:
            print(f"  - {problem}", file=sys.stderr)
        return 1
    result = orchestrator.up(plan, wait_ready=not args.no_wait, start_bridge=not args.no_bridge)
    print(_fleet_table(plan))
    print(f"\nmanifest   {result['manifest_path']}")
    if result["bridge_config"]:
        print(f"bridge     {result['bridge_config']}")
    if scn.supervisor == "tmux":
        print(f"consoles   tmux attach -t uavsim-{plan.world.info.name}")
    not_ready = [n for n, ok in result["ready"].items() if not ok]
    if not_ready:
        print(f"\nWARNING: no autopilot heartbeat yet from: {', '.join(not_ready)}")
        return 1
    return 0


def cmd_spawn(args) -> int:
    from uavsim.allocator import FleetRequest

    scn = scenario_mod.load(args.scenario)
    plan = orchestrator.plan(scn, check_environment=False)
    result = orchestrator.add(plan, [FleetRequest(args.type, args.count)],
                              wait_ready=not args.no_wait)
    for inst in result["instances"]:
        adapter = plan.adapters[inst.vehicle.backend.id]
        ep = adapter.endpoints(inst)
        print(f"  {inst.name}  sysid {inst.sysid}  "
              f"{ep.get('mavlink') or ep.get('dds_namespace') or '-'}")
    print(f"manifest   {result['manifest_path']}")
    return 0


def cmd_down(args) -> int:
    scn = scenario_mod.load(args.scenario)
    plan = orchestrator.plan(scn, check_environment=False)
    for line in orchestrator.down(plan, stop_gazebo=args.gazebo):
        print(f"  {line}")
    return 0


def cmd_verify(args) -> int:
    scn = scenario_mod.load(args.scenario)
    plan = orchestrator.plan(scn, check_environment=False)
    return verify_mod.run(plan, live=args.live)


def cmd_manifest(args) -> int:
    scn = scenario_mod.load(args.scenario)
    path = orchestrator._run_dir(scn) / "manifest.json"
    if not path.exists():
        print(f"no manifest at {path} -- is the scenario running?", file=sys.stderr)
        return 1
    print(json.dumps(json.loads(path.read_text()), indent=2))
    return 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="uavsim", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="command", required=True)

    p = sub.add_parser("list", help="show what the registry offers")
    p.add_argument("what", nargs="?", choices=["vehicles", "airframes", "backends", "payloads"])
    p.set_defaults(func=cmd_list)

    p = sub.add_parser("plan", help="validate and show the allocation, start nothing")
    p.add_argument("scenario")
    p.add_argument("--sdf", action="store_true", help="also print each rendered instance model")
    p.add_argument("--no-check", action="store_true", help="skip backend preflight checks")
    p.set_defaults(func=cmd_plan)

    p = sub.add_parser("world", help="build the world sdf and print the env to use it")
    p.add_argument("scenario")
    p.set_defaults(func=cmd_world)

    p = sub.add_parser("up", help="build, spawn and start everything")
    p.add_argument("scenario")
    p.add_argument("--no-wait", action="store_true", help="do not wait for autopilot heartbeats")
    p.add_argument("--no-bridge", action="store_true", help="write the bridge config but do not run it")
    p.add_argument("--no-check", action="store_true", help="skip backend preflight checks")
    p.set_defaults(func=cmd_up)

    p = sub.add_parser("spawn", help="add more vehicles to a running scenario")
    p.add_argument("scenario")
    p.add_argument("type", help="vehicle type id (see 'uavsim list vehicles')")
    p.add_argument("--count", type=int, default=1)
    p.add_argument("--no-wait", action="store_true")
    p.set_defaults(func=cmd_spawn)

    p = sub.add_parser("down", help="stop the autopilots started for a scenario")
    p.add_argument("scenario")
    p.add_argument("--gazebo", action="store_true", help="also stop the Gazebo server")
    p.set_defaults(func=cmd_down)

    p = sub.add_parser("verify", help="check the environment, and optionally the running run")
    p.add_argument("scenario")
    p.add_argument("--live", action="store_true", help="also check the running simulation")
    p.set_defaults(func=cmd_verify)

    p = sub.add_parser("manifest", help="print the manifest of the current run")
    p.add_argument("scenario")
    p.set_defaults(func=cmd_manifest)

    args = ap.parse_args(argv)
    try:
        return args.func(args)
    except (RegistryError, AllocationError, RenderError, FileNotFoundError,
            orchestrator.OrchestratorError) as exc:
        print(f"uavsim: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
