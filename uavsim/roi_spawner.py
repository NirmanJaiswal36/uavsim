"""Spawns the search-and-rescue sites one by one, each at its own sim time.

    python3 -m uavsim.roi_spawner --world rescue --schedule <log_dir>/roi_schedule.json

``uavsim up`` starts this; you never run it by hand. The schedule is
``world.roi_spawn_schedule()``: a prop and a plate per site that is not in the world
from the start, ordered by ``appear_s``.

Sim time is read by polling ``gz topic -e -n 1`` on the world clock rather than by
subscribing here: while a Python callback is subscribed, gz-transport service requests
from the same process time out (see ``poselog.scene_snapshot``), and the create
service is exactly what this process needs.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

from .adapter import Pose, SpawnRequest
from .spawner import SpawnError, Spawner

POLL_S = 1.0
CLOCK_TIMEOUT_S = 60.0     # no clock for this long: Gazebo is gone, give up


def sim_time(world: str) -> float | None:
    """Current sim time in seconds, or None when the world did not answer."""
    from gz.msgs10.clock_pb2 import Clock
    from google.protobuf import text_format

    try:
        out = subprocess.run(["gz", "topic", "-e", "-n", "1", "-t", f"/world/{world}/clock"],
                             capture_output=True, text=True, timeout=10)
    except (OSError, subprocess.TimeoutExpired):
        return None
    if out.returncode != 0 or not out.stdout.strip():
        return None
    try:
        clock = text_format.Parse(out.stdout, Clock())
    except text_format.ParseError:
        return None
    return clock.sim.sec + clock.sim.nsec / 1e9


def request_for(entry: dict) -> SpawnRequest:
    x, y, z, yaw = entry["pose"]
    return SpawnRequest(name=entry["name"], sdf=entry.get("sdf"), model_uri=entry.get("uri"),
                        pose=Pose(x=x, y=y, z=z, yaw=yaw))


def run(world: str, schedule: list[dict]) -> int:
    pending = sorted(schedule, key=lambda e: e["appear_s"])
    spawner = Spawner(world=world)
    last_tick = time.time()
    while pending:
        now = sim_time(world)
        if now is None:
            if time.time() - last_tick > CLOCK_TIMEOUT_S:
                print(f"roi_spawner: no clock from world {world!r} for "
                      f"{CLOCK_TIMEOUT_S:.0f} s; {len(pending)} entities never spawned",
                      file=sys.stderr)
                return 1
            time.sleep(POLL_S)
            continue
        last_tick = time.time()
        while pending and pending[0]["appear_s"] <= now:
            entry = pending.pop(0)
            try:
                spawner.spawn(request_for(entry))
                print(f"roi_spawner: sim t = {now:.0f} s, spawned {entry['name']}", flush=True)
            except SpawnError as exc:
                # e.g. already there after a re-'up' against a running server
                print(f"roi_spawner: {exc}", file=sys.stderr, flush=True)
        time.sleep(POLL_S)
    print("roi_spawner: every site is out", flush=True)
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--world", required=True)
    parser.add_argument("--schedule", required=True, type=Path)
    args = parser.parse_args(argv)
    return run(args.world, json.loads(args.schedule.read_text()))


if __name__ == "__main__":
    sys.exit(main())
