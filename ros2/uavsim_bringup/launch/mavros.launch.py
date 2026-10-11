"""Attach one MAVROS node per ArduPilot vehicle to a simulation that is already up.

    ./bin/uavsim up scenarios/forest_rescue.yaml      # or sim.launch.py
    ros2 launch uavsim_bringup mavros.launch.py world:=rescue

Separate from ``sim.launch.py`` on purpose.  That file is the environment, and it has
to keep working on a machine with no MAVROS installed; MAVROS is a client-side choice.
Composing the two is one ``IncludeLaunchDescription`` if you want both at once.

This reads the run manifest rather than a port table, so it follows the fleet.  It
needs the simulation to be *running*: the manifest is written by ``uavsim up``.

Each vehicle gets its own ``fcu_url`` taken from ``endpoints.extra_out``.  That matters:
``endpoints.mavlink`` (14650 + 10*index) is a single UDP socket, so if your own
MAVLink client holds it MAVROS gets nothing, silently.  Add this to the scenario::

    backends:
      ardupilot:
        extra_out: ["127.0.0.1:1475{index}"]

``scenarios/forest_rescue.yaml`` already does.  Note that ``extra_out`` only exists
when MAVProxy does, i.e. with ``supervisor: tmux``; a headless run falls back to SITL's
own TCP port, which also admits exactly one client.
"""

from __future__ import annotations

import json
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

RUNS_DIR = Path(os.environ.get("UAVSIM_RUN_DIR", "/tmp/uavsim"))


def _fcu_url(endpoints: dict) -> str:
    """Translate a pymavlink endpoint into MAVROS's spelling.

    MAVROS binds the local port and learns the remote from the first packet, so a
    MAVProxy ``--out host:port`` becomes ``udp://:port@``.
    """
    extra = endpoints.get("extra_out") or []
    if extra:
        return f"udp://:{str(extra[0]).rsplit(':', 1)[1]}@"
    url = endpoints.get("mavlink") or ""
    if url.startswith(("udp:", "udpin:", "udpout:")):
        return f"udp://:{url.rsplit(':', 1)[1]}@"
    if url.startswith("tcp:"):
        return "tcp://" + url[len("tcp:"):]
    raise RuntimeError("no usable MAVLink endpoint in the manifest; add "
                       "backends.ardupilot.extra_out to the scenario")


def _launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration("world").perform(context)
    manifest_path = Path(world) if Path(world).is_file() \
        else RUNS_DIR / world / "manifest.json"
    if not manifest_path.is_file():
        raise RuntimeError(
            f"no run manifest at {manifest_path}. Start the simulation first:\n"
            f"  ./bin/uavsim up scenarios/forest_rescue.yaml")

    manifest = json.loads(manifest_path.read_text())
    nodes = []
    for vehicle in manifest.get("vehicles", []):
        if vehicle.get("backend") != "ardupilot":
            continue                       # MAVROS speaks to ArduPilot vehicles only
        index = int(vehicle.get("index", 0))
        nodes.append(Node(
            package="mavros", executable="mavros_node", name="mavros",
            namespace=f"{vehicle['name']}/mavros", output="screen",
            parameters=[{
                "fcu_url": _fcu_url(vehicle.get("endpoints") or {}),
                "gcs_url": "",
                "target_system_id": int(vehicle.get("sysid", 1)),
                "target_component_id": 1,
                # MAVROS's own identity on the link: must be unique per node, and
                # must not collide with a vehicle sysid
                "system_id": 200 + index,
                "component_id": 190,
                "use_sim_time": True,
            }],
        ))

    if not nodes:
        raise RuntimeError(f"no ArduPilot vehicles in {manifest_path}")
    print(f"[mavros.launch] {len(nodes)} mavros node(s) from {manifest_path}")
    return nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "world", default_value="rescue",
            description="world name under the run directory, or a path to manifest.json"),
        OpaqueFunction(function=_launch_setup),
    ])
