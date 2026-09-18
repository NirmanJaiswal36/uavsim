"""Bring the rescue environment up from ROS 2.

    ros2 launch uavsim_bringup sim.launch.py scenario:=forest_rescue

This is a thin wrapper: uavsim still does the world building, spawning and
autopilot startup, and this file only adds the ROS 2 side (the generated bridge
and use_sim_time). No mission logic lives here -- arming, take-off and search
patterns belong to whatever framework you are testing.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _find_uavsim_root() -> Path:
    """Locate the uavsim checkout that holds the registry and scenarios.

    This launch file is installed to share/uavsim_bringup/launch/, so it cannot
    assume a fixed number of parent directories -- the answer differs between
    running from source and running from a colcon install space.
    """
    env = os.environ.get("UAVSIM_ROOT")
    if env:
        root = Path(env).expanduser().resolve()
        if not (root / "uavsim" / "cli.py").is_file():
            raise RuntimeError(f"UAVSIM_ROOT={root} does not look like a uavsim checkout")
        return root
    for candidate in Path(__file__).resolve().parents:
        if (candidate / "uavsim" / "cli.py").is_file() and (candidate / "registry").is_dir():
            return candidate
    raise RuntimeError(
        "cannot locate the uavsim checkout from "
        f"{Path(__file__).resolve()}; set UAVSIM_ROOT to the repository root")


PKG_ROOT = _find_uavsim_root()
sys.path.insert(0, str(PKG_ROOT))


def _launch_setup(context, *args, **kwargs):
    from uavsim import orchestrator, scenario as scenario_mod

    name = LaunchConfiguration("scenario").perform(context)
    wait = LaunchConfiguration("wait_ready").perform(context).lower() in ("1", "true", "yes")

    scn = scenario_mod.load(name)
    plan = orchestrator.plan(scn)
    if plan.problems:
        raise RuntimeError("environment is not ready:\n  - " + "\n  - ".join(plan.problems))

    # uavsim runs Gazebo and the autopilots under its own supervisor; the bridge
    # is started here so it is a proper ROS 2 node in this launch tree.
    result = orchestrator.up(plan, wait_ready=wait, start_bridge=False)
    bridge_config = result["bridge_config"]

    actions = []
    if bridge_config:
        actions.append(Node(
            package="ros_gz_bridge", executable="parameter_bridge", name="uavsim_bridge",
            parameters=[{"config_file": str(bridge_config), "use_sim_time": True}],
            output="screen",
        ))
    actions.append(ExecuteProcess(
        cmd=["echo", f"uavsim manifest: {result['manifest_path']}"], output="screen"))
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("scenario", default_value="forest_rescue",
                              description="scenario name or path (see scenarios/)"),
        DeclareLaunchArgument("wait_ready", default_value="true",
                              description="wait for each autopilot to report in"),
        OpaqueFunction(function=_launch_setup),
    ])
