"""PX4 SITL backend.

PX4 can spawn its own Gazebo model, but then the world would have two owners of
entity creation.  Instead we always spawn the rendered model ourselves and hand
PX4 the name through ``PX4_GZ_MODEL_NAME`` while ``PX4_GZ_STANDALONE=1`` keeps
it from starting a second server.  PX4's gz_bridge derives every topic it uses
from that model name (GZBridge.cpp), so a renamed instance is fully isolated.

Ports follow PX4's own scheme (ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink):
instance i gets GCS UDP 18570+i and offboard 14540+i.  MAV_SYS_ID would default
to i+1, which would clash with the ArduPilot vehicles in a mixed fleet, so the
globally allocated sysid is forced through PX4_PARAM_MAV_SYS_ID.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import time
from pathlib import Path
from typing import Any

from uavsim.adapter import AutopilotAdapter, Instance, ProcessSpec
from uavsim.spec import ResolvedVehicle


class PX4Adapter(AutopilotAdapter):
    id = "px4"

    # -- paths ---------------------------------------------------------------

    @property
    def px4_root(self) -> Path:
        return Path(str(self.settings["px4_root"])).expanduser()

    @property
    def px4_binary(self) -> Path:
        return self.px4_root / str(self.settings["px4_binary"])

    @property
    def model_path(self) -> Path:
        return self.px4_root / str(self.settings["model_path"])

    def resource_paths(self) -> list[str]:
        """PX4's gz models, added to GZ_SIM_RESOURCE_PATH instead of being copied."""
        return [str(self.model_path)]

    # -- ports ---------------------------------------------------------------

    def gcs_port(self, inst: Instance) -> int:
        return int(self.settings["gcs_port_base"]) + inst.backend_index

    def offboard_port(self, inst: Instance) -> int:
        return int(self.settings["offboard_port_base"]) + inst.backend_index

    # -- checks --------------------------------------------------------------

    def preflight(self) -> list[str]:
        problems = []
        if not self.px4_binary.exists():
            problems.append(
                f"PX4 binary not found at {self.px4_binary} -- build it with "
                f"'make px4_sitl_default' in {self.px4_root}")
        if not self.model_path.is_dir():
            problems.append(f"PX4 gz models not found at {self.model_path}")
        if self.settings.get("start_dds_agent", True) and \
                shutil.which(str(self.settings["dds_agent"])) is None:
            problems.append(f"{self.settings['dds_agent']} is not on PATH (uXRCE-DDS agent)")
        return problems

    def validate(self, vehicle: ResolvedVehicle) -> list[str]:
        problems = []
        vars_ = vehicle.binding_vars
        model = vars_.get("px4_model")
        if not model:
            problems.append(f"{vehicle.id}: px4 binding is missing vars.px4_model")
        elif not (self.model_path / str(model)).is_dir():
            problems.append(f"{vehicle.id}: PX4 model {model!r} not found under {self.model_path}")
        if not vars_.get("sys_autostart"):
            problems.append(f"{vehicle.id}: px4 binding is missing vars.sys_autostart "
                            f"(airframe id from ROMFS/px4fmu_common/init.d-posix/airframes)")
        return problems

    # -- model ---------------------------------------------------------------

    def sdf_vars(self, inst: Instance) -> dict[str, Any]:
        return {}

    # -- processes -----------------------------------------------------------

    def shared_processes(self, instances: list[Instance]) -> list[ProcessSpec]:
        if not instances or not self.settings.get("start_dds_agent", True):
            return []
        return [ProcessSpec(
            name="uxrce_dds_agent",
            cmd=[str(self.settings["dds_agent"]), "udp4", "-p", str(self.settings["dds_port"])],
        )]

    def processes(self, inst: Instance) -> list[ProcessSpec]:
        vars_ = inst.vehicle.binding_vars
        env = {
            "PX4_SYS_AUTOSTART": str(vars_["sys_autostart"]),
            "PX4_SIM_MODEL": f"gz_{vars_['px4_model']}",
            "PX4_GZ_STANDALONE": "1",
            "PX4_GZ_WORLD": self.world.name,
            "PX4_GZ_MODEL_NAME": inst.name,           # attach, do not spawn
            "PX4_UXRCE_DDS_NS": inst.name,
            "PX4_UXRCE_DDS_PORT": str(self.settings["dds_port"]),
            "PX4_PARAM_MAV_SYS_ID": str(inst.sysid),  # applied after rcS's instance default
            "PX4_PARAM_UXRCE_DDS_KEY": str(inst.sysid),
            "GZ_SIM_RESOURCE_PATH": os.pathsep.join(
                self.world.resource_paths + self.resource_paths()),
        }
        env.update({k: str(v) for k, v in inst.vehicle.backend_config.get("env", {}).items()})
        for name, value in inst.vehicle.backend_config.get("params", {}).items():
            env[f"PX4_PARAM_{name}"] = str(value)

        cmd = [str(self.px4_binary), "-i", str(inst.backend_index)]
        if not self.settings.get("interactive", True):
            # Without a terminal the px4 shell redraws its "pxh>" prompt forever:
            # 180 MB of escape sequences per minute, per vehicle. -d skips it.
            cmd.append("-d")

        return [ProcessSpec(
            name=inst.name,
            cmd=cmd,
            env=env,
            cwd=self.px4_root,
            delay_s=float(self.settings.get("start_stagger_s", 2.0)),
            interactive=True,
        )]

    def wait_ready(self, inst: Instance, timeout_s: float) -> bool:
        """PX4 is up once its DDS client publishes the vehicle status topic."""
        topic = f"/{inst.name}/fmu/out/vehicle_status"
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            try:
                out = subprocess.run(["ros2", "topic", "list"], capture_output=True,
                                     text=True, timeout=15)
            except (OSError, subprocess.TimeoutExpired):
                return True            # no ROS 2 here; nothing to wait for
            if topic in out.stdout or f"{topic}_v1" in out.stdout:
                return True
            time.sleep(2.0)
        return False

    def stop_hints(self, inst: Instance) -> list[str]:
        return ["px4"]

    # -- reporting -----------------------------------------------------------

    def endpoints(self, inst: Instance) -> dict[str, Any]:
        return {
            "protocol": "mavlink2+dds",
            "sysid": inst.sysid,
            "px4_instance": inst.backend_index,
            "mavlink_udp": f"udpout:127.0.0.1:{self.gcs_port(inst)}",
            "mavlink_offboard_udp": f"udpout:127.0.0.1:{self.offboard_port(inst)}",
            "dds_namespace": f"/{inst.name}",
            "dds_agent_port": int(self.settings["dds_port"]),
            "ros2_topics": {
                "status": f"/{inst.name}/fmu/out/vehicle_status",
                "command": f"/{inst.name}/fmu/in/vehicle_command",
                "setpoint": f"/{inst.name}/fmu/in/trajectory_setpoint",
            },
        }
