"""ArduPilot SITL backend.

Port layout follows ArduPilot's own rule -- ``-I N`` adds ``10*N`` to every port
(libraries/AP_HAL_SITL/SITL_cmdline.cpp) -- so instance N gets FDM 9002+10N,
SITL TCP 5760+10N and MAVProxy UDP 14550+10N.  Only the FDM port must really be
unique: Gazebo binds it, and the socket is SO_REUSEADDR, so a collision does not
fail, it silently splits the stream.

Home is deliberately the world origin for every vehicle.  ArduPilotPlugin
reports position from components::WorldPose, i.e. NED relative to the Gazebo
world origin, so a per-vehicle ``--home`` would add a second offset on top of
the spawn pose and put the EKF tens of metres off.
"""

from __future__ import annotations

import shutil
import time
from pathlib import Path
from typing import Any

from uavsim.adapter import AutopilotAdapter, Instance, ProcessSpec
from uavsim.spec import ResolvedVehicle


BINARIES = {"ArduCopter": "arducopter", "ArduPlane": "arduplane",
            "Rover": "ardurover", "ArduSub": "ardusub", "Blimp": "blimp"}


class ArduPilotAdapter(AutopilotAdapter):
    id = "ardupilot"

    # -- binaries ------------------------------------------------------------

    def binary_path(self, vehicle: str) -> Path:
        root = Path(str(self.settings.get("ardupilot_root", "~/ardupilot"))).expanduser()
        return root / str(self.settings.get("build_dir", "build/sitl/bin")) / \
            BINARIES.get(vehicle, vehicle.lower())

    def use_mavproxy(self) -> bool:
        """MAVProxy needs a console; without one it reads EOF on stdin, exits, and
        sim_vehicle.py then kills the SITL it just started."""
        mode = str(self.settings.get("mavproxy", "auto"))
        if mode == "always":
            return True
        if mode == "never":
            return False
        return bool(self.settings.get("interactive", True))

    # -- ports ---------------------------------------------------------------

    def _port(self, base_key: str, index: int) -> int:
        return int(self.settings[base_key]) + int(self.settings["port_stride"]) * index

    def fdm_port(self, inst: Instance) -> int:
        return self._port("fdm_base", inst.backend_index)

    def tcp_port(self, inst: Instance) -> int:
        return self._port("tcp_base", inst.backend_index)

    def gcs_port(self, inst: Instance) -> int:
        return self._port("gcs_base", inst.backend_index)

    # -- checks --------------------------------------------------------------

    def preflight(self) -> list[str]:
        problems = []
        if shutil.which("sim_vehicle.py") is None:
            problems.append(
                "sim_vehicle.py is not on PATH -- source your ArduPilot dev environment "
                "(e.g. export PATH=$PATH:$HOME/ardupilot/Tools/autotest)")
        return problems

    def validate(self, vehicle: ResolvedVehicle) -> list[str]:
        problems = []
        vars_ = vehicle.binding_vars
        for key in ("vehicle", "frame"):
            if not vars_.get(key):
                problems.append(f"{vehicle.id}: ardupilot binding is missing vars.{key}")
        # Every instance is started with --no-rebuild, because two sim_vehicle.py
        # instances running 'waf configure' at once corrupt each other's build
        # directory. So the binary has to exist up front.
        if vars_.get("vehicle"):
            binary = self.binary_path(str(vars_["vehicle"]))
            if not binary.exists():
                problems.append(
                    f"{vehicle.id}: {binary} is not built -- build it once with "
                    f"'sim_vehicle.py -v {vars_['vehicle']} -f {vars_.get('frame')} "
                    f"--no-mavproxy --build-target bin/{binary.name}' (or ./waf {binary.name})")
        for param in vehicle.backend_config.get("param_files", []):
            if not (Path(param).expanduser().is_absolute() or (self._pkg_root() / param).exists()):
                problems.append(f"{vehicle.id}: param file not found: {param}")
        return problems

    @staticmethod
    def _pkg_root() -> Path:
        return Path(__file__).resolve().parents[3]

    # -- model ---------------------------------------------------------------

    def sdf_vars(self, inst: Instance) -> dict[str, Any]:
        return {
            "fdm_addr": self.settings["fdm_addr"],
            "fdm_port_in": self.fdm_port(inst),
            "lock_step": bool(self.settings["lock_step"]),
        }

    # -- processes -----------------------------------------------------------

    def processes(self, inst: Instance) -> list[ProcessSpec]:
        cfg = inst.vehicle.backend_config
        vars_ = inst.vehicle.binding_vars

        cmd = [
            "sim_vehicle.py",
            "-v", str(vars_["vehicle"]),
            "-f", str(vars_["frame"]),
            "--model", "JSON",
            "-I", str(inst.backend_index),
            "--sysid", str(inst.sysid),
            "-l", self.world.home_string,
            "--use-dir", str(inst.run_dir),
            "--no-rebuild",           # concurrent waf runs corrupt the shared build dir
        ]
        if not self.use_mavproxy():
            cmd.append("--no-mavproxy")
        for param in cfg.get("param_files", []):
            path = Path(param).expanduser()
            if not path.is_absolute():
                path = self._pkg_root() / path
            cmd.append(f"--add-param-file={path}")
        if cfg.get("wipe_eeprom", self.settings.get("wipe_eeprom", True)):
            cmd.append("-w")
        if self.use_mavproxy():
            # Pin MAVProxy's output explicitly, and suppress sim_vehicle.py's
            # own 14550 + 10*instance output. That default collides with PX4's
            # onboard MAVLink (14580 + instance) from the 4th ArduPilot vehicle
            # onwards, and would leave gcs_base/mavlink_url reporting a port
            # that is not the one actually in use.
            cmd.append("--no-extra-ports")
            cmd += ["--out", f"127.0.0.1:{self.gcs_port(inst)}"]
            for out in self._extra_outs(inst):
                cmd += ["--out", out]
            mavproxy_args = cfg.get("mavproxy_args", self.settings.get("mavproxy_args", ""))
            if mavproxy_args:
                cmd += ["-m", mavproxy_args]

        return [ProcessSpec(
            name=inst.name,
            cmd=cmd,
            # run_in_terminal_window.sh would open its own tmux window if it saw
            # TMUX, and an xterm otherwise; a detached screen keeps the SITL
            # binary out of the way and leaves MAVProxy in our window.
            env={"SITL_RITW_TERMINAL": "screen -D -m"},
            unset_env=["TMUX"],
            cwd=inst.run_dir,
            delay_s=float(self.settings.get("start_stagger_s", 3.0)),
            interactive=True,
        )]

    def _extra_outs(self, inst: Instance) -> list[str]:
        outs = inst.vehicle.backend_config.get("extra_out", self.settings.get("extra_out") or [])
        return [str(o).format(index=inst.backend_index, sysid=inst.sysid, name=inst.name,
                              gcs_port=self.gcs_port(inst)) for o in outs]

    def mavlink_url(self, inst: Instance) -> str:
        """Where this vehicle's MAVLink actually is.

        With MAVProxy it is that instance's own UDP output; without it, SITL's
        own serial0 TCP port.
        """
        if self.use_mavproxy():
            return f"udp:127.0.0.1:{self.gcs_port(inst)}"
        return f"tcp:127.0.0.1:{self.tcp_port(inst)}"

    def wait_ready(self, inst: Instance, timeout_s: float) -> bool:
        """A heartbeat carrying this vehicle's own sysid."""
        try:
            from pymavlink import mavutil
        except ImportError:
            return True
        # SITL's TCP port only exists once it has started, so keep retrying the
        # connection itself until the timeout, not just the heartbeat.
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            try:
                conn = mavutil.mavlink_connection(self.mavlink_url(inst))
            except OSError:
                time.sleep(2.0)
                continue
            try:
                hb = conn.wait_heartbeat(timeout=max(5.0, deadline - time.time()))
                if hb and hb.get_srcSystem() == inst.sysid:
                    return True
            finally:
                conn.close()
        return False

    def stop_hints(self, inst: Instance) -> list[str]:
        vehicle = str(inst.vehicle.binding_vars.get("vehicle", "ArduCopter"))
        names = [BINARIES.get(vehicle, vehicle.lower())]
        if self.use_mavproxy():
            names.append("mavproxy.py")
        return names

    # -- reporting -----------------------------------------------------------

    def endpoints(self, inst: Instance) -> dict[str, Any]:
        return {
            "protocol": "mavlink2",
            "sysid": inst.sysid,
            "sitl_instance": inst.backend_index,
            "fdm_udp": self.fdm_port(inst),
            "sitl_tcp": f"tcp:127.0.0.1:{self.tcp_port(inst)}",
            "mavlink": self.mavlink_url(inst),
            "mavlink_udp": (f"udpin:127.0.0.1:{self.gcs_port(inst)}"
                            if self.use_mavproxy() else None),
            "mavproxy": self.use_mavproxy(),
            "extra_out": self._extra_outs(inst) if self.use_mavproxy() else [],
            "home": self.world.home_string,
        }
