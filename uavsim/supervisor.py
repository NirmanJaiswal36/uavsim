"""Runs and stops the processes the adapters describe.

Two backends:

    tmux        one visible window per process -- the MAVProxy/PX4 consoles you
                actually want to type into. Default.
    subprocess  detached children with log files, for CI and headless runs.

Nothing here knows what an autopilot is; it only runs ProcessSpecs.
"""

from __future__ import annotations

import os
import shlex
import shutil
import signal
import subprocess
import time
from dataclasses import dataclass, field
from pathlib import Path

from .adapter import ProcessSpec


class SupervisorError(Exception):
    pass


def _shell_line(spec: ProcessSpec) -> str:
    parts = []
    if spec.cwd:
        parts.append(f"cd {shlex.quote(str(spec.cwd))} &&")
    if spec.unset_env:
        # 'env -u TMUX' also stops sim_vehicle.py's run_in_terminal_window.sh from
        # noticing our tmux session and opening a window of its own.
        parts.append("env " + " ".join(f"-u {shlex.quote(v)}" for v in spec.unset_env))
    for key, value in spec.env.items():
        parts.append(f"{key}={shlex.quote(str(value))}")
    parts.append(" ".join(shlex.quote(c) for c in spec.cmd))
    return " ".join(parts)


@dataclass
class Supervisor:
    session: str = "uavsim"
    log_dir: Path = Path("/tmp/uavsim/logs")
    kind: str = "tmux"
    started: list[str] = field(default_factory=list)

    def __post_init__(self) -> None:
        if self.kind == "tmux" and shutil.which("tmux") is None:
            self.kind = "subprocess"
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self._procs: dict[str, subprocess.Popen] = {}

    # -- lifecycle -----------------------------------------------------------

    def start_session(self) -> None:
        if self.kind != "tmux":
            return
        subprocess.run(["tmux", "kill-session", "-t", self.session],
                       capture_output=True, check=False)
        # A tmux session inherits the tmux *server's* environment, so DISPLAY has
        # to be passed explicitly or the MAVProxy consoles never appear.
        subprocess.run([
            "tmux", "new-session", "-d", "-s", self.session, "-n", "main",
            "-e", f"DISPLAY={os.environ.get('DISPLAY', ':0')}",
            "-e", f"XAUTHORITY={os.environ.get('XAUTHORITY', str(Path.home() / '.Xauthority'))}",
            "exec bash",
        ], check=True)

    def run(self, spec: ProcessSpec) -> None:
        if spec.delay_s:
            time.sleep(spec.delay_s)
        if spec.cwd:
            Path(spec.cwd).mkdir(parents=True, exist_ok=True)
        if self.kind == "tmux":
            self._run_tmux(spec)
        else:
            self._run_subprocess(spec)
        self.started.append(spec.name)

    def _run_tmux(self, spec: ProcessSpec) -> None:
        subprocess.run(["tmux", "new-window", "-d", "-t", self.session,
                        "-n", spec.name, _shell_line(spec)], check=True)
        # keep a failed window open so its error is readable
        subprocess.run(["tmux", "set-option", "-w", "-t", f"{self.session}:{spec.name}",
                        "remain-on-exit", "on"], capture_output=True, check=False)

    def _run_subprocess(self, spec: ProcessSpec) -> None:
        env = dict(os.environ)
        for key in spec.unset_env:
            env.pop(key, None)
        env.update({k: str(v) for k, v in spec.env.items()})
        log = (self.log_dir / f"{spec.name}.log").open("ab")
        self._procs[spec.name] = subprocess.Popen(
            spec.cmd, env=env, cwd=str(spec.cwd) if spec.cwd else None,
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

    # -- teardown ------------------------------------------------------------

    def stop(self, process_names: list[str]) -> list[str]:
        """Stop our children, then insist on the named binaries. Returns a report."""
        report = []
        for name, proc in self._procs.items():
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                report.append(f"{name}: SIGINT")
        for binary in dict.fromkeys(process_names):
            report.extend(self._kill_group(binary))
        if self.kind == "tmux":
            killed = subprocess.run(["tmux", "kill-session", "-t", self.session],
                                    capture_output=True, check=False)
            if killed.returncode == 0:
                report.append(f"tmux session {self.session} killed")
        return report

    @staticmethod
    def _kill_group(name: str) -> list[str]:
        """Politely, then firmly -- matching on the exact process name only.

        ``pkill -f`` would match any command line that merely mentions the
        string, including the shell running this script.
        """
        count = subprocess.run(["pgrep", "-xc", name], capture_output=True, text=True)
        alive = int(count.stdout.strip() or 0) if count.returncode == 0 else 0
        if not alive:
            return []
        subprocess.run(["pkill", "-x", name], capture_output=True, check=False)
        for _ in range(10):
            time.sleep(0.5)
            check = subprocess.run(["pgrep", "-xc", name], capture_output=True, text=True)
            if int(check.stdout.strip() or 0) == 0:
                return [f"{name}: stopped ({alive})"]
        subprocess.run(["pkill", "-9", "-x", name], capture_output=True, check=False)
        time.sleep(1.0)
        left = subprocess.run(["pgrep", "-xc", name], capture_output=True, text=True)
        remaining = int(left.stdout.strip() or 0)
        return [f"{name}: stopped ({alive}, needed SIGKILL)" if not remaining
                else f"WARNING: {remaining} {name} still alive"]

    # -- gazebo --------------------------------------------------------------

    def start_gazebo(self, world_path: Path, *, headless: bool = False, verbosity: int = 3,
                     env: dict[str, str] | None = None) -> None:
        """Start the server. Never paused: the ArduPilot lock-step handshake
        deadlocks if the world starts paused."""
        cmd = ["gz", "sim", f"-v{verbosity}", "-r"]
        if headless:
            cmd += ["-s", "--headless-rendering"]
        cmd.append(str(world_path))
        self.run(ProcessSpec(name="gazebo", cmd=cmd, env=env or {}))
