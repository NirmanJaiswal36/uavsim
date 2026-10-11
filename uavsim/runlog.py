"""The judges' record of one run: a pose bag plus what is needed to read it.

Every ``uavsim up`` opens a log directory and every ``uavsim down`` closes it::

    <repo>/uavsim_runs/<team>_<UTC start>/     (UAVSIM_LOG_DIR overrides <repo>/uavsim_runs)
        bag/                 rosbag2 (sqlite3): /uavsim/poses, every vehicle, 10 Hz sim time
        run_info.json        team, scenario + sha256, code version, fleet, start/end
        scenario.yaml        the scenario file exactly as it was run
        ground_truth.json    where the sites really are -- judges only
        SHA256SUMS           of all of the above, written by 'down'
    <repo>/uavsim_runs/<team>_<UTC start>.tar.gz   the same directory, to hand in

<repo> is this checkout, wherever 'uavsim' is run from; uavsim_runs/ is git-ignored.

Nothing here is ever published on gz or ROS: the answers live in files only.
"""

from __future__ import annotations

import getpass
import hashlib
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import tarfile
import time
from pathlib import Path
from typing import Any

from .adapter import ProcessSpec
from .registry import PKG_ROOT

BAG = "bag"
RUN_INFO = "run_info.json"
GROUND_TRUTH = "ground_truth.json"
SCENARIO_COPY = "scenario.yaml"
SUMS = "SHA256SUMS"
RECORDER_PID = "poselog.pid"
RECORDER = "poselog"

# what the recorder needs to see in a tmux window, which inherits the tmux server's
# environment rather than this shell's
_ROS_ENV = ("PYTHONPATH", "LD_LIBRARY_PATH", "AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH",
            "ROS_DISTRO", "ROS_VERSION", "ROS_PYTHON_VERSION", "ROS_DOMAIN_ID",
            "RMW_IMPLEMENTATION", "GZ_VERSION")


def log_root() -> Path:
    return Path(os.environ.get("UAVSIM_LOG_DIR") or PKG_ROOT / "uavsim_runs").expanduser()


def team_name(value: str | None) -> str:
    """A filesystem-safe team label; the login name when none is given."""
    raw = (value or "").strip() or getpass.getuser()
    name = re.sub(r"[^A-Za-z0-9_-]+", "-", raw).strip("-")
    return name or "team"


def preflight() -> list[str]:
    """Can this interpreter write a rosbag? (ROS 2 sourced, rosbag2_py present.)"""
    try:
        import rosbag2_py                                    # noqa: F401
        from rclpy.serialization import serialize_message    # noqa: F401
        from tf2_msgs.msg import TFMessage                   # noqa: F401
    except ImportError as exc:
        return [f"run logging needs ROS 2 in this shell ({exc.name} is missing): "
                f"source /opt/ros/humble/setup.bash, or set logging.enabled: false "
                f"in the scenario for a run nobody will judge"]
    return []


def create(*, team: str, scenario_path: Path, world: str, roi_seed: int,
           ground_truth: dict[str, Any], rate_hz: float) -> Path:
    """Open a fresh log directory for a run that is about to start."""
    stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    root = log_root()
    log_dir = root / f"{team}_{stamp}"
    n = 1
    while log_dir.exists():
        n += 1
        log_dir = root / f"{team}_{stamp}_{n}"
    log_dir.mkdir(parents=True)

    shutil.copyfile(scenario_path, log_dir / SCENARIO_COPY)
    _write_json(log_dir / GROUND_TRUTH, ground_truth)
    _write_json(log_dir / RUN_INFO, {
        "team": team,
        "world": world,
        "roi_seed": roi_seed,
        "scenario": {"path": str(scenario_path), "sha256": _sha256(scenario_path)},
        "code": _git_version(),
        "versions": _versions(),
        "host": os.uname().nodename,
        "started_utc": _utc_now(),
        "ended_utc": None,
        "poses": {"bag": BAG, "topic": "/uavsim/poses", "type": "tf2_msgs/msg/TFMessage",
                  "rate_hz": rate_hz, "frame": "world (Gazebo ENU, origin = forest centre)",
                  "time": "sim time"},
        "fleet": [],
    })
    return log_dir


def recorder_process(world: str, log_dir: Path, rate_hz: float) -> ProcessSpec:
    return ProcessSpec(
        name=RECORDER,
        cmd=[sys.executable, "-m", "uavsim.poselog", "--world", world,
             "--bag", str(log_dir / BAG), "--rate", f"{rate_hz:g}",
             "--pid-file", str(log_dir / RECORDER_PID)],
        env={k: os.environ[k] for k in _ROS_ENV if k in os.environ},
        cwd=PKG_ROOT,
    )


def wait_recording(log_dir: Path, timeout_s: float = 30.0) -> bool:
    """True once the recorder has subscribed (it writes its pid file then)."""
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if (log_dir / RECORDER_PID).exists():
            return True
        time.sleep(0.25)
    return False


def stop_recorder(log_dir: Path, timeout_s: float = 20.0) -> str:
    """SIGINT the recorder and wait for it to close the bag."""
    pid_file = log_dir / RECORDER_PID
    try:
        pid = int(pid_file.read_text().strip())
    except (FileNotFoundError, ValueError):
        return "pose recorder: not running"
    try:
        os.kill(pid, signal.SIGINT)
    except ProcessLookupError:
        pid_file.unlink(missing_ok=True)
        return "pose recorder: had already exited"
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if not pid_file.exists() and (log_dir / BAG / "metadata.yaml").exists():
            return "pose recorder: bag closed"
        time.sleep(0.25)
    return (f"WARNING: pose recorder did not close the bag in {timeout_s:.0f}s; "
            f"'uavsim down' will try to recover it (ros2 bag reindex {log_dir / BAG})")


def finish(log_dir: Path, manifest_path: Path | None) -> dict[str, Any]:
    """Stamp the end, record the fleet, checksum everything and pack it up."""
    info_path = log_dir / RUN_INFO
    info = json.loads(info_path.read_text())
    info["ended_utc"] = _utc_now()
    recovered = _reindex_if_unclosed(log_dir / BAG)
    if manifest_path and manifest_path.exists():
        manifest = json.loads(manifest_path.read_text())
        info["fleet"] = [{"name": v["name"], "type": v["type"], "backend": v["backend"],
                          "sysid": v["sysid"], "spawn_enu": v["spawn"]["enu"][:3]}
                         for v in manifest.get("vehicles", [])]
    info["poses"].update(_bag_summary(log_dir / BAG))
    if recovered:
        info["poses"]["closed_cleanly"] = False       # the samples are there; say how
        info["poses"]["recovered_by"] = "ros2 bag reindex"
    _write_json(info_path, info)

    files = sorted(p for p in log_dir.rglob("*") if p.is_file() and p.name != SUMS)
    (log_dir / SUMS).write_text("".join(
        f"{_sha256(p)}  {p.relative_to(log_dir)}\n" for p in files))

    archive = log_dir.with_name(log_dir.name + ".tar.gz")
    with tarfile.open(archive, "w:gz") as tar:
        tar.add(log_dir, arcname=log_dir.name)
    return {"log_dir": log_dir, "archive": archive, "sha256": _sha256(archive),
            "samples": info["poses"].get("samples", 0)}


# -- helpers -------------------------------------------------------------------

def _reindex_if_unclosed(bag: Path) -> bool:
    """A recorder that was killed leaves its samples but no metadata.yaml; rebuild it.

    sqlite3 commits every write, so nothing up to the kill is lost.
    """
    if (bag / "metadata.yaml").exists() or not any(bag.glob("*.db3")):
        return False
    try:
        subprocess.run(["ros2", "bag", "reindex", str(bag)], capture_output=True, timeout=120,
                       check=False)
    except (OSError, subprocess.TimeoutExpired):
        return False
    return (bag / "metadata.yaml").exists()


def _bag_summary(bag: Path) -> dict[str, Any]:
    meta = bag / "metadata.yaml"
    if not meta.exists():
        return {"closed_cleanly": False}
    import yaml

    data = (yaml.safe_load(meta.read_text()) or {}).get("rosbag2_bagfile_information", {})
    return {"closed_cleanly": True,
            "samples": int(data.get("message_count", 0)),
            "duration_s": round(int((data.get("duration") or {}).get("nanoseconds", 0)) / 1e9, 1)}


def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def _git_version() -> dict[str, Any]:
    def git(*args: str) -> str | None:
        try:
            out = subprocess.run(["git", "-C", str(PKG_ROOT), *args], capture_output=True,
                                 text=True, timeout=5)
        except (OSError, subprocess.TimeoutExpired):
            return None
        return out.stdout.strip() if out.returncode == 0 else None

    status = git("status", "--porcelain", "--untracked-files=no")
    return {"commit": git("rev-parse", "HEAD"), "dirty": bool(status) if status is not None else None}


def _versions() -> dict[str, Any]:
    try:
        gz = subprocess.run(["gz", "sim", "--versions"], capture_output=True, text=True,
                            timeout=10).stdout.split()
    except (OSError, subprocess.TimeoutExpired):
        gz = []
    return {"gz_sim": gz[0] if gz else None, "ros_distro": os.environ.get("ROS_DISTRO"),
            "python": sys.version.split()[0]}


def _utc_now() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def _write_json(path: Path, data: dict[str, Any]) -> None:
    path.write_text(json.dumps(data, indent=2) + "\n")
