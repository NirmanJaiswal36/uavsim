"""Reads a run log back: a summary, a top-down map, and a 3D replay in the real world.

    uavsim review [<log>]          # tables in the terminal + <log>/tracks.png
    uavsim replay [<log>]          # the run again, in Gazebo, with trails

``<log>`` is a log directory, its ``.tar.gz``, or omitted for the newest run in
``uavsim_runs/`` in the repo. Everything here reads files only: no simulation needs to be up,
and nothing in a log is changed (the PNG goes next to it, not into it, when the
log is an archive).
"""

from __future__ import annotations

import atexit
import bisect
import json
import math
import os
import shutil
import signal
import subprocess
import tarfile
import tempfile
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

from . import runlog
from . import world as world_mod
from .poselog import STORAGE, TOPIC
from .registry import PKG_ROOT

AIRBORNE_M = 1.0                 # above the vehicle's own lowest point: off its pad

# tab20, reordered so neighbours in a fleet are easy to tell apart
PALETTE = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd", "#8c564b", "#e377c2",
           "#17becf", "#bcbd22", "#7f7f7f", "#aec7e8", "#ffbb78", "#98df8a", "#ff9896",
           "#c5b0d5", "#c49c94", "#f7b6d2", "#9edae5", "#dbdb8d", "#c7c7c7"]


@dataclass
class Track:
    name: str
    t: list[float] = field(default_factory=list)          # sim seconds
    xyz: list[tuple[float, float, float]] = field(default_factory=list)
    yaw: list[float] = field(default_factory=list)

    @property
    def ground_z(self) -> float:
        return min(p[2] for p in self.xyz)

    def airborne(self) -> list[bool]:
        floor = self.ground_z + AIRBORNE_M
        return [p[2] > floor for p in self.xyz]


@dataclass
class RunLog:
    path: Path                                   # the log directory
    archive: Path | None                         # the .tar.gz it came from, if any
    info: dict[str, Any]
    truth: dict[str, Any]
    roi: dict[str, Any]
    tracks: dict[str, Track]
    samples: list[tuple[float, dict[str, tuple[float, float, float, float]]]]

    @property
    def start_s(self) -> float:
        return self.samples[0][0] if self.samples else 0.0

    @property
    def end_s(self) -> float:
        return self.samples[-1][0] if self.samples else 0.0


# -- loading -------------------------------------------------------------------

def resolve(path: str | Path | None) -> Path:
    """A log dir, a .tar.gz, or None for the newest log under the log root."""
    if path:
        p = Path(path).expanduser()
        if not p.exists():
            raise FileNotFoundError(f"no run log at {p}")
        return p
    root = runlog.log_root()
    logs = sorted((d for d in root.glob("*") if d.is_dir() and (d / runlog.RUN_INFO).exists()),
                  key=lambda d: d.stat().st_mtime)
    if not logs:
        raise FileNotFoundError(f"no run logs in {root}; pass the log directory or .tar.gz")
    return logs[-1]


def load(path: str | Path | None) -> RunLog:
    src = resolve(path)
    archive = None
    if src.is_file():
        archive = src
        out = Path(tempfile.mkdtemp(prefix="uavsim_review_"))
        atexit.register(shutil.rmtree, out, ignore_errors=True)
        with tarfile.open(src) as tar:
            tar.extractall(out, filter="data")
        dirs = [d for d in out.iterdir() if d.is_dir()]
        if len(dirs) != 1:
            raise FileNotFoundError(f"{src} is not a uavsim run log archive")
        src = dirs[0]
    info = json.loads((src / runlog.RUN_INFO).read_text())
    truth_path = src / runlog.GROUND_TRUTH
    truth = json.loads(truth_path.read_text()) if truth_path.exists() else {}
    scenario = yaml.safe_load((src / runlog.SCENARIO_COPY).read_text()) \
        if (src / runlog.SCENARIO_COPY).exists() else {}
    roi_id = (scenario.get("world") or {}).get("roi", "roi_forest")
    roi = world_mod.load_roi(roi_id) if roi_id else {}
    samples = read_bag(src / runlog.BAG)
    tracks: dict[str, Track] = {}
    for t, poses in samples:
        for name, (x, y, z, yaw) in poses.items():
            track = tracks.setdefault(name, Track(name))
            track.t.append(t)
            track.xyz.append((x, y, z))
            track.yaw.append(yaw)
    return RunLog(path=src, archive=archive, info=info, truth=truth, roi=roi,
                  tracks=dict(sorted(tracks.items())), samples=samples)


def read_bag(bag: Path) -> list[tuple[float, dict[str, tuple[float, float, float, float]]]]:
    """``[(sim_s, {vehicle: (x, y, z, yaw)}), ...]`` from a pose bag."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from tf2_msgs.msg import TFMessage

    meta = bag / "metadata.yaml"
    if not meta.exists():
        raise FileNotFoundError(
            f"{bag} has no metadata.yaml: the run was not ended with 'uavsim down'. "
            f"Recover it with:\n  ros2 bag reindex {bag}")
    storage = (yaml.safe_load(meta.read_text()) or {}).get(
        "rosbag2_bagfile_information", {}).get("storage_identifier", STORAGE)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=storage),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    out = []
    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        if topic != TOPIC:
            continue
        msg = deserialize_message(data, TFMessage)
        poses = {}
        for tf in msg.transforms:
            p, q = tf.transform.translation, tf.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            poses[tf.child_frame_id] = (p.x, p.y, p.z, yaw)
        out.append((stamp_ns / 1e9, poses))
    return out


# -- the summary ---------------------------------------------------------------

def summary(run: RunLog) -> str:
    info = run.info
    lines = [
        f"run        {run.path.name}" + (f"  (from {run.archive})" if run.archive else ""),
        f"team       {info.get('team')}",
        f"world      {info.get('world')}, layout seed {info.get('roi_seed')}",
        f"wall clock {info.get('started_utc')} -> {info.get('ended_utc') or '(not closed)'}",
        f"sim time   {run.start_s:.1f} s -> {run.end_s:.1f} s "
        f"({run.end_s - run.start_s:.1f} s, {len(run.samples)} samples)",
        f"code       {(info.get('code') or {}).get('commit')} "
        f"{'(modified)' if (info.get('code') or {}).get('dirty') else ''}".rstrip(),
        "",
    ]
    xmin, xmax, ymin, ymax = world_mod.mission_extent(run.roi) if run.roi else (0, 0, 0, 0)

    rows = [("VEHICLE", "AIRBORNE", "OVER FOREST", "FLOWN", "MAX ALT", "FIRST UP", "LAST DOWN")]
    for track in run.tracks.values():
        up = track.airborne()
        air = forest = flown = 0.0
        for i in range(1, len(track.t)):
            dt = track.t[i] - track.t[i - 1]
            if up[i - 1] and up[i]:
                air += dt
                x, y, _ = track.xyz[i]
                if xmin <= x <= xmax and ymin <= y <= ymax:
                    forest += dt
            flown += math.dist(track.xyz[i - 1], track.xyz[i])
        ups = [track.t[i] for i, a in enumerate(up) if a]
        rows.append((track.name, f"{air:.0f} s", f"{forest:.0f} s", f"{flown:.0f} m",
                     f"{max(p[2] for p in track.xyz) - track.ground_z:.1f} m",
                     f"{ups[0]:.0f} s" if ups else "-", f"{ups[-1]:.0f} s" if ups else "-"))
    lines += _table(rows)

    pair = closest_pair(run)
    if pair:
        d, a, b, t = pair
        lines += ["", f"closest two airborne vehicles came: {d:.1f} m ({a} / {b}, t = {t:.0f} s)"]

    if run.truth.get("rois"):
        lines += ["", "closest airborne pass over each site's marker (judges' view):"]
        rows = [("SITE", "TYPE", "ARUCO", "MARKER ENU", "CLOSEST", "AT ALT", "BY", "AT T")]
        for roi in run.truth["rois"]:
            mx, my = roi["marker"]["enu"]
            best = closest_pass(run, mx, my)
            rows.append((roi["entity"], roi["type"], str(roi["marker"]["aruco_id"]),
                         f"{mx:.0f},{my:.0f}",
                         *((f"{best[0]:.1f} m", f"{best[1]:.1f} m", best[2], f"{best[3]:.0f} s")
                           if best else ("-", "-", "-", "-"))))
        lines += _table(rows)
    else:
        lines += ["", "(no ground_truth.json in this log: site table skipped)"]
    return "\n".join(lines)


def closest_pass(run: RunLog, x: float, y: float):
    """(horizontal distance, altitude above pad, vehicle, sim t) of the nearest airborne pass."""
    best = None
    for track in run.tracks.values():
        floor = track.ground_z
        for (px, py, pz), t, up in zip(track.xyz, track.t, track.airborne()):
            if not up:
                continue
            d = math.hypot(px - x, py - y)
            if best is None or d < best[0]:
                best = (d, pz - floor, track.name, t)
    return best


def closest_pair(run: RunLog):
    best = None
    for t, poses in run.samples:
        names = [n for n in poses if poses[n][2] > run.tracks[n].ground_z + AIRBORNE_M]
        for i, a in enumerate(names):
            for b in names[i + 1:]:
                d = math.dist(poses[a][:3], poses[b][:3])
                if best is None or d < best[0]:
                    best = (d, a, b, t)
    return best


def _table(rows: list[tuple[str, ...]]) -> list[str]:
    widths = [max(len(r[i]) for r in rows) for i in range(len(rows[0]))]
    return ["  ".join(c.ljust(widths[i]) for i, c in enumerate(r)).rstrip() for r in rows]


# -- the map -------------------------------------------------------------------

def plot(run: RunLog, out: Path) -> Path:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(13, 9), dpi=110)
    if run.roi:
        xmin, xmax, ymin, ymax = world_mod.mission_extent(run.roi)
        ax.add_patch(plt.Rectangle((xmin, ymin), xmax - xmin, ymax - ymin, fill=True,
                                   facecolor="#eef5e9", edgecolor="#4d7c3a", lw=1.2, zorder=0))
        trees = run.roi.get("trees") or []
        if trees:
            ax.scatter([t[0] for t in trees], [t[1] for t in trees], s=3, c="#7aa56a",
                       lw=0, zorder=1, label="trees")
    for roi in run.truth.get("rois", []):
        mx, my = roi["marker"]["enu"]
        ax.scatter([mx], [my], marker="s", s=60, facecolor="white", edgecolor="black",
                   zorder=4)
        ax.annotate(f"{roi['marker']['aruco_id']} {roi['type']}", (mx, my),
                    textcoords="offset points", xytext=(6, 5), fontsize=8, zorder=5)
    for i, track in enumerate(run.tracks.values()):
        color = PALETTE[i % len(PALETTE)]
        xs, ys = [p[0] for p in track.xyz], [p[1] for p in track.xyz]
        ax.plot(xs, ys, color=color, lw=1.4, label=track.name, zorder=3)
        ax.scatter([xs[0]], [ys[0]], color=color, marker="o", s=25, zorder=4)
        ax.scatter([xs[-1]], [ys[-1]], color=color, marker="x", s=40, zorder=4)
    ax.set_aspect("equal")
    ax.set_xlabel("x east (m)")
    ax.set_ylabel("y north (m)")
    ax.set_title(f"{run.info.get('team')}  {run.path.name}   "
                 f"sim {run.end_s - run.start_s:.0f} s   (o start, x end, squares: markers)")
    ax.grid(alpha=0.25)
    ax.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), fontsize=8)
    fig.tight_layout()
    fig.savefig(out)
    plt.close(fig)
    return out


def default_plot_path(run: RunLog) -> Path:
    if run.archive:
        return run.archive.with_name(run.archive.name.removesuffix(".tar.gz") + "_tracks.png")
    return run.path / "tracks.png"


# -- the 3D replay -------------------------------------------------------------

REPLAY_FPS = 20.0
TRAIL_SEGMENT = 10               # samples per trail segment (1 s at 10 Hz)


def replay(run: RunLog, *, speed: float = 5.0, start_s: float = 0.0) -> int:
    """Open the run's world (same layout seed) in Gazebo and fly the log through it.

    The vehicles are GUI markers, not models: nothing is simulated, so the replay
    cannot drift from the log, and it needs no autopilot. A marker trail is left
    behind each vehicle.

    The replay runs on a gz-transport partition of its own. A log's world has the
    same name as the live world it came from, and a second server answering for
    /world/rescue/* -- say a run whose GUI was closed but whose server was left up
    by 'uavsim down --keep-gazebo' -- splits the GUI's scene requests between
    the two: the vehicles draw, the world never does. Every gz process started
    from here, and this process's own gz-transport, inherit the partition.
    """
    from .spawner import _wait_for_service

    os.environ["GZ_PARTITION"] = f"uavsim_replay_{os.getpid()}"
    scenario_path = run.path / runlog.SCENARIO_COPY
    world_dir = Path(tempfile.mkdtemp(prefix="uavsim_replay_"))
    atexit.register(shutil.rmtree, world_dir, ignore_errors=True)
    world = _replay_world(scenario_path, int(run.info["roi_seed"]), world_dir)
    env = {**os.environ,
           "GZ_SIM_RESOURCE_PATH": os.pathsep.join(
               [str(PKG_ROOT / "models"), str(PKG_ROOT / "worlds"), str(world.parent),
                os.environ.get("GZ_SIM_RESOURCE_PATH", "")])}
    # paused, and nothing in it moves: the markers are the only thing that changes
    gz = subprocess.Popen(["gz", "sim", "-v1", str(world)], env=env, start_new_session=True,
                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        print("waiting for the Gazebo GUI ...", flush=True)
        if not _wait_for_service("/gui/move_to", 180.0):
            print("the Gazebo GUI did not come up", flush=True)
            return 1
        time.sleep(3.0)
        _aim_camera()
        _play(run, speed=speed, start_s=start_s, alive=lambda: gz.poll() is None)
        if gz.poll() is None:
            print("replay finished; close the Gazebo window (or Ctrl-C) to exit", flush=True)
        gz.wait()
    except KeyboardInterrupt:
        pass
    finally:
        if gz.poll() is None:
            os.killpg(gz.pid, signal.SIGINT)
            try:
                gz.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(gz.pid, signal.SIGKILL)
    return 0


def _replay_world(scenario_path: Path, seed: int, out_dir: Path) -> Path:
    from dataclasses import replace

    from . import scenario as scenario_mod

    scn = scenario_mod.load(scenario_path)
    spec = replace(scn.world, targets=replace(scn.world.targets, roi_seed=seed))
    return world_mod.build(spec, out_dir).info.sdf_path


def _aim_camera(eye=(-440.0, -240.0, 170.0), target=(-60.0, 10.0, 0.0)) -> None:
    """Behind the apron, above the south-west: the pads and most of the forest in frame.

    Through the gz CLI, not this process's gz-transport: after a GUICamera request
    here, protobuf no longer builds the Vector3d points the trail markers need.
    """
    dx, dy, dz = (target[i] - eye[i] for i in range(3))
    yaw, pitch = math.atan2(dy, dx), math.atan2(-dz, math.hypot(dx, dy))
    cy, sy, cp, sp = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(pitch / 2), math.sin(pitch / 2)
    req = (f"pose: {{position: {{x: {eye[0]}, y: {eye[1]}, z: {eye[2]}}}, "
           f"orientation: {{w: {cy * cp}, x: {-sy * sp}, y: {cy * sp}, z: {sy * cp}}}}}")
    subprocess.run(["gz", "service", "-s", "/gui/move_to/pose", "--reqtype", "gz.msgs.GUICamera",
                    "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", req],
                   capture_output=True, timeout=20, check=False)


def _play(run: RunLog, *, speed: float, start_s: float, alive=lambda: True) -> None:
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.marker_pb2 import Marker
    from gz.msgs10.marker_v_pb2 import Marker_V
    from gz.msgs10.vector3d_pb2 import Vector3d          # noqa: F401  Marker.point's type
    from gz.transport13 import Node

    node = Node()
    colors = {name: PALETTE[i % len(PALETTE)] for i, name in enumerate(run.tracks)}
    # from 1: marker id 0 reads as "unset", and the GUI then makes a new marker every
    # time instead of moving the old one
    ids = {name: 1 + i for i, name in enumerate(run.tracks)}
    trail: dict[str, list[tuple[float, float, float]]] = {n: [] for n in run.tracks}
    segment = [1]

    def material(m, hex_color: str, alpha: float = 1.0) -> None:
        r, g, b = (int(hex_color[i:i + 2], 16) / 255 for i in (1, 3, 5))
        for c in (m.material.ambient, m.material.diffuse, m.material.emissive):
            c.r, c.g, c.b, c.a = r, g, b, alpha

    def send(markers: Marker_V) -> None:
        if markers.marker:
            node.request("/marker_array", markers, Marker_V, Boolean, 2000)

    def flush_trail(name: str, out: Marker_V, keep_last: bool = True) -> None:
        points = trail[name]
        if len(points) < 2:
            return
        m = out.marker.add()
        m.ns, m.id, m.action, m.type = "trail", segment[0], Marker.ADD_MODIFY, Marker.LINE_STRIP
        segment[0] += 1
        material(m, colors[name])
        for x, y, z in points:
            p = m.point.add()
            p.x, p.y, p.z = x, y, z
        trail[name] = points[-1:] if keep_last else []

    def vehicle(name: str, x: float, y: float, z: float, yaw: float, out: Marker_V) -> None:
        body = out.marker.add()
        body.ns, body.id, body.action, body.type = "vehicle", ids[name], Marker.ADD_MODIFY, Marker.BOX
        body.pose.position.x, body.pose.position.y, body.pose.position.z = x, y, z
        body.pose.orientation.z, body.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        # several times a real quadcopter, so it can be found from the overview
        body.scale.x, body.scale.y, body.scale.z = 3.0, 3.0, 0.8
        material(body, colors[name])
        label = out.marker.add()
        label.ns, label.id, label.action, label.type = "label", ids[name], Marker.ADD_MODIFY, Marker.TEXT
        label.text = name
        label.pose.position.x, label.pose.position.y, label.pose.position.z = x, y, z + 4.0
        label.scale.x = label.scale.y = label.scale.z = 3.0
        material(label, colors[name])

    times = [t for t, _ in run.samples]
    t0 = run.start_s + start_s
    i = 0
    # everything before --start: trails only, drawn at once
    while i < len(run.samples) and times[i] < t0:
        for name, (x, y, z, _) in run.samples[i][1].items():
            trail[name].append((x, y, z))
        i += 1
    out = Marker_V()
    for name in run.tracks:
        flush_trail(name, out)
    send(out)

    print(f"replaying {run.end_s - run.start_s:.0f} s of sim time at {speed:g}x "
          f"({len(run.tracks)} vehicles)", flush=True)
    wall0, last_report = time.monotonic(), -1
    while i < len(run.samples) and alive():
        now_s = t0 + (time.monotonic() - wall0) * speed
        j = bisect.bisect_right(times, now_s, lo=i)
        out = Marker_V()
        latest: dict[str, tuple[float, float, float, float]] = {}
        for k in range(i, j):
            for name, pose in run.samples[k][1].items():
                trail[name].append(pose[:3])
                latest[name] = pose
                if len(trail[name]) > TRAIL_SEGMENT:
                    flush_trail(name, out)
        for name, (x, y, z, yaw) in latest.items():
            vehicle(name, x, y, z, yaw, out)
        send(out)
        i = max(j, i)
        elapsed = int((now_s - run.start_s) // 30)
        if elapsed != last_report:
            last_report = elapsed
            print(f"  t = {min(now_s, run.end_s) - run.start_s:6.0f} s", flush=True)
        time.sleep(1.0 / REPLAY_FPS)
    out = Marker_V()
    for name in run.tracks:
        flush_trail(name, out, keep_last=False)
    send(out)
