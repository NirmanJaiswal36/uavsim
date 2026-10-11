"""Records where every vehicle went: one pose per vehicle, 10 times a second, to a rosbag.

    python3 -m uavsim.poselog --world rescue --bag <log_dir>/bag [--rate 10]

``uavsim up`` starts this and ``uavsim down`` stops it with SIGINT; you never run
it by hand. It is the judges' record of a run, so it is deliberately small:

* **Only top-level dynamic models.** Gazebo's ``/world/<w>/dynamic_pose/info``
  carries every non-static entity -- each rotor, gimbal link and nested model, as
  a pose relative to its parent -- at 60 Hz. A top-level model's pose is its pose
  in the world, and that is the one worth keeping: which vehicle was where. Which
  ids are top-level comes from ``scene/info``, re-read whenever an id shows up
  that has not been seen before (a vehicle added with ``uavsim spawn``).
  Statics -- trees, the sites, the GCS -- are never in that stream, so whatever
  moves is logged: a team's own drone too, without being registered anywhere.
* **Written straight to the bag**, not published and re-recorded: nothing on the
  ROS graph (or gz) carries the ground-truth poses for a team to read, and there
  is no bridge or ``ros2 bag record`` process to keep alive.
* **Stamped with sim time**, both the bag timestamps and ``header.stamp``, so
  "time taken" means simulated seconds whatever the real-time factor was.

One topic, ``/uavsim/poses`` (``tf2_msgs/msg/TFMessage``): per sample, one
transform per vehicle, ``frame_id`` ``world`` (Gazebo ENU, origin at the forest
centre), ``child_frame_id`` the vehicle's model name.
"""

from __future__ import annotations

import argparse
import json
import os
import queue
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

TOPIC = "/uavsim/poses"
TOPIC_TYPE = "tf2_msgs/msg/TFMessage"
FRAME = "world"
STORAGE = "sqlite3"                 # ships with every ROS 2 Humble install
DEFAULT_RATE_HZ = 10.0
SCENE_REFRESH_S = 0.5
SCENE_TRIES = 3            # then an id the scene never explains is let go


class TopLevelModels:
    """Which entity ids in a pose stream are top-level models, from ``scene/info``.

    ``query()`` returns ``(top-level id -> name, every id the scene explains)``,
    or None when the world did not answer.
    """

    def __init__(self, query) -> None:
        self._query = query
        self.names: dict[int, str] = {}
        self._known: set[int] = set()        # every id the scene has explained
        self._tries: dict[int, int] = {}
        self._refreshed = -SCENE_REFRESH_S

    def resolve(self, ids) -> None:
        # an id the last scene did not mention is a newcomer (or the scene was read
        # a moment before it existed): look again, but at most every SCENE_REFRESH_S,
        # since the forest world's scene is megabytes
        unknown = [i for i in ids if i not in self._known]
        if not unknown or time.monotonic() - self._refreshed < SCENE_REFRESH_S:
            return
        self._refreshed = time.monotonic()
        answer = self._query()
        if answer is None:
            return
        self.names, known = answer
        self._known |= known
        for i in unknown:
            if i not in self._known:
                self._tries[i] = self._tries.get(i, 0) + 1
                if self._tries[i] >= SCENE_TRIES:
                    self._known.add(i)


def scene_snapshot(world: str, timeout_ms: int = 5000):
    """``(top-level id -> name, all model and link ids)`` from ``scene/info``.

    Never call this in a process that subscribes to a pose stream: while a Python
    callback is subscribed at 60 Hz, gz-transport service requests from that
    process time out (every one, from any Node). The recorder therefore asks a
    child process, see ``_scene_from_child``.
    """
    from gz.msgs10.empty_pb2 import Empty
    from gz.msgs10.scene_pb2 import Scene
    from gz.transport13 import Node

    ok, scene = Node().request(f"/world/{world}/scene/info", Empty(), Empty, Scene, timeout_ms)
    if not ok:
        return None
    known: set[int] = set()

    def walk(model) -> None:
        known.add(model.id)
        known.update(link.id for link in model.link)
        for nested in model.model:
            walk(nested)
    for model in scene.model:
        walk(model)
    return {m.id: m.name for m in scene.model}, known


def _scene_from_child(world: str):
    try:
        out = subprocess.run([sys.executable, "-m", "uavsim.poselog", "--scene", world],
                             capture_output=True, text=True, timeout=20,
                             cwd=Path(__file__).resolve().parent.parent)
    except subprocess.TimeoutExpired:
        return None
    if out.returncode != 0:
        return None
    data = json.loads(out.stdout)
    return {int(k): v for k, v in data["names"].items()}, set(data["known"])


def _writer(bag: Path):
    import rosbag2_py

    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=STORAGE),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    writer.create_topic(rosbag2_py.TopicMetadata(name=TOPIC, type=TOPIC_TYPE,
                                                 serialization_format="cdr"))
    return writer


def to_tf_message(poses, names: dict[int, str], stamp_sec: int, stamp_nsec: int):
    """The top-level entries of a gz ``Pose_V`` as a ``TFMessage``."""
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage

    msg = TFMessage()
    for pose in poses:
        name = names.get(pose.id)
        if name is None:
            continue
        tf = TransformStamped()
        tf.header.stamp.sec, tf.header.stamp.nanosec = stamp_sec, stamp_nsec
        tf.header.frame_id = FRAME
        tf.child_frame_id = name
        t, q = tf.transform.translation, tf.transform.rotation
        t.x, t.y, t.z = (float(v) for v in (pose.position.x, pose.position.y,
                                             pose.position.z))
        q.x, q.y, q.z, q.w = (float(v) for v in (pose.orientation.x, pose.orientation.y,
                                                 pose.orientation.z, pose.orientation.w))
        msg.transforms.append(tf)
    return msg


def record(world: str, bag: Path, rate_hz: float, stop: threading.Event,
           pid_file: Path | None = None) -> int:
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node
    from rclpy.serialization import serialize_message

    period_ns = int(1e9 / rate_hz)
    samples: queue.Queue = queue.Queue(maxsize=1000)
    last_ns = [-period_ns]

    def on_poses(msg: Pose_V) -> None:
        # cheap on purpose: this runs on gz-transport's thread at 60 Hz
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nsec
        if stamp_ns < last_ns[0]:
            last_ns[0] = -period_ns           # the world was reset; start over
        if stamp_ns - last_ns[0] < period_ns:
            return
        last_ns[0] = stamp_ns
        try:
            samples.put_nowait((stamp_ns, msg))
        except queue.Full:
            pass

    node = Node()
    models = TopLevelModels(lambda: _scene_from_child(world))
    writer = _writer(bag)
    if not node.subscribe(Pose_V, f"/world/{world}/dynamic_pose/info", on_poses):
        print(f"poselog: could not subscribe to world {world!r}", file=sys.stderr)
        return 1
    print(f"poselog: recording {TOPIC} from world {world!r} at {rate_hz:g} Hz to {bag}",
          flush=True)
    if pid_file:
        # only now: uavsim up waits for this file before it spawns anything, so the
        # vehicles' first seconds on their pads are in the bag
        pid_file.write_text(f"{os.getpid()}\n")

    written = 0
    tracked: set[str] = set()
    while not stop.is_set():
        try:
            stamp_ns, msg = samples.get(timeout=0.2)
        except queue.Empty:
            continue
        models.resolve([p.id for p in msg.pose])
        tf = to_tf_message(msg.pose, models.names, msg.header.stamp.sec, msg.header.stamp.nsec)
        names = {t.child_frame_id for t in tf.transforms}
        if names != tracked:
            tracked = names
            print(f"poselog: recording {len(names)}: {', '.join(sorted(names)) or '-'}",
                  flush=True)
        if tf.transforms:
            writer.write(TOPIC, serialize_message(tf), stamp_ns)
            written += 1
            if written % int(rate_hz * 60) == 0:
                print(f"poselog: {written} samples, sim t = {stamp_ns / 1e9:.0f} s, "
                      f"{len(tf.transforms)} vehicles", flush=True)
    # closing is what writes metadata.yaml; a bag without one needs 'ros2 bag reindex'
    writer.close()
    print(f"poselog: closed {bag} after {written} samples", flush=True)
    return 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--world")
    ap.add_argument("--bag", type=Path, help="bag directory to create")
    ap.add_argument("--rate", type=float, default=DEFAULT_RATE_HZ, help="samples per sim second")
    ap.add_argument("--pid-file", type=Path,
                    help="written once recording, removed once the bag is closed")
    ap.add_argument("--scene", metavar="WORLD", help=argparse.SUPPRESS)
    args = ap.parse_args(argv)
    if args.scene:                       # the recorder's child: see scene_snapshot()
        answer = scene_snapshot(args.scene)
        if answer is None:
            return 1
        print(json.dumps({"names": answer[0], "known": sorted(answer[1])}))
        return 0
    if not args.world or not args.bag:
        ap.error("--world and --bag are required")

    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: stop.set())
    try:
        return record(args.world, args.bag, args.rate, stop, args.pid_file)
    finally:
        if args.pid_file:
            args.pid_file.unlink(missing_ok=True)


if __name__ == "__main__":
    raise SystemExit(main())
