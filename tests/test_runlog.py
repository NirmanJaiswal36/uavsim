"""The pose log, written and read back without a simulator.

Needs ROS 2 sourced (rosbag2_py, tf2_msgs); skipped otherwise.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

pytest.importorskip("rosbag2_py")
pytest.importorskip("tf2_msgs.msg")

from uavsim import poselog, review, runlog       # noqa: E402


def _pose(id_, name, x, y, z, yaw_q=(0.0, 1.0)):
    return SimpleNamespace(id=id_, name=name,
                           position=SimpleNamespace(x=x, y=y, z=z),
                           orientation=SimpleNamespace(x=0.0, y=0.0, z=yaw_q[0], w=yaw_q[1]))


def test_only_top_level_models_become_transforms():
    poses = [_pose(7, "drone_a", 5, 3, 10), _pose(8, "base_link", 0, 0, 0),
             _pose(11, "gimbal", 0, 0, -0.6), _pose(20, "drone_b", -1, -2, 3)]
    msg = poselog.to_tf_message(poses, {7: "drone_a", 20: "drone_b"}, 12, 500)
    assert [t.child_frame_id for t in msg.transforms] == ["drone_a", "drone_b"]
    tf = msg.transforms[0]
    assert tf.header.frame_id == "world"
    assert (tf.header.stamp.sec, tf.header.stamp.nanosec) == (12, 500)
    assert (tf.transform.translation.x, tf.transform.translation.z) == (5, 10)


class _FakeScenes:
    """Answers the scene query from a list of scenes, one per call."""

    def __init__(self, scenes):
        self.scenes, self.calls = list(scenes), 0

    def __call__(self):
        models = self.scenes[min(self.calls, len(self.scenes) - 1)]
        self.calls += 1
        return ({mid: name for mid, name, _ in models},
                {i for mid, _, links in models for i in (mid, *links)})


def test_a_vehicle_the_scene_did_not_list_yet_is_picked_up_later(monkeypatch):
    clock = [100.0]
    monkeypatch.setattr(poselog.time, "monotonic", lambda: clock[0])
    # the first scene/info answer predates the newcomer (id 20) whose poses already arrive
    node = _FakeScenes([[(7, "drone_a", [8])], [(7, "drone_a", [8]), (20, "late", [21])]])
    models = poselog.TopLevelModels(node)
    models.resolve([7, 8, 20, 21])
    assert models.names == {7: "drone_a"} and node.calls == 1
    models.resolve([7, 8, 20, 21])
    assert node.calls == 1                       # throttled
    clock[0] += poselog.SCENE_REFRESH_S
    models.resolve([7, 8, 20, 21])
    assert models.names == {7: "drone_a", 20: "late"} and node.calls == 2
    clock[0] += 10
    models.resolve([7, 8, 20, 21])
    assert node.calls == 2                       # everything explained: no more requests


def test_an_id_the_scene_never_explains_stops_costing_requests(monkeypatch):
    clock = [100.0]
    monkeypatch.setattr(poselog.time, "monotonic", lambda: clock[0])
    node = _FakeScenes([[(7, "drone_a", [8])]])
    models = poselog.TopLevelModels(node)
    for _ in range(10):
        models.resolve([7, 8, 99])
        clock[0] += poselog.SCENE_REFRESH_S
    assert node.calls == poselog.SCENE_TRIES


def _fake_log(tmp_path: Path, monkeypatch) -> Path:
    """A log directory whose bag holds one vehicle flying over site_3's marker."""
    from rclpy.serialization import serialize_message

    monkeypatch.setenv("UAVSIM_LOG_DIR", str(tmp_path))
    scenario = REPO_ROOT / "scenarios" / "forest_rescue.yaml"
    truth = {"roi_seed": 5, "rois": [{"entity": "site_3", "type": "tent",
                                      "marker": {"aruco_id": 4, "enu": [40.0, 10.0]}}]}
    log_dir = runlog.create(team="t", scenario_path=scenario, world="rescue", roi_seed=5,
                            ground_truth=truth, rate_hz=10.0)
    writer = poselog._writer(log_dir / runlog.BAG)
    # 0-2 s on the pad at x=-400, then a 1 s hop to over the marker at 12 m, then a hover
    track = [(-400.0, 0.0, 0.2)] * 20 + [(40.0, 12.0, 12.2)] * 30
    for i, (x, y, z) in enumerate(track):
        stamp_ns = 100_000_000 * i + 5_000_000_000
        msg = poselog.to_tf_message([_pose(1, "v0", x, y, z)], {1: "v0"},
                                    stamp_ns // 1_000_000_000, stamp_ns % 1_000_000_000)
        writer.write(poselog.TOPIC, serialize_message(msg), stamp_ns)
    writer.close()
    return log_dir


def test_a_closed_log_reads_back_and_is_summarised(tmp_path, monkeypatch):
    log_dir = _fake_log(tmp_path, monkeypatch)
    packed = runlog.finish(log_dir, None)
    assert packed["samples"] == 50
    info = json.loads((log_dir / runlog.RUN_INFO).read_text())
    assert info["poses"]["closed_cleanly"] is True

    for source in (log_dir, packed["archive"]):
        run = review.load(source)
        assert list(run.tracks) == ["v0"]
        assert run.start_s == pytest.approx(5.0)
        assert run.end_s == pytest.approx(9.9)
        best = review.closest_pass(run, 40.0, 10.0)
        assert best[0] == pytest.approx(2.0) and best[1] == pytest.approx(12.0)
        text = review.summary(run)
        assert "site_3" in text and "v0" in text

    out = review.plot(review.load(log_dir), tmp_path / "map.png")
    assert out.stat().st_size > 10_000


def test_the_newest_log_is_the_default(tmp_path, monkeypatch):
    log_dir = _fake_log(tmp_path, monkeypatch)
    assert review.resolve(None) == log_dir


def test_a_log_that_was_never_closed_says_how_to_recover(tmp_path, monkeypatch):
    log_dir = _fake_log(tmp_path, monkeypatch)
    (log_dir / runlog.BAG / "metadata.yaml").unlink()
    with pytest.raises(FileNotFoundError, match="ros2 bag reindex"):
        review.load(log_dir)


def test_down_recovers_a_bag_whose_recorder_was_killed(tmp_path, monkeypatch):
    import shutil

    if shutil.which("ros2") is None:
        pytest.skip("needs the ros2 CLI")
    log_dir = _fake_log(tmp_path, monkeypatch)
    (log_dir / runlog.BAG / "metadata.yaml").unlink()
    packed = runlog.finish(log_dir, None)
    assert packed["samples"] == 50
    info = json.loads((log_dir / runlog.RUN_INFO).read_text())
    assert info["poses"]["recovered_by"] == "ros2 bag reindex"
    assert len(review.load(log_dir).samples) == 50


def test_team_names_are_filesystem_safe():
    assert runlog.team_name("Team Ørn #1") == "Team-rn-1"
    assert runlog.team_name("  ") == runlog.team_name(None)
    assert runlog.team_name("../../etc") == "etc"
