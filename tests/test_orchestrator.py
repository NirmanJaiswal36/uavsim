"""Ordering guarantees of orchestrator.up(), checked with no simulator running.

Gazebo and the supervisor are replaced by recorders, so what is under test is only the
sequence of calls up() makes.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from uavsim import orchestrator, runlog, scenario as scenario_mod      # noqa: E402


@pytest.fixture(autouse=True)
def _private_log_root(monkeypatch, tmp_path):
    """Never let a test leave run logs in the repo's real uavsim_runs/."""
    monkeypatch.setenv("UAVSIM_LOG_DIR", str(tmp_path / "uavsim_runs"))


class _Recorder:
    def __init__(self, calls: list[str], **_kwargs):
        self.calls = calls
        self.log_dir = Path("/nonexistent")

    # Supervisor
    def start_session(self):
        self.calls.append("start_session")

    def start_gazebo(self, *_args, headless: bool = False, **_kwargs):
        self.calls.append("start_gazebo_headless" if headless else "start_gazebo_gui")

    def run(self, spec):
        self.calls.append(f"run:{spec.name}")

    def stop(self, names):
        self.calls.append("stop")
        return []

    def _kill_group(self, name):
        self.calls.append(f"kill:{name}")
        return []

    # Spawner
    def wait_for_world(self, timeout_s: float = 0.0) -> bool:
        self.calls.append("wait_for_world")
        return True

    def wait_for_gui(self, timeout_s: float = 0.0, settle_s: float = 0.0) -> bool:
        self.calls.append("wait_for_gui")
        return True

    def spawn(self, request):
        self.calls.append(f"spawn:{request.name}")


@pytest.fixture
def recorded_up(monkeypatch, tmp_path):
    """up() for forest_rescue with gazebo and the supervisor recorded, not run."""
    monkeypatch.setattr(orchestrator, "RUNS_DIR", tmp_path)
    calls: list[str] = []
    monkeypatch.setattr(orchestrator, "Supervisor", lambda **kw: _Recorder(calls, **kw))
    monkeypatch.setattr(orchestrator, "Spawner", lambda **kw: _Recorder(calls, **kw))

    def go(**gazebo) -> list[str]:
        scn = scenario_mod.load(REPO_ROOT / "scenarios" / "forest_rescue.yaml")
        scn.gazebo.update(gazebo)
        plan = orchestrator.plan(scn, check_environment=False)
        orchestrator.up(plan, wait_ready=False, start_bridge=False)
        return calls
    return go


def test_vehicles_are_spawned_only_after_the_gui_scene_is_ready(recorded_up):
    """A model created while the GUI is still building its scene is never drawn: it
    flies, its autopilot is fine, and it is invisible for the whole run -- which is
    exactly what "the drones are not spawning" looked like."""
    calls = recorded_up(headless=False, start=True)
    spawns = [i for i, c in enumerate(calls) if c.startswith("spawn:")]
    assert spawns, "forest_rescue spawns its ArduPilot vehicles from up()"
    assert "wait_for_gui" in calls
    assert calls.index("wait_for_gui") < spawns[0]


def test_headless_runs_do_not_wait_for_a_gui(recorded_up):
    assert "wait_for_gui" not in recorded_up(headless=True, start=True)


def test_attaching_to_an_existing_server_does_not_wait_for_a_gui(recorded_up):
    # with start: false the GUI, if any, is the user's and is already up
    assert "wait_for_gui" not in recorded_up(headless=False, start=False)


# -- the ROI layout ------------------------------------------------------------

def _scenario(**gazebo):
    scn = scenario_mod.load(REPO_ROOT / "scenarios" / "forest_rescue.yaml")
    scn.gazebo.update(gazebo)
    return scn


def test_the_challenge_scenario_has_ten_rois_and_no_fixed_seed():
    targets = _scenario().world.targets
    assert targets.roi_count == 10
    assert targets.roi_seed is None


def test_random_spells_a_fresh_seed(tmp_path):
    path = tmp_path / "s.yaml"
    path.write_text("fleet: []\ntargets: {rois: {count: 2, seed: random}}\n")
    assert scenario_mod.load(path).world.targets.roi_seed is None
    path.write_text("fleet: []\ntargets: {rois: {count: 2, seed: 5}}\n")
    assert scenario_mod.load(path).world.targets.roi_seed == 5


def test_every_up_draws_a_fresh_layout(monkeypatch, tmp_path):
    monkeypatch.setattr(orchestrator, "RUNS_DIR", tmp_path)
    a = orchestrator.plan(_scenario(), check_environment=False)
    b = orchestrator.plan(_scenario(), check_environment=False)
    assert a.world.targets.roi_seed != b.world.targets.roi_seed
    assert [(r.x, r.y) for r in a.world.rois] != [(r.x, r.y) for r in b.world.rois]


def test_an_explicit_seed_replays_a_layout(monkeypatch, tmp_path):
    monkeypatch.setattr(orchestrator, "RUNS_DIR", tmp_path)
    a = orchestrator.plan(_scenario(), check_environment=False, roi_seed=42)
    b = orchestrator.plan(_scenario(), check_environment=False, roi_seed=42)
    assert a.world.targets.roi_seed == 42
    assert [(r.x, r.y, r.yaw) for r in a.world.rois] == [(r.x, r.y, r.yaw) for r in b.world.rois]


def test_commands_on_a_running_run_keep_its_layout(monkeypatch, tmp_path):
    """'uavsim spawn' re-plans and rewrites the manifest; with a fresh seed it would
    describe ROIs that are not in the running world."""
    monkeypatch.setattr(orchestrator, "RUNS_DIR", tmp_path)
    calls: list[str] = []
    monkeypatch.setattr(orchestrator, "Supervisor", lambda **kw: _Recorder(calls, **kw))
    monkeypatch.setattr(orchestrator, "Spawner", lambda **kw: _Recorder(calls, **kw))

    up_plan = orchestrator.plan(_scenario(), check_environment=False)
    orchestrator.up(up_plan, wait_ready=False, start_bridge=False)
    seed = up_plan.world.targets.roi_seed

    state = orchestrator.read_run_state(up_plan.run_dir)
    assert state["roi_seed"] == seed
    for attached in (orchestrator.plan(_scenario(), check_environment=False, attach=True),
                     orchestrator.plan(_scenario(start=False), check_environment=False)):
        assert attached.world.targets.roi_seed == seed
        assert [(r.x, r.y) for r in attached.world.rois] == \
            [(r.x, r.y) for r in up_plan.world.rois]


def _recorded(monkeypatch, tmp_path) -> list[str]:
    monkeypatch.setattr(orchestrator, "RUNS_DIR", tmp_path)
    calls: list[str] = []
    monkeypatch.setattr(orchestrator, "Supervisor", lambda **kw: _Recorder(calls, **kw))
    monkeypatch.setattr(orchestrator, "Spawner", lambda **kw: _Recorder(calls, **kw))
    return calls


def test_up_writes_the_answers_only_to_the_judges_log(monkeypatch, tmp_path):
    import json

    _recorded(monkeypatch, tmp_path)
    plan = orchestrator.plan(_scenario(), check_environment=False)
    result = orchestrator.up(plan, wait_ready=False, start_bridge=False, team="Team Ørn #1")

    log_dir = result["log_dir"]
    assert log_dir.parent == tmp_path / "uavsim_runs"
    assert log_dir.name.startswith("Team-rn-1_")
    truth = json.loads((log_dir / "ground_truth.json").read_text())
    assert truth["roi_seed"] == plan.world.targets.roi_seed
    assert len(truth["rois"]) == 10
    # not next to the manifest, where the participants' code looks
    assert not (plan.run_dir / "ground_truth.json").exists()
    manifest = json.loads(result["manifest_path"].read_text())
    assert "rois" not in manifest["targets"]
    assert manifest["targets"]["roi_count"] == 10
    assert str(log_dir) not in result["manifest_path"].read_text()

    info = json.loads((log_dir / "run_info.json").read_text())
    assert info["team"] == "Team-rn-1" and info["roi_seed"] == plan.world.targets.roi_seed
    assert (log_dir / "scenario.yaml").read_text() == plan.scenario.path.read_text()
    assert orchestrator.read_run_state(plan.run_dir)["log_dir"] == str(log_dir)


def test_the_pose_recorder_starts_before_any_vehicle_is_spawned(monkeypatch, tmp_path):
    calls = _recorded(monkeypatch, tmp_path)
    orchestrator.up(orchestrator.plan(_scenario(), check_environment=False),
                    wait_ready=False, start_bridge=False)
    spawns = [i for i, c in enumerate(calls) if c.startswith("spawn:")]
    assert f"run:{runlog.RECORDER}" in calls
    assert calls.index(f"run:{runlog.RECORDER}") < min(spawns)


def test_logging_can_be_turned_off(monkeypatch, tmp_path):
    calls = _recorded(monkeypatch, tmp_path)
    scn = _scenario()
    scn.logging["enabled"] = False
    plan = orchestrator.plan(scn, check_environment=False)
    result = orchestrator.up(plan, wait_ready=False, start_bridge=False)
    assert result["log_dir"] is None
    assert f"run:{runlog.RECORDER}" not in calls
    assert not (tmp_path / "uavsim_runs").exists()
    assert (plan.run_dir / "ground_truth.json").exists()


def test_down_closes_and_packs_the_log(monkeypatch, tmp_path):
    import json
    import tarfile

    _recorded(monkeypatch, tmp_path)
    plan = orchestrator.plan(_scenario(), check_environment=False)
    log_dir = orchestrator.up(plan, wait_ready=False, start_bridge=False)["log_dir"]

    report = orchestrator.down(orchestrator.plan(_scenario(), check_environment=False,
                                                 attach=True))
    assert "pose recorder: not running" in report
    archive = log_dir.with_name(log_dir.name + ".tar.gz")
    assert any(str(archive) in line for line in report)
    with tarfile.open(archive) as tar:
        names = tar.getnames()
    for f in ("run_info.json", "ground_truth.json", "scenario.yaml", "SHA256SUMS"):
        assert f"{log_dir.name}/{f}" in names
    sums = (log_dir / "SHA256SUMS").read_text()
    assert "ground_truth.json" in sums and "run_info.json" in sums
    info = json.loads((log_dir / "run_info.json").read_text())
    assert info["ended_utc"] and len(info["fleet"]) == 4
    # a second down must not pack it again
    assert orchestrator.read_run_state(plan.run_dir).get("log_dir") is None


def test_down_stops_gazebo_unless_asked_to_keep_it(monkeypatch, tmp_path):
    """A server left behind after its GUI window is closed runs on invisibly and
    breaks the next replay of that world, so stopping it is the default."""
    calls = _recorded(monkeypatch, tmp_path)
    plan = orchestrator.plan(_scenario(), check_environment=False)
    orchestrator.up(plan, wait_ready=False, start_bridge=False)
    attached = orchestrator.plan(_scenario(), check_environment=False, attach=True)

    orchestrator.down(attached, stop_gazebo=False)
    assert not [c for c in calls if c.startswith("kill:")]
    assert orchestrator.read_run_state(plan.run_dir)["roi_seed"] == plan.world.targets.roi_seed

    orchestrator.down(attached)
    assert "kill:gz-sim-server" in calls
    assert orchestrator.read_run_state(plan.run_dir) == {}


def test_logs_go_to_the_repo_by_default(monkeypatch):
    monkeypatch.delenv("UAVSIM_LOG_DIR")
    assert runlog.log_root() == REPO_ROOT / "uavsim_runs"
