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

from uavsim import orchestrator, scenario as scenario_mod      # noqa: E402


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
    # and PX4 spawns its own model when its process starts, so that must wait too
    px4 = [i for i, c in enumerate(calls) if c.startswith("run:px4")]
    assert px4 and calls.index("wait_for_gui") < px4[0]


def test_headless_runs_do_not_wait_for_a_gui(recorded_up):
    assert "wait_for_gui" not in recorded_up(headless=True, start=True)


def test_attaching_to_an_existing_server_does_not_wait_for_a_gui(recorded_up):
    # with start: false the GUI, if any, is the user's and is already up
    assert "wait_for_gui" not in recorded_up(headless=False, start=False)
