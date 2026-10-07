"""Tests for the reference clients in examples/swarm/.

These run in CI with no simulator, no autopilot and no ROS, which is why ``assign.py``
and ``manifest_io.py`` import neither pymavlink nor rclpy: the interesting logic -- the
assignment and the manifest contract -- stays testable.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))
# the examples are standalone scripts, not a package: run from their own directory they
# find each other on sys.path, and so do we
sys.path.insert(0, str(REPO_ROOT / "examples" / "swarm"))

import assign as assign_mod                                      # noqa: E402
import manifest_io                                               # noqa: E402


def _manifest(vehicles: int = 4, rois: int = 8, *, extra_out: bool = True) -> dict:
    return {
        "schema_version": 2,
        "world": {"name": "rescue",
                  "origin": {"latitude_deg": -35.389138, "longitude_deg": 149.219188,
                             "elevation_m": 580.0}},
        "roi": {"id": "roi_forest", "mission_radius_m": 85.0},
        "targets": {
            "fires": [], "casualties": [],
            "rois": [{
                "id": f"roi_{i}", "index": i, "type": "survivor",
                "enu": [10.0 * i, -5.0 * i], "yaw_rad": 0.5,
                "geo": [-35.389 + 0.0001 * i, 149.219 + 0.0001 * i],
                "marker": {"dictionary": "4x4_50", "aruco_id": i, "size_m": 2.0,
                           "model": f"aruco_marker_{i}",
                           "enu": [10.0 * i + 3, -5.0 * i],
                           "geo": [-35.389 + 0.0001 * i, 149.2192 + 0.0001 * i]},
            } for i in range(rois)],
        },
        "vehicles": [{
            "name": f"ardupilot_iris_gimbal_{i}", "type": "ardupilot_iris_gimbal",
            "backend": "ardupilot", "index": i, "sysid": i + 1,
            "spawn": {"enu": [-426.0, -15.0 + 6.0 * i, 0.0], "geo": [-35.39, 149.214]},
            "endpoints": {"mavlink": f"udp:127.0.0.1:{14650 + 10 * i}",
                          "extra_out": [f"127.0.0.1:{14750 + i}"] if extra_out else []},
        } for i in range(vehicles)] + [{
            "name": "px4_x500_0", "backend": "px4", "index": 0, "sysid": 90,
            "spawn": {"enu": [-426.0, 9.0, 0.0], "geo": [-35.39, 149.214]},
            "endpoints": {"mavlink_udp": "udpout:127.0.0.1:18570"},
        }],
    }


@pytest.fixture
def run() -> manifest_io.Run:
    return manifest_io.parse(_manifest())


# -- manifest_io ---------------------------------------------------------------

def test_manifest_io_round_trips_a_manifest(run):
    assert run.world == "rescue"
    assert run.mission_radius_m == 85.0
    assert run.origin == (-35.389138, 149.219188, 580.0)
    assert len(run.vehicles) == 5 and len(run.rois) == 8
    assert [r.marker.aruco_id for r in run.rois] == list(range(8))
    assert run.rois[3].lat == run.rois[3].geo[0]


def test_ardupilot_and_px4_are_separated(run):
    assert [v.name for v in manifest_io.px4(run)] == ["px4_x500_0"]
    assert all(v.backend == "ardupilot" for v in manifest_io.ardupilot(run))
    assert len(manifest_io.ardupilot(run)) == 4


def test_px4_endpoint_falls_back_to_mavlink_udp(run):
    assert manifest_io.px4(run)[0].mavlink == "udpout:127.0.0.1:18570"


def test_fcu_url_prefers_extra_out_then_udp_then_tcp():
    """The main 'mavlink' port is one UDP socket; sharing it silently starves a client."""
    with_extra = manifest_io.ardupilot(manifest_io.parse(_manifest()))[1]
    assert manifest_io.fcu_url(with_extra) == "udp://:14751@"

    without = manifest_io.ardupilot(manifest_io.parse(_manifest(extra_out=False)))[1]
    assert manifest_io.fcu_url(without) == "udp://:14660@"

    headless = manifest_io.Vehicle(name="v", sysid=1, backend="ardupilot", index=0,
                                   mavlink="tcp:127.0.0.1:5760")
    assert manifest_io.fcu_url(headless) == "tcp://127.0.0.1:5760"

    with pytest.raises(RuntimeError, match="extra_out"):
        manifest_io.fcu_url(manifest_io.Vehicle(name="v", sysid=1, backend="ardupilot",
                                                index=0, mavlink=None))


def test_load_run_names_uavsim_up_when_there_is_no_manifest(tmp_path):
    with pytest.raises(FileNotFoundError, match="uavsim up"):
        manifest_io.load_run(tmp_path / "nothing")


def test_load_run_reads_a_file_on_disk(tmp_path):
    path = tmp_path / "manifest.json"
    path.write_text(json.dumps(_manifest()))
    assert manifest_io.load_run(path).world == "rescue"
    assert manifest_io.load_run(tmp_path).world == "rescue"      # directory also works


# -- assignment ----------------------------------------------------------------

@pytest.mark.parametrize("strategy", assign_mod.STRATEGIES)
def test_assignment_covers_every_roi_exactly_once(run, strategy):
    taskings = assign_mod.assign(manifest_io.ardupilot(run), run.rois, strategy=strategy)
    assigned = [leg.roi.index for t in taskings for leg in t.legs]
    assert sorted(assigned) == list(range(8))
    assert len(assigned) == len(set(assigned))


@pytest.mark.parametrize("strategy", assign_mod.STRATEGIES)
def test_assignment_is_deterministic(run, strategy):
    fleet = manifest_io.ardupilot(run)
    first = assign_mod.assign(fleet, run.rois, strategy=strategy)
    second = assign_mod.assign(fleet, run.rois, strategy=strategy)
    assert [[leg.roi.index for leg in t.legs] for t in first] \
        == [[leg.roi.index for leg in t.legs] for t in second]


def test_nearest_balances_the_load(run):
    """One well-placed vehicle must not take the whole forest."""
    taskings = assign_mod.assign(manifest_io.ardupilot(run), run.rois, strategy="nearest")
    assert [len(t.legs) for t in taskings] == [2, 2, 2, 2]


def test_round_robin_deals_the_rois_in_order(run):
    taskings = assign_mod.assign(manifest_io.ardupilot(run), run.rois,
                                 strategy="round_robin")
    assert [[leg.roi.index for leg in t.legs] for t in taskings] \
        == [[0, 4], [1, 5], [2, 6], [3, 7]]


def test_cruise_altitudes_are_deconflicted(run):
    """Altitude layering is the only collision avoidance these examples have."""
    fleet = manifest_io.ardupilot(run)
    levels = assign_mod.cruise_altitudes(fleet, base_m=15.0, separation_m=3.0)
    values = sorted(levels.values())
    assert len(set(values)) == len(fleet)
    assert all(b - a >= 3.0 - 1e-9 for a, b in zip(values, values[1:]))
    assert values == [15.0, 18.0, 21.0, 24.0]


def test_the_observation_altitude_stays_detectable(run):
    """Every leg must be flown low enough for the 2 m plate to resolve: at 25 m it is
    under 3 px per module and OpenCV stops seeing it."""
    taskings = assign_mod.assign(manifest_io.ardupilot(run), run.rois, observe_alt_m=15.0)
    assert all(leg.observe_alt_m <= 20.0 for t in taskings for leg in t.legs)


def test_assign_rejects_an_unknown_strategy_and_an_empty_fleet(run):
    with pytest.raises(ValueError, match="unknown strategy"):
        assign_mod.assign(manifest_io.ardupilot(run), run.rois, strategy="spiral")
    with pytest.raises(ValueError, match="no vehicles"):
        assign_mod.assign([], run.rois)


def test_describe_names_every_vehicle_and_marker(run):
    text = assign_mod.describe(
        assign_mod.assign(manifest_io.ardupilot(run), run.rois))
    for vehicle in manifest_io.ardupilot(run):
        assert vehicle.name in text
    for roi in run.rois:
        assert f"aruco {roi.marker.aruco_id}" in text
    assert "8 ROIs across 4 vehicles" in text
