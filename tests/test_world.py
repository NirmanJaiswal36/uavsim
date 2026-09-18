"""World building, bridge generation and manifest tests."""

from __future__ import annotations

import sys
import xml.dom.minidom
from pathlib import Path

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from uavsim import bridge, manifest as manifest_mod              # noqa: E402
from uavsim.adapter import Pose                                  # noqa: E402
from uavsim.allocator import FleetRequest, allocate              # noqa: E402
from uavsim.registry import Registry                             # noqa: E402
from uavsim.site import DEFAULT_GCS_POSE, DEFAULT_SITE           # noqa: E402
from uavsim.world import (TargetSpec, WorldSpec, build, enu_to_geo,  # noqa: E402
                          load_roi, place_casualties)


@pytest.fixture(scope="module")
def registry() -> Registry:
    return Registry()


@pytest.fixture(scope="module")
def built(tmp_path_factory):
    spec = WorldSpec(name="test_world",
                     targets=TargetSpec(fires=True, casualty_count=4, casualty_seed=11))
    return build(spec, tmp_path_factory.mktemp("worlds"))


# -- world -------------------------------------------------------------------

def test_world_is_well_formed_and_named(built):
    doc = xml.dom.minidom.parse(str(built.info.sdf_path))
    world = doc.getElementsByTagName("world")[0]
    assert world.getAttribute("name") == "test_world"


def test_world_has_exactly_one_plane_collision(built):
    """Two infinite planes make the ground-contact LCP singular and hang physics."""
    text = built.info.sdf_path.read_text()
    import re
    stripped = re.sub(r"<!--.*?-->", "", text, flags=re.S)
    assert stripped.count("<plane>") == 1


def test_world_contains_no_vehicles(built):
    text = built.info.sdf_path.read_text()
    assert "ArduPilotPlugin" not in text
    assert "model://drone" not in text
    assert "iris_with_standoffs" not in text


def test_world_contains_the_roi_the_gcs_and_the_targets(built):
    text = built.info.sdf_path.read_text()
    assert "grasspatch" in text and "Oak Tree" in text      # scenery came across
    assert "gcs_site" in text and "runway" in text
    assert text.count('<model name="fire_') == len(built.fires) == 20
    assert text.count('<model name="casualty_') == len(built.casualties) == 4


def test_geo_origin_is_the_roi_centre(built):
    assert built.info.latitude_deg == pytest.approx(-35.389138)
    assert built.roi["centre"] == [0.0, 0.0]


def test_disabling_targets_leaves_them_out(tmp_path):
    spec = WorldSpec(name="bare", targets=TargetSpec(fires=False, casualty_count=0))
    bare = build(spec, tmp_path)
    assert bare.fires == [] and bare.casualties == []
    assert "fire_0" not in bare.info.sdf_path.read_text()


# -- targets -----------------------------------------------------------------

def test_casualty_placement_is_deterministic():
    roi = load_roi("roi_forest")
    spec = TargetSpec(casualty_count=6, casualty_seed=3)
    assert place_casualties(roi, spec) == place_casualties(roi, spec)


def test_casualties_stay_inside_the_mission_radius_and_clear_of_trees():
    roi = load_roi("roi_forest")
    placed = place_casualties(roi, TargetSpec(casualty_count=10, casualty_seed=5),
                              [list(f) for f in roi["fires"]])
    radius = roi["mission_radius_m"]
    for x, y, _ in placed:
        assert (x * x + y * y) ** 0.5 <= radius
        assert all(((x - tx) ** 2 + (y - ty) ** 2) ** 0.5 >= 3.0 for tx, ty in roi["trees"])
        assert all(((x - fx) ** 2 + (y - fy) ** 2) ** 0.5 >= 5.0 for fx, fy, _ in roi["fires"])


def test_explicit_casualty_positions_are_used_verbatim():
    roi = load_roi("roi_forest")
    spec = TargetSpec(casualty_positions=[[1.0, 2.0], [3.0, 4.0, 0.5]])
    assert place_casualties(roi, spec) == [[1.0, 2.0, 0.0], [3.0, 4.0, 0.5]]


def test_geo_conversion_round_trips_against_the_origin():
    origin = {"latitude_deg": -35.389138, "longitude_deg": 149.219188, "elevation_m": 580.0}
    lat, lon = enu_to_geo(0.0, 0.0, origin)
    assert (lat, lon) == (origin["latitude_deg"], origin["longitude_deg"])
    lat_n, _ = enu_to_geo(0.0, 1000.0, origin)
    assert (lat_n - lat) * 111320 == pytest.approx(1000.0, rel=1e-3)


# -- bridge and manifest -----------------------------------------------------

def _instances(registry, requests):
    return allocate(requests, registry=registry, site=DEFAULT_SITE,
                    gcs_pose=DEFAULT_GCS_POSE, run_dir=Path("/tmp/uavsim-tests"))


def test_bridge_config_covers_every_declared_topic(registry):
    insts = _instances(registry, [FleetRequest("ardupilot_iris_gimbal", 2)])
    entries = bridge.build_config(insts, "w")
    topics = {e["ros_topic_name"] for e in entries}
    assert "/clock" in topics
    for inst in insts:
        assert f"/{inst.name}/gimbal_cam/image_raw" in topics
        for axis in ("roll", "pitch", "yaw"):
            assert f"/{inst.name}/gimbal_cam/cmd_{axis}" in topics
    assert all(e["gz_topic_name"] for e in entries)
    assert len(topics) == len(entries), "duplicate ros topic in the bridge config"


def test_bridge_config_is_valid_yaml_for_ros_gz_bridge(registry, tmp_path):
    insts = _instances(registry, [FleetRequest("ardupilot_iris_gimbal", 1)])
    path = bridge.write_config(insts, "w", tmp_path / "bridge.yaml")
    entries = yaml.safe_load(path.read_text())
    assert isinstance(entries, list)
    for entry in entries:
        assert {"ros_topic_name", "gz_topic_name", "ros_type_name",
                "gz_type_name", "direction"} <= set(entry)


def test_manifest_reports_everything_a_framework_needs(registry, built):
    insts = _instances(registry, [FleetRequest("ardupilot_iris_gimbal", 2),
                                  FleetRequest("px4_x500", 1)])
    endpoints = {}
    for inst in insts:
        adapter = registry.adapter_class(inst.vehicle.backend.id)(
            registry.backends[inst.vehicle.backend.id].settings, built.info)
        endpoints[inst.name] = adapter.endpoints(inst)
    data = manifest_mod.build(built, insts, endpoints)

    assert data["schema_version"] == manifest_mod.SCHEMA_VERSION
    assert data["gcs"]["distance_to_roi_m"] == 500.0
    assert data["roi"]["mission_radius_m"] == 60.0
    assert len(data["targets"]["fires"]) == 20
    assert len(data["vehicles"]) == 3
    for vehicle in data["vehicles"]:
        assert vehicle["endpoints"]
        assert vehicle["spawn"]["geo"][0] < 0                    # southern hemisphere
        assert vehicle["capabilities"]["autopilot"]["backend"] in ("ardupilot", "px4")
    sysids = [v["sysid"] for v in data["vehicles"]]
    assert len(set(sysids)) == len(sysids)


def test_manifest_topics_are_per_vehicle(registry, built):
    insts = _instances(registry, [FleetRequest("ardupilot_iris_gimbal", 2)])
    data = manifest_mod.build(built, insts, {})
    topics = [s["ros"]["topic"] for v in data["vehicles"]
              for s in v["capabilities"]["sensors"] if s.get("ros")]
    assert len(set(topics)) == len(topics)


# -- rendering artefacts -------------------------------------------------------

def test_ground_visual_never_overlaps_the_roi_grass(built):
    """The grass patches are planes at z=0; a ground visual under them z-fights and
    the patches flicker in and out as the camera zooms (seen in the GUI)."""
    from uavsim.world import terrain_patch
    xmin, xmax, ymin, ymax = built.roi["ground_extent"]
    patch = terrain_patch(built.site_pose, built.roi)
    assert patch is not None
    px0, px1 = patch["cx"] - patch["sx"] / 2, patch["cx"] + patch["sx"] / 2
    py0, py1 = patch["cy"] - patch["sy"] / 2, patch["cy"] + patch["sy"] / 2
    overlaps = px0 < xmax and px1 > xmin and py0 < ymax and py1 > ymin
    assert not overlaps, f"terrain {patch} overlaps grass extent {built.roi['ground_extent']}"


def test_world_has_the_expected_lighting(built):
    # Cameras are the main sensor here, so the sun direction and background are
    # part of the contract: changing them changes every image a user collects.
    text = built.info.sdf_path.read_text()
    assert 'name="sunUTC"' in text
    assert "0.001 0.625 -0.78" in text
    assert "<background>0.7 0.7 0.7 1</background>" in text
