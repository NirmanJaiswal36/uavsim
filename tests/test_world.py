"""World building, bridge generation and manifest tests."""

from __future__ import annotations

import itertools
import json
import math
import re
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
from uavsim.site import DEFAULT_GCS_POSE, DEFAULT_SITE, compose  # noqa: E402
from uavsim.world import DEFAULT_ROI_TYPES, camera_eye_and_target  # noqa: E402
from uavsim.world import (PROP_FOOTPRINTS, TargetSpec, WorldSpec, build,  # noqa: E402
                          enu_to_geo, gcs_standoff_m, load_roi, marker_offset_m,
                          place_casualties, place_rois)


def _inside(roi, x, y) -> bool:
    xmin, xmax, ymin, ymax = roi["mission_extent"]
    return xmin <= x <= xmax and ymin <= y <= ymax


@pytest.fixture(scope="module")
def registry() -> Registry:
    return Registry()


@pytest.fixture(scope="module")
def built_with_rois(tmp_path_factory):
    spec = WorldSpec(name="roi_world",
                     targets=TargetSpec(fires=False, roi_count=10, roi_seed=17))
    return build(spec, tmp_path_factory.mktemp("roi_worlds"))


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
    stripped = re.sub(r"<!--.*?-->", "", text, flags=re.S)
    assert stripped.count("<plane>") == 1


def test_world_contains_no_vehicles(built):
    text = built.info.sdf_path.read_text()
    assert "ArduPilotPlugin" not in text
    assert "model://drone" not in text
    assert "iris_with_standoffs" not in text


def test_world_contains_the_roi_the_gcs_and_the_targets(built):
    text = built.info.sdf_path.read_text()
    assert "grasspatch_visual" in text and "oak tree" in text   # scenery came across
    assert "gcs_site" in text and "runway" not in text
    assert text.count('<model name="fire_') == len(built.fires) == 40
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


def test_casualties_stay_inside_the_forest_and_clear_of_trees():
    roi = load_roi("roi_forest")
    placed = place_casualties(roi, TargetSpec(casualty_count=10, casualty_seed=5),
                              [list(f) for f in roi["fires"]])
    for x, y, _ in placed:
        assert _inside(roi, x, y)
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
    assert data["gcs"]["distance_to_roi_m"] == 365.0
    assert data["gcs"]["standoff_m"] == 100.0
    assert "runway" not in data["gcs"]
    assert data["roi"]["size_m"] == [500.0, 500.0]
    assert data["roi"]["mission_extent_enu"] == [-250.0, 250.0, -250.0, 250.0]
    assert len(data["roi"]["boundary_geo"]) == 4
    assert "fires" not in data["targets"] and "casualties" not in data["targets"]
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


def test_the_gcs_leaves_100_m_of_open_ground_to_the_forest(built):
    """The apron's front edge is the forest-facing extremity of the GCS, so that --
    not the site origin -- is what must clear the trees. See the arithmetic in
    uavsim/site.py."""
    site = built.site
    front = built.site_pose.x + site.front_x_m
    assert built.roi["mission_extent"][0] - front == pytest.approx(100.0, abs=0.5)
    assert gcs_standoff_m(built.site_pose, site, built.roi) == pytest.approx(100.0)
    pads = [compose(built.site_pose, slot) for slot in site.pad_slots()]
    assert max(p.x + site.pad_radius_m for p in pads) < front   # pads are on the apron
    nearest_tree = min(x for x, _ in built.roi["trees"])
    assert nearest_tree - front >= 100.0


def test_the_forest_is_a_500_m_square_centred_on_the_origin():
    roi = load_roi("roi_forest")
    assert roi["size_m"] == [500.0, 500.0]
    assert roi["mission_extent"] == [-250.0, 250.0, -250.0, 250.0]
    assert roi["ground_extent"] == roi["mission_extent"]       # grass covers it exactly
    assert all(_inside(roi, x, y) for x, y in roi["trees"])
    xs, ys = [t[0] for t in roi["trees"]], [t[1] for t in roi["trees"]]
    assert min(xs) < -225 and max(xs) > 225 and min(ys) < -225 and max(ys) > 225


# -- regions of interest -------------------------------------------------------

def test_rois_are_placed_clear_of_trunks_fires_and_each_other():
    """An ROI under a canopy is an ROI whose marker is never seen, and two ROIs in one
    camera frame make a detection impossible to attribute."""
    roi = load_roi("roi_forest")
    fires = [list(f) for f in roi["fires"]]
    placed = place_rois(roi, TargetSpec(roi_count=8, roi_seed=17), fires)
    assert len(placed) == 8
    assert len({r.aruco_id for r in placed}) == 8
    assert {r.type for r in placed} <= set(DEFAULT_ROI_TYPES)
    for r in placed:
        assert _inside(roi, r.x, r.y)
        assert all(math.hypot(r.x - tx, r.y - ty) >= 6.0 for tx, ty in roi["trees"])
        assert all(math.hypot(r.x - fx, r.y - fy) >= 8.0 for fx, fy, _ in fires)
    for a, b in itertools.combinations(placed, 2):
        assert math.hypot(a.x - b.x, a.y - b.y) >= 15.0


def test_the_roi_marker_sits_beside_the_prop_not_under_it():
    placed = place_rois(load_roi("roi_forest"), TargetSpec(roi_count=10, roi_seed=17))
    for r in placed:
        assert r.marker_offset_m == marker_offset_m(r.type, 2.0)
        assert math.hypot(r.marker_x - r.x, r.marker_y - r.y) \
            == pytest.approx(r.marker_offset_m)


def test_marker_offsets_clear_every_prop_footprint():
    """The wreckage's 3.2 m scorch disc used to sit under a plate 3 m out, coplanar
    with it -- the plate flickered in and out. Every plate, at any relative rotation,
    must now clear its own prop's footprint."""
    assert set(PROP_FOOTPRINTS) == set(DEFAULT_ROI_TYPES)
    for roi_type in DEFAULT_ROI_TYPES:
        offset = marker_offset_m(roi_type, 2.0)
        for cx, cy, r in PROP_FOOTPRINTS[roi_type]:
            # the plate is inside a circle of radius sqrt(2) around its centre
            assert math.hypot(offset - cx, cy) >= r + math.sqrt(2.0)
    assert marker_offset_m("wreckage", 2.0) > 3.2 + math.sqrt(2.0)


def _plate_points(r, size_m: float = 2.0, n: int = 5):
    """A grid over the plate (axis-aligned in the world), in world coordinates."""
    h = size_m / 2.0
    steps = [-h + size_m * i / (n - 1) for i in range(n)]
    return [(r.marker_x + dx, r.marker_y + dy) for dx in steps for dy in steps]


def _in_footprint(prop, x: float, y: float) -> bool:
    c, s_ = math.cos(-prop.yaw), math.sin(-prop.yaw)
    lx = c * (x - prop.x) - s_ * (y - prop.y)
    ly = s_ * (x - prop.x) + c * (y - prop.y)
    return any(math.hypot(lx - cx, ly - cy) < r for cx, cy, r in PROP_FOOTPRINTS[prop.type])


def test_ten_visible_rois_place_for_any_seed():
    """Every run draws a fresh seed, so the invariants have to hold for all of them:
    the plate out from under every canopy and every prop, inside the search area,
    and apart from every other plate."""
    roi = load_roi("roi_forest")
    xmin, xmax, ymin, ymax = roi["mission_extent"]
    trees = roi["trees"]
    for seed in range(500):
        placed = place_rois(roi, TargetSpec(roi_count=10, roi_seed=seed))
        assert len(placed) == 10
        assert sorted(r.aruco_id for r in placed) == list(range(10))
        assert sorted(r.site for r in placed) == list(range(10))
        for r in placed:
            assert xmin + 6 <= r.marker_x <= xmax - 6 and ymin + 6 <= r.marker_y <= ymax - 6
            assert all(math.hypot(r.marker_x - tx, r.marker_y - ty) >= 7.5
                       for tx, ty in trees), (seed, r)
            for prop in placed:
                assert not any(_in_footprint(prop, x, y) for x, y in _plate_points(r)), \
                    (seed, r, prop)
        for a, b in itertools.combinations(placed, 2):
            assert math.hypot(a.marker_x - b.marker_x, a.marker_y - b.marker_y) >= 15.0
            assert math.hypot(a.x - b.x, a.y - b.y) >= 15.0


def test_ten_rois_are_two_of_each_type():
    placed = place_rois(load_roi("roi_forest"), TargetSpec(roi_count=10, roi_seed=3))
    types = [r.type for r in placed]
    assert all(types.count(t) == 2 for t in DEFAULT_ROI_TYPES)


def test_different_seeds_give_different_layouts():
    roi = load_roi("roi_forest")
    a = place_rois(roi, TargetSpec(roi_count=10, roi_seed=1))
    b = place_rois(roi, TargetSpec(roi_count=10, roi_seed=2))
    assert [(r.x, r.y) for r in a] != [(r.x, r.y) for r in b]


def test_an_unresolved_seed_is_refused():
    with pytest.raises(ValueError):
        place_rois(load_roi("roi_forest"), TargetSpec(roi_count=10, roi_seed=None))


def test_roi_placement_is_deterministic():
    roi, spec = load_roi("roi_forest"), TargetSpec(roi_count=8, roi_seed=17)
    assert place_rois(roi, spec) == place_rois(roi, spec)


def test_explicit_roi_positions_are_used_verbatim():
    spec = TargetSpec(roi_count=2, roi_positions=[[1.0, 2.0], [3.0, 4.0, 0.5]])
    placed = place_rois(load_roi("roi_forest"), spec)
    assert [(r.x, r.y, r.yaw) for r in placed] == [(1.0, 2.0, 0.0), (3.0, 4.0, 0.5)]


def test_rois_add_no_plane_and_no_fuel_model_to_the_world(built_with_rois):
    """ROI props are primitives on purpose: they must not add to first-run download
    time, and they must not add a second plane collision."""
    text = built_with_rois.info.sdf_path.read_text()
    stripped = re.sub(r"<!--.*?-->", "", text, flags=re.S)
    assert stripped.count("<plane>") == 1
    assert len(re.findall(r'<model name="site_\d+">', text)) == 10
    for marker_id in range(10):
        assert f"model://aruco_marker_{marker_id}" in text
    trees = built_with_rois.roi["tree_count"]
    assert text.count("fuel.gazebosim.org") == 4 * trees   # 2 meshes + 2 textures per tree, nothing new


def test_entity_names_do_not_give_the_answer_away(built_with_rois):
    """gz lists entity names to anyone who asks, so they must not say which site
    holds which marker id or prop type."""
    text = built_with_rois.info.sdf_path.read_text()
    names = re.findall(r'<model name="(site_[^"]*)"', text) \
        + re.findall(r"<name>(site_[^<]*)</name>", text)
    assert len(names) == 20
    for name in names:
        assert re.fullmatch(r"site_\d+(_plate)?", name)
    assert not re.search(r"roi_\d|aruco_\d", re.sub(r"<!--.*?-->", "", text, flags=re.S))
    # and the shuffle really is one: site k is not simply roi k
    assert [r.site for r in built_with_rois.rois] != list(range(10))


def test_the_manifest_does_not_publish_where_the_targets_are(built_with_rois):
    data = manifest_mod.build(built_with_rois, [], {})
    targets = data["targets"]
    assert targets["roi_count"] == 10
    assert targets["marker"] == {"dictionary": "4x4_50", "size_m": 2.0,
                                 "ids": list(range(10)),
                                 "normal": "+Z (horizontal, faces a nadir camera)"}
    text = json.dumps(data)
    for r in built_with_rois.rois:
        assert f"{r.x}" not in text and f"{round(r.marker_x, 2)}" not in text
    assert "roi_seed" not in text and "rois" not in targets


def test_the_ground_truth_carries_roi_and_marker_positions(built_with_rois):
    data = manifest_mod.ground_truth(built_with_rois)
    assert data["roi_seed"] == 17
    rois = data["rois"]
    assert len(rois) == 10
    assert {r["marker"]["aruco_id"] for r in rois} == set(range(10))
    for r in rois:
        assert r["type"] in DEFAULT_ROI_TYPES
        assert r["id"] == f"roi_{r['index']}"
        assert len(r["enu"]) == 2 and r["geo"][0] < 0        # southern hemisphere
        assert r["marker"]["dictionary"] == "4x4_50"
        assert r["marker"]["size_m"] == 2.0
        assert r["marker"]["model"] == f"aruco_marker_{r['marker']['aruco_id']}"


def test_disabling_rois_leaves_them_out(built):
    assert built.rois == []
    assert "aruco_marker" not in built.info.sdf_path.read_text()


# -- generated scenery ---------------------------------------------------------

def test_the_forest_fragment_matches_its_generator():
    """The fragment is generated and committed; this is what makes that safe."""
    import subprocess
    root = Path(__file__).resolve().parents[1]
    done = subprocess.run([sys.executable, "tools/gen_roi_forest.py", "--check"],
                          cwd=root, capture_output=True, text=True)
    assert done.returncode == 0, done.stdout + done.stderr


def test_the_roi_metadata_agrees_with_the_scenery():
    """Metadata drifting out of step with the SDF is the bug the generator exists
    to prevent, so assert the two against each other rather than against literals."""
    roi = load_roi("roi_forest")
    sdf = (Path(__file__).resolve().parents[1]
           / "worlds" / "fragments" / "roi_forest.sdf").read_text()
    trees = roi["tree_count"]
    assert trees == len(roi["trees"])
    assert sdf.count("oak tree/7/files/meshes") + sdf.count("pine tree/6/files/meshes") \
        == 2 * trees                            # a branch and a bark visual each
    assert sdf.count("_Tree_") == 3 * trees     # ... plus one collision
    assert sdf.count("grasspatch_visual/materials") == 34 * 34
    assert sdf.count("<plane>") == 0            # the one plane lives in base_world
    assert sdf.count("navsat_sensor") == 1      # the PX4 backend needs it
    assert len(roi["fires"]) == 40
    for x, y, z in roi["fires"]:
        assert _inside(roi, x, y)
        assert z == 0.40


def test_the_bulk_scenery_is_one_static_link():
    """One model per tree and tile makes every object a physics body with a full
    canopy-mesh collision (tools/gen_roi_forest.py has the measurements), and a flat
    entity tree buries the runtime-spawned vehicles. So everything lives on one link."""
    sdf = (Path(__file__).resolve().parents[1]
           / "worlds" / "fragments" / "roi_forest.sdf").read_text()
    stripped = re.sub(r"<!--.*?-->", "", sdf, flags=re.S)
    top_level = re.findall(r'^    <model name="([^"]+)"', stripped, re.M)
    assert top_level == ["axes", "forest_scenery"]
    assert stripped.count("<model ") == 2
    assert stripped.count("<link ") == 2
    assert "<include>" not in stripped, "every include is its own model and physics body"


def test_the_forest_keeps_the_legacy_tree_density():
    """A bigger forest at the same density, not the same trees spread thinner."""
    roi = load_roi("roi_forest")
    old_density = 100 / (math.pi * 104.7 ** 2)
    width = roi["mission_extent"][1] - roi["mission_extent"][0]
    height = roi["mission_extent"][3] - roi["mission_extent"][2]
    assert roi["tree_count"] / (width * height) == pytest.approx(old_density, rel=0.01)


def test_the_axis_marker_stays_at_world_level():
    """It carries the world's only navsat sensor, and the PX4 backend depends on
    that sensor's topic path -- nesting it would silently move the topic."""
    sdf = (Path(__file__).resolve().parents[1]
           / "worlds" / "fragments" / "roi_forest.sdf").read_text()
    assert sdf.count("navsat_sensor") == 1
    axes = sdf.index('<model name="axes">')
    scenery = sdf.index('<model name="forest_scenery">')
    assert axes < scenery, "axes must not be inside the scenery model"


# -- rendering artefacts -------------------------------------------------------

def test_ground_visual_never_overlaps_the_roi_grass(built):
    """The grass tiles' top faces are at z=0; a ground visual under them z-fights and
    the tiles flicker in and out as the camera zooms (seen in the GUI)."""
    from uavsim.world import terrain_patch
    xmin, xmax, ymin, ymax = built.roi["ground_extent"]
    patch = terrain_patch(built.site_pose, built.roi)
    assert patch is not None
    px0, px1 = patch["cx"] - patch["sx"] / 2, patch["cx"] + patch["sx"] / 2
    py0, py1 = patch["cy"] - patch["sy"] / 2, patch["cy"] + patch["sy"] / 2
    overlaps = px0 < xmax and px1 > xmin and py0 < ymax and py1 > ymin
    assert not overlaps, f"terrain {patch} overlaps grass extent {built.roi['ground_extent']}"


def test_the_default_camera_frames_the_landing_pads(built):
    """You must be able to see your own vehicles when the world comes up.

    The pads are ~120 m from the eye but tens of metres below it, so a camera aimed
    at the ROI with a fixed pitch leaves them just under the bottom edge of the
    frame -- vehicles spawn correctly and are invisible, which reads as "the drones
    are not spawning". Checked against a conservative 60x35 degree frame.
    """
    spec = WorldSpec(gcs_pose=built.site_pose, site=built.site)
    eye, target = camera_eye_and_target(spec)
    dx, dy, dz = target.x - eye.x, target.y - eye.y, target.z - eye.z
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(-dz, math.hypot(dx, dy))

    pads = [compose(built.site_pose, slot) for slot in built.site.pad_slots()]
    assert len(pads) == 24
    for pad in pads:
        px, py, pz = pad.x - eye.x, pad.y - eye.y, 0.3 - eye.z
        off_v = math.degrees(math.atan2(-pz, math.hypot(px, py)) - pitch)
        off_h = math.degrees((math.atan2(py, px) - yaw + math.pi) % (2 * math.pi) - math.pi)
        assert abs(off_v) < 17.5, f"pad {off_v:.1f} deg off vertically, outside the frame"
        assert abs(off_h) < 30.0, f"pad {off_h:.1f} deg off horizontally"


def test_the_default_camera_also_shows_the_roi(built):
    """Framing the pads must not cost us the view of what the mission is about."""
    spec = WorldSpec(gcs_pose=built.site_pose, site=built.site)
    eye, target = camera_eye_and_target(spec)
    dx, dy, dz = target.x - eye.x, target.y - eye.y, target.z - eye.z
    yaw, pitch = math.atan2(dy, dx), math.atan2(-dz, math.hypot(dx, dy))

    bx, by = built.roi["mission_extent"][0], 0.0        # near edge of the ROI
    rx, ry, rz = bx - eye.x, by - eye.y, 0.0 - eye.z
    off_v = math.degrees(math.atan2(-rz, math.hypot(rx, ry)) - pitch)
    off_h = math.degrees((math.atan2(ry, rx) - yaw + math.pi) % (2 * math.pi) - math.pi)
    assert abs(off_v) < 17.5 and abs(off_h) < 30.0, \
        f"ROI boundary {off_v:.1f}/{off_h:.1f} deg off axis, outside the frame"


def test_world_has_the_expected_lighting(built):
    # Cameras are the main sensor here, so the sun direction and background are
    # part of the contract: changing them changes every image a user collects.
    text = built.info.sdf_path.read_text()
    assert 'name="sunUTC"' in text
    assert "0.001 0.625 -0.78" in text
    assert "<background>0.7 0.7 0.7 1</background>" in text
