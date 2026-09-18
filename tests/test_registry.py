"""Registry, resolution and rendering tests. No Gazebo, no SITL, no network."""

from __future__ import annotations

import sys
import xml.dom.minidom
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from uavsim.adapter import Pose, WorldInfo                       # noqa: E402
from uavsim.allocator import AllocationError, FleetRequest, allocate  # noqa: E402
from uavsim.registry import Registry, RegistryError              # noqa: E402
from uavsim.sdf import render_model                              # noqa: E402
from uavsim.site import DEFAULT_GCS_POSE, DEFAULT_SITE, SiteLayout  # noqa: E402

RUN_DIR = Path("/tmp/uavsim-tests")


@pytest.fixture(scope="module")
def registry() -> Registry:
    return Registry()


@pytest.fixture(scope="module")
def world() -> WorldInfo:
    return WorldInfo(name="test", sdf_path=Path("test.sdf"), latitude_deg=-35.389138,
                     longitude_deg=149.219188, elevation_m=580.0)


# -- registry ----------------------------------------------------------------

def test_every_registry_file_validates(registry):
    assert registry.airframes and registry.backends and registry.payloads and registry.vehicles


def test_every_vehicle_resolves(registry):
    for vehicle_id in registry.vehicles:
        resolved = registry.resolve(vehicle_id)
        assert resolved.binding.overlay or resolved.binding.model_uri


def test_every_backend_adapter_loads(registry, world):
    for backend_id, backend in registry.backends.items():
        adapter = registry.adapter_class(backend_id)(backend.settings, world)
        assert adapter.id == backend_id


def test_unknown_vehicle_names_the_known_ones(registry):
    with pytest.raises(RegistryError) as exc:
        registry.resolve("no_such_vehicle")
    assert "ardupilot_iris_gimbal" in str(exc.value)


def test_binding_must_exist_for_backend(registry):
    # zephyr has no px4 binding; resolving that pair must fail loudly
    registry.vehicles["ardupilot_zephyr"]
    airframe = registry.airframes["zephyr"]
    assert "px4" not in airframe.bindings


# -- capabilities ------------------------------------------------------------

def test_capabilities_merge_airframe_backend_and_payloads(registry):
    caps = registry.resolve("ardupilot_iris_gimbal").capabilities(name="v0", world="w")
    assert caps["platform"]["class"] == "multirotor"
    assert caps["platform"]["hover"] is True
    assert caps["platform"]["vtol"] is False
    assert caps["autopilot"]["backend"] == "ardupilot"
    assert "mavlink2" in caps["autopilot"]["protocols"]
    kinds = {s["kind"] for s in caps["sensors"]}
    assert {"camera.rgb", "imu"} <= kinds
    assert caps["actuators"][0]["kind"] == "gimbal.3axis"
    assert "/v0/gimbal_cam/image_raw" == caps["sensors"][0]["ros"]["topic"]
    assert "world/w/" in caps["sensors"][0]["gz_topic"]


def test_vtol_class_reports_vtol(registry):
    caps = registry.resolve("px4_standard_vtol").capabilities()
    assert caps["platform"]["vtol"] is True and caps["platform"]["hover"] is True


# -- allocation --------------------------------------------------------------

def _allocate(registry, requests, site=DEFAULT_SITE):
    return allocate(requests, registry=registry, site=site, gcs_pose=DEFAULT_GCS_POSE,
                    run_dir=RUN_DIR)


def test_mixed_fleet_gets_unique_names_sysids_and_instances(registry):
    insts = _allocate(registry, [FleetRequest("ardupilot_iris_gimbal", 3),
                                 FleetRequest("px4_x500", 2)])
    assert [i.name for i in insts] == [
        "ardupilot_iris_gimbal_0", "ardupilot_iris_gimbal_1", "ardupilot_iris_gimbal_2",
        "px4_x500_0", "px4_x500_1"]
    assert [i.sysid for i in insts] == [1, 2, 3, 4, 5]
    # per-backend instance numbers restart at zero, which is what -I / -i expect
    assert [i.backend_index for i in insts] == [0, 1, 2, 0, 1]


def test_fdm_ports_are_unique_per_ardupilot_instance(registry, world):
    insts = _allocate(registry, [FleetRequest("ardupilot_iris_gimbal", 4)])
    adapter = registry.adapter_class("ardupilot")(registry.backends["ardupilot"].settings, world)
    ports = [adapter.fdm_port(i) for i in insts]
    assert ports == [9002, 9012, 9022, 9032]
    assert len(set(ports)) == len(ports)


def test_ardupilot_gcs_ports_avoid_px4s_onboard_mavlink_range(registry, world):
    # PX4 SITL binds 14580+instance for its own onboard MAVLink link. If the
    # ArduPilot MAVProxy output ports overlap that range the collision is
    # silent: PX4 wins the bind and the ArduPilot vehicle never reaches a GCS.
    # A 14550 base with a stride of 10 put the 4th Iris on 14580, which is
    # exactly the shipped forest_rescue fleet.
    insts = _allocate(registry, [FleetRequest("ardupilot_iris_gimbal", 8)])
    adapter = registry.adapter_class("ardupilot")(registry.backends["ardupilot"].settings, world)
    gcs_ports = [adapter.gcs_port(i) for i in insts]
    px4_reserved = set(range(14540, 14600))
    assert not px4_reserved.intersection(gcs_ports), (
        f"ArduPilot GCS ports {sorted(set(gcs_ports) & px4_reserved)} collide with PX4")
    assert len(set(gcs_ports)) == len(gcs_ports)


def test_fixed_wing_goes_to_the_runway_multirotor_to_the_pads(registry):
    insts = _allocate(registry, [FleetRequest("ardupilot_zephyr", 1),
                                 FleetRequest("ardupilot_iris_gimbal", 1)])
    zephyr, iris = insts
    assert zephyr.pose.x > iris.pose.x        # runway is closer to the ROI than the apron
    assert abs(zephyr.pose.y) < 1e-6          # on the centreline


def test_vehicles_face_the_roi(registry):
    insts = _allocate(registry, [FleetRequest("ardupilot_iris_gimbal", 6)])
    for inst in insts:
        assert abs(inst.pose.yaw) < 0.2       # roughly +X, toward the world origin


def test_pad_spacing_below_footprint_is_rejected(registry):
    tight = SiteLayout(**{**DEFAULT_SITE.__dict__, "pad_spacing_m": 0.5})
    with pytest.raises(AllocationError, match="pad spacing"):
        _allocate(registry, [FleetRequest("ardupilot_iris_gimbal", 1)], site=tight)


def test_running_out_of_pads_is_an_error_not_a_pile_up(registry):
    with pytest.raises(AllocationError, match="landing pads"):
        _allocate(registry, [FleetRequest("ardupilot_iris_gimbal", DEFAULT_SITE.pad_capacity + 1)])


def test_singleton_payload_cannot_be_requested_twice(registry):
    with pytest.raises(AllocationError, match="singleton"):
        _allocate(registry, [FleetRequest("px4_x500_depth", 2)])


def test_spawn_positions_do_not_overlap(registry):
    insts = _allocate(registry, [FleetRequest("ardupilot_iris_gimbal", 8),
                                 FleetRequest("px4_x500", 4)])
    for a in insts:
        for b in insts:
            if a is b:
                continue
            gap = ((a.pose.x - b.pose.x) ** 2 + (a.pose.y - b.pose.y) ** 2) ** 0.5
            clearance = (a.vehicle.airframe.footprint_radius_m
                         + b.vehicle.airframe.footprint_radius_m)
            assert gap > clearance, f"{a.name} and {b.name} overlap"


# -- rendering ---------------------------------------------------------------

def _render(registry, world, vehicle_id, count=1):
    insts = _allocate(registry, [FleetRequest(vehicle_id, count)])
    out = []
    for inst in insts:
        adapter = registry.adapter_class(inst.vehicle.backend.id)(
            registry.backends[inst.vehicle.backend.id].settings, world)
        out.append((inst, render_model(inst, world_name=world.name,
                                       backend_vars=adapter.sdf_vars(inst))))
    return out


def test_every_vehicle_renders_well_formed_sdf(registry, world):
    for vehicle_id in registry.vehicles:
        for inst, sdf in _render(registry, world, vehicle_id):
            assert sdf is not None, vehicle_id
            doc = xml.dom.minidom.parseString(sdf)
            model = doc.getElementsByTagName("model")[0]
            assert model.getAttribute("name") == inst.name


def test_rendered_instances_are_isolated(registry, world):
    rendered = _render(registry, world, "ardupilot_iris_gimbal", count=3)
    ports = {s.split("<fdm_port_in>")[1].split("<")[0] for _, s in rendered}
    assert ports == {"9002", "9012", "9022"}
    for inst, sdf in rendered:
        assert f"/{inst.name}/gimbal_cam/cmd_roll" in sdf
        assert ">/gimbal/cmd_" not in sdf          # never the shared legacy topic


def test_no_instance_adds_a_plane_collision(registry, world):
    """A second infinite plane makes the ground-contact LCP singular and hangs physics."""
    for vehicle_id in registry.vehicles:
        for _, sdf in _render(registry, world, vehicle_id):
            assert "<plane>" not in sdf


def test_payload_is_reflected_in_both_sdf_and_capabilities(registry, world):
    (inst, sdf), = _render(registry, world, "ardupilot_iris_gimbal")
    caps = inst.vehicle.capabilities(name=inst.name, world=world.name)
    camera = next(s for s in caps["sensors"] if s["kind"] == "camera.rgb")
    assert "gimbal_cam" in sdf
    assert camera["gz_topic"].startswith(f"/world/{world.name}/model/{inst.name}/")


def test_static_backend_has_no_autopilot_plugin(registry, world):
    (_, sdf), = _render(registry, world, "static_iris")
    assert "ArduPilotPlugin" not in sdf
