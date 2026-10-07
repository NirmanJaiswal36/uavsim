#!/usr/bin/env python3
"""The same mission as pymavlink_swarm.py, driven through MAVROS.

    ./bin/uavsim up scenarios/forest_rescue.yaml
    ros2 launch uavsim_bringup mavros.launch.py world:=rescue      # second terminal
    python3 examples/swarm/mavros_swarm.py --world rescue

A reference client, not part of the environment.  It shares ``manifest_io`` and
``assign`` with the pymavlink example, so the two provably fly the same plan and only
the transport differs.  Run it directly with python3; it does not need colcon.

MAVROS is not started by ``sim.launch.py`` -- that file only adds the ros_gz_bridge.
Launch ``mavros.launch.py`` first or there will be no topics to talk to.

ENU HERE, NED THERE.  MAVROS republishes everything in **ENU**, so
``<ns>/mavros/local_position/pose`` is ``x = east, y = north, z = up`` -- the same axes
as Gazebo.  ``LOCAL_POSITION_NED`` in the pymavlink example is ``x = north, y = east,
z = down``.  The two examples' "local x" are different axes.  What they share is the
*origin*: every ArduPilot vehicle here is homed at the Gazebo world origin, which is
the **centre of the forest**, not the pad it took off from (see
``uavsim/adapter.py``).  A vehicle sitting on the apron reads
``local_position/pose.x`` of about -426.

Both examples therefore command in the **global** frame --
``mavros_msgs/GlobalPositionTarget`` with ``FRAME_GLOBAL_REL_ALT`` -- which sidesteps
the convention difference entirely and consumes the manifest's lat/lon directly.
``setpoint_position/local`` would work too, at the cost of remembering which of the two
conventions you are in.

Like PX4 offboard, MAVROS will not accept the mode change until a setpoint stream is
already running, so the timer starts before ``set_mode``.
"""

from __future__ import annotations

import argparse
import math
import sys
import threading
import time

from assign import Tasking, assign, describe
from manifest_io import Vehicle, ardupilot, load_run, px4

EARTH_RADIUS_M = 6378137.0
SETPOINT_HZ = 5.0


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi, dlam = phi2 - phi1, math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(a))


class MavrosVehicle:
    """One vehicle's MAVROS interface, under ``/<vehicle name>/mavros/``."""

    def __init__(self, node, vehicle: Vehicle, home_elevation_m: float) -> None:
        from geographic_msgs.msg import GeoPointStamped  # noqa: F401  (dep check)
        from mavros_msgs.msg import GlobalPositionTarget, State
        from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, \
            QoSReliabilityPolicy
        from sensor_msgs.msg import NavSatFix

        self.node = node
        self.vehicle = vehicle
        # every vehicle here is homed at the forest centre, so this is the datum
        # that NavSatFix's AMSL altitude has to be measured against
        self.home_elevation_m = home_elevation_m
        self.ns = f"/{vehicle.name}/mavros"
        self.state: State | None = None
        self.fix: NavSatFix | None = None
        self.target: tuple[float, float, float] | None = None
        self._group = ReentrantCallbackGroup()

        # MAVROS publishes state and position as BEST_EFFORT/VOLATILE; a RELIABLE
        # subscription never matches and you get silence instead of an error
        sensor_qos = QoSProfile(
            depth=10, history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE)

        node.create_subscription(State, f"{self.ns}/state", self._on_state, sensor_qos,
                                 callback_group=self._group)
        node.create_subscription(NavSatFix, f"{self.ns}/global_position/global",
                                 self._on_fix, sensor_qos, callback_group=self._group)
        self.setpoint_pub = node.create_publisher(
            GlobalPositionTarget, f"{self.ns}/setpoint_position/global", 10)

        self.arming = node.create_client(CommandBool, f"{self.ns}/cmd/arming",
                                         callback_group=self._group)
        self.takeoff_srv = node.create_client(CommandTOL, f"{self.ns}/cmd/takeoff",
                                              callback_group=self._group)
        self.set_mode_srv = node.create_client(SetMode, f"{self.ns}/set_mode",
                                               callback_group=self._group)
        # the stream has to be live before set_mode, so start it now and let it
        # publish nothing until a target exists
        node.create_timer(1.0 / SETPOINT_HZ, self._publish_setpoint,
                          callback_group=self._group)

    # -- state ---------------------------------------------------------------

    def _on_state(self, msg) -> None:
        self.state = msg

    def _on_fix(self, msg) -> None:
        self.fix = msg

    def log(self, message: str) -> None:
        self.node.get_logger().info(f"[{self.vehicle.name}] {message}")

    def _publish_setpoint(self) -> None:
        from mavros_msgs.msg import GlobalPositionTarget
        if self.target is None:
            return
        lat, lon, alt = self.target
        msg = GlobalPositionTarget()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
        msg.type_mask = (GlobalPositionTarget.IGNORE_VX | GlobalPositionTarget.IGNORE_VY
                         | GlobalPositionTarget.IGNORE_VZ
                         | GlobalPositionTarget.IGNORE_AFX
                         | GlobalPositionTarget.IGNORE_AFY
                         | GlobalPositionTarget.IGNORE_AFZ
                         | GlobalPositionTarget.IGNORE_YAW
                         | GlobalPositionTarget.IGNORE_YAW_RATE)
        msg.latitude, msg.longitude, msg.altitude = lat, lon, alt
        self.setpoint_pub.publish(msg)

    # -- blocking helpers ----------------------------------------------------

    def wait_connected(self, timeout_s: float = 30.0) -> bool:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if self.state is not None and self.state.connected:
                self.log("mavros connected to the autopilot")
                return True
            time.sleep(0.2)
        self.log(f"{self.ns}/state never reported connected; "
                 f"is mavros.launch.py running?")
        return False

    def _call(self, client, request, timeout_s: float = 10.0):
        if not client.wait_for_service(timeout_sec=timeout_s):
            self.log(f"service {client.srv_name} not available")
            return None
        future = client.call_async(request)
        deadline = time.time() + timeout_s
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)
        return future.result()

    def set_mode(self, mode: str, timeout_s: float = 15.0) -> bool:
        from mavros_msgs.srv import SetMode
        self.log(f"mode {mode}")
        self._call(self.set_mode_srv, SetMode.Request(custom_mode=mode))
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if self.state is not None and self.state.mode == mode:
                return True
            time.sleep(0.2)
        self.log(f"mode did not become {mode} (is a setpoint stream running?)")
        return False

    def arm(self, timeout_s: float = 20.0) -> bool:
        from mavros_msgs.srv import CommandBool
        self.log("arming")
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            result = self._call(self.arming, CommandBool.Request(value=True))
            if result is not None and result.success:
                return True
            time.sleep(1.0)        # pre-arm checks settle a few seconds after boot
        self.log("arming failed")
        return False

    def takeoff(self, alt_m: float, timeout_s: float = 60.0) -> bool:
        from mavros_msgs.srv import CommandTOL
        self.log(f"takeoff to {alt_m:.1f} m")
        result = self._call(self.takeoff_srv, CommandTOL.Request(altitude=alt_m))
        if result is None or not result.success:
            self.log("takeoff rejected")
            return False
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            # NavSatFix altitude is AMSL; home elevation is the forest's 580 m datum
            if self.fix is not None and self.fix.altitude > 0:
                if self.fix.altitude - self.home_elevation_m >= alt_m * 0.9:
                    self.log("airborne")
                    return True
            time.sleep(0.5)
        self.log("did not reach takeoff altitude")
        return False

    def goto(self, lat: float, lon: float, rel_alt_m: float) -> None:
        self.target = (lat, lon, rel_alt_m)

    def wait_arrival(self, lat: float, lon: float, rel_alt_m: float, *,
                     tol_m: float = 3.0, timeout_s: float = 180.0) -> bool:
        self.goto(lat, lon, rel_alt_m)
        deadline = time.time() + timeout_s
        hits = 0
        while time.time() < deadline:
            if self.fix is not None:
                flat = haversine_m(self.fix.latitude, self.fix.longitude, lat, lon)
                if flat <= tol_m:
                    hits += 1
                    if hits >= 2:         # two in a row: one straddling sample is noise
                        return True
                else:
                    hits = 0
            time.sleep(0.2)
        self.log(f"did not arrive within {timeout_s:.0f}s")
        return False

    def land(self) -> bool:
        return self.set_mode("LAND")


def fly(mav: MavrosVehicle, tasking: Tasking, *, dwell_s: float, tol_m: float,
        timeout_s: float, do_return: bool, stagger_s: float) -> bool:
    if stagger_s:
        time.sleep(stagger_s)
    if not mav.wait_connected():
        return False
    # a target must exist before set_mode, so aim at the first leg from the ground
    first = tasking.legs[0] if tasking.legs else None
    if first:
        mav.goto(first.lat, first.lon, tasking.cruise_alt_m)
    if not mav.set_mode("GUIDED") or not mav.arm():
        return False
    if not mav.takeoff(tasking.cruise_alt_m):
        return False

    for leg in tasking.legs:
        mav.log(f"-> {leg.roi.id} ({leg.roi.type}, aruco {leg.roi.marker.aruco_id})")
        if not mav.wait_arrival(leg.lat, leg.lon, tasking.cruise_alt_m, tol_m=tol_m,
                                timeout_s=timeout_s):
            return False
        mav.goto(leg.lat, leg.lon, leg.observe_alt_m)   # drop to a readable altitude
        mav.log(f"on station over {leg.roi.id}, holding {dwell_s:.0f}s")
        time.sleep(dwell_s)
        mav.goto(leg.lat, leg.lon, tasking.cruise_alt_m)

    if do_return:
        lat, lon = tasking.vehicle.spawn_geo
        mav.log("returning to the pad (an explicit leg -- home is the forest centre)")
        mav.wait_arrival(lat, lon, tasking.cruise_alt_m, tol_m=tol_m, timeout_s=timeout_s)
    mav.land()
    mav.log("done")
    return True


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--world", default="rescue")
    ap.add_argument("--strategy", default="nearest", choices=("nearest", "round_robin"))
    ap.add_argument("--cruise-alt", type=float, default=15.0)
    ap.add_argument("--alt-sep", type=float, default=3.0)
    ap.add_argument("--observe-alt", type=float, default=15.0)
    ap.add_argument("--dwell", type=float, default=5.0)
    ap.add_argument("--tolerance", type=float, default=3.0)
    ap.add_argument("--timeout", type=float, default=180.0)
    ap.add_argument("--rtl", action="store_true")
    ap.add_argument("--stagger", type=float, default=5.0)
    ap.add_argument("--dry-run", action="store_true", help="print the plan and stop")
    args = ap.parse_args(argv)

    run = load_run(args.world)
    fleet = ardupilot(run)
    skipped = px4(run)
    if skipped:
        print(f"WARNING: skipping {len(skipped)} PX4 vehicle(s) "
              f"({', '.join(v.name for v in skipped)}): this example is ArduPilot only. "
              f"PX4 here speaks DDS -- see endpoints.ros2_topics in the manifest.",
              file=sys.stderr)
    if not fleet:
        print(f"no ArduPilot vehicles in {run.manifest_path}", file=sys.stderr)
        return 1
    if not run.rois:
        print(f"no ROIs in {run.manifest_path}; set targets.rois.count in the scenario",
              file=sys.stderr)
        return 1

    taskings = assign(fleet, run.rois, strategy=args.strategy,
                      base_alt_m=args.cruise_alt, alt_separation_m=args.alt_sep,
                      observe_alt_m=args.observe_alt)
    print(f"manifest   {run.manifest_path}")
    print(describe(taskings))
    print()
    if args.dry_run:
        return 0

    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init()
    node = rclpy.create_node("uavsim_swarm_example")
    node.set_parameters([rclpy.parameter.Parameter(
        "use_sim_time", rclpy.Parameter.Type.BOOL, True)])

    mavs = [MavrosVehicle(node, t.vehicle, run.origin[2]) for t in taskings]

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    results: dict[str, bool] = {}
    threads = [threading.Thread(
        target=lambda m=m, t=t, i=i: results.__setitem__(
            t.vehicle.name,
            fly(m, t, dwell_s=args.dwell, tol_m=args.tolerance, timeout_s=args.timeout,
                do_return=args.rtl, stagger_s=args.stagger * i)),
        name=t.vehicle.name, daemon=True)
        for i, (m, t) in enumerate(zip(mavs, taskings))]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()

    for name in sorted(results):
        print(f"{name:<26} {'ok' if results[name] else 'FAILED'}")
    executor.shutdown()
    rclpy.shutdown()
    return 1 if any(not ok for ok in results.values()) else 0


if __name__ == "__main__":
    raise SystemExit(main())
