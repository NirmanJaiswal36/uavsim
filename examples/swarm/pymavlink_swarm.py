#!/usr/bin/env python3
"""Flies the whole fleet to its assigned regions of interest, over raw MAVLink.

A reference client, not part of the environment: nothing in ``uavsim/`` imports this,
and deleting ``examples/`` changes nothing.  It is here to show what consuming the run
manifest looks like, and to give you something that flies on the first afternoon.

    ./bin/uavsim up scenarios/forest_rescue.yaml
    python3 examples/swarm/pymavlink_swarm.py --world rescue --dry-run   # see the plan
    python3 examples/swarm/pymavlink_swarm.py --world rescue

THE FRAME TRAP.  Read this before debugging a position that looks wrong.

Every ArduPilot SITL in this environment is started with ``-l <world origin>``
(``uavsim/adapter.py``), because ``ArduPilotPlugin`` reports ``WorldPose`` -- NED
relative to the *Gazebo world origin* -- and giving each vehicle its own home would
double-count the spawn offset and put the EKF tens of metres out.

The Gazebo world origin is the **centre of the forest**, which is also the geographic
origin (CMAC, -35.389138 / 149.219188 / 580 m).

So for every vehicle here, home and ``LOCAL_POSITION_NED`` (0, 0) are the forest
centre, *not* the pad the vehicle is standing on.  Gazebo is ENU and MAVLink local is
NED, so::

    north = +y_enu      east = +x_enu      down = -z_enu

The fleet spawns on the apron around ``x_enu = -426``, which reads as
``LOCAL_POSITION_NED`` east = -426.  A freshly booted vehicle therefore reports itself
~426 m west of its own home, and **bare RTL flies it into the middle of the forest**.
This script lands in place instead, or flies an explicit return leg with ``--rtl``.

Because of all that, every waypoint here is commanded in the *global* frame with
``SET_POSITION_TARGET_GLOBAL_INT`` / ``MAV_FRAME_GLOBAL_RELATIVE_ALT_INT``: the
manifest already hands us each ROI's lat/lon, relative altitude is measured from home
at the forest's own elevation datum, and there is no axis convention left to get
wrong.  ``MAV_CMD_DO_REPOSITION`` is available behind ``--command reposition``.
``MAV_CMD_NAV_WAYPOINT`` is deliberately not used: it belongs to an uploaded mission
in AUTO, which would mean mission-upload plumbing for no benefit here.

PX4 vehicles are skipped with a warning.  PX4 offboard needs an
``OFFBOARD_CONTROL_MODE`` + setpoint stream running at over 2 Hz *before* the mode
switch is accepted, uses different arming and mode semantics, and in this repo is
meant to be driven over DDS -- see ``endpoints.ros2_topics`` in the manifest.
"""

from __future__ import annotations

import argparse
import math
import sys
import threading
import time
from dataclasses import dataclass

from assign import Tasking, assign, describe
from manifest_io import Vehicle, ardupilot, load_run, px4

EARTH_RADIUS_M = 6378137.0


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi, dlam = phi2 - phi1, math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(a))


@dataclass
class Link:
    """One vehicle's MAVLink connection.

    One ``Link`` per thread, always: a ``mavutil`` connection is not thread-safe, and
    sharing one across the fleet produces interleaved reads that look like packet loss.
    """

    vehicle: Vehicle
    timeout_s: float = 30.0
    verbose: bool = True
    conn: object | None = None

    # -- plumbing ------------------------------------------------------------

    def log(self, message: str) -> None:
        if self.verbose:
            print(f"[{self.vehicle.name}] {message}", flush=True)

    def connect(self) -> bool:
        from pymavlink import mavutil
        assert self.vehicle.mavlink
        self.log(f"connecting to {self.vehicle.mavlink}")
        self.conn = mavutil.mavlink_connection(self.vehicle.mavlink,
                                               source_system=250)
        return self.wait_heartbeat()

    def wait_heartbeat(self) -> bool:
        """Wait for a heartbeat carrying *this* vehicle's sysid.

        Worth being strict about: with MAVProxy outputs it is easy to end up pointed
        at the wrong instance, and a heartbeat from sysid 2 on vehicle 1's port is the
        only early warning you get.
        """
        deadline = time.time() + self.timeout_s
        while time.time() < deadline:
            msg = self.conn.recv_match(type="HEARTBEAT", blocking=True, timeout=2.0)
            if msg and msg.get_srcSystem() == self.vehicle.sysid:
                self.conn.target_system = msg.get_srcSystem()
                self.conn.target_component = 1
                self.log(f"heartbeat from sysid {self.vehicle.sysid}")
                return True
        self.log(f"no heartbeat from sysid {self.vehicle.sysid} in {self.timeout_s:.0f}s")
        return False

    def command_long(self, command: int, *params: float) -> None:
        values = list(params) + [0.0] * (7 - len(params))
        self.conn.mav.command_long_send(self.conn.target_system,
                                        self.conn.target_component,
                                        command, 0, *values[:7])

    def wait_ack(self, command: int, timeout_s: float = 5.0) -> bool:
        from pymavlink import mavutil
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            msg = self.conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=1.0)
            if msg and msg.command == command:
                ok = msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
                if not ok:
                    self.log(f"command {command} rejected: result {msg.result}")
                return ok
        self.log(f"command {command} not acknowledged")
        return False

    # -- state ---------------------------------------------------------------

    def global_position(self) -> tuple[float, float, float] | None:
        """``(lat, lon, relative_alt_m)`` -- relative to home, i.e. to the forest."""
        msg = self.conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2.0)
        if not msg:
            return None
        return msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000.0

    def local_ned(self) -> tuple[float, float, float] | None:
        """North, east, down -- with the origin at the forest centre. See the module docstring."""
        msg = self.conn.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=2.0)
        return (msg.x, msg.y, msg.z) if msg else None

    def mode(self) -> str:
        msg = self.conn.recv_match(type="HEARTBEAT", blocking=True, timeout=2.0)
        return self.conn.flightmode if msg else "?"

    # -- commands ------------------------------------------------------------

    def set_mode(self, name: str, timeout_s: float = 10.0) -> bool:
        self.log(f"mode {name}")
        self.conn.set_mode(name)
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            msg = self.conn.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
            if msg and self.conn.flightmode == name:
                return True
        self.log(f"mode did not become {name}")
        return False

    def arm(self, timeout_s: float = 20.0) -> bool:
        from pymavlink import mavutil
        self.log("arming")
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            self.command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
            if self.wait_ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 3.0):
                return True
            # a pre-arm check that has not passed yet is the usual reason; retrying is
            # the right response, since EKF and GPS settle a few seconds after boot
            time.sleep(1.0)
        return False

    def disarm(self) -> bool:
        from pymavlink import mavutil
        self.command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0)
        return self.wait_ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)

    def takeoff(self, alt_m: float, timeout_s: float = 60.0) -> bool:
        from pymavlink import mavutil
        self.log(f"takeoff to {alt_m:.1f} m")
        self.command_long(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, alt_m)
        if not self.wait_ack(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF):
            return False
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            position = self.global_position()
            if position and position[2] >= alt_m * 0.95:
                self.log(f"airborne at {position[2]:.1f} m")
                return True
        self.log("did not reach takeoff altitude")
        return False

    def goto(self, lat: float, lon: float, rel_alt_m: float, *,
             use_reposition: bool = False) -> None:
        """Send one position target. Cheap enough to re-send while waiting."""
        from pymavlink import mavutil
        if use_reposition:
            self.command_long(mavutil.mavlink.MAV_CMD_DO_REPOSITION,
                              -1, 0, 0, float("nan"), lat, lon, rel_alt_m)
            return
        self.conn.mav.set_position_target_global_int_send(
            0, self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            # position only: ignore velocity, acceleration, yaw and yaw rate
            0b0000111111111000,
            int(lat * 1e7), int(lon * 1e7), rel_alt_m,
            0, 0, 0, 0, 0, 0, 0, 0)

    def wait_arrival(self, lat: float, lon: float, rel_alt_m: float, *,
                     tol_m: float = 3.0, alt_tol_m: float = 2.0,
                     timeout_s: float = 180.0, use_reposition: bool = False) -> bool:
        """Arrive when we are close in lat/lon *and* altitude, twice in a row.

        Two consecutive hits rather than one, because a single GPS sample straddling
        the tolerance will otherwise call the leg finished mid-flight.  Deliberately
        not using ``NAV_CONTROLLER_OUTPUT.wp_dist``: it is not populated meaningfully
        for GUIDED position targets.
        """
        deadline = time.time() + timeout_s
        hits = 0
        last_send = 0.0
        while time.time() < deadline:
            if time.time() - last_send > 0.5:            # 2 Hz: survives a dropped packet
                self.goto(lat, lon, rel_alt_m, use_reposition=use_reposition)
                last_send = time.time()
            position = self.global_position()
            if not position:
                continue
            flat = haversine_m(position[0], position[1], lat, lon)
            if flat <= tol_m and abs(position[2] - rel_alt_m) <= alt_tol_m:
                hits += 1
                if hits >= 2:
                    return True
            else:
                hits = 0
        self.log(f"did not arrive within {timeout_s:.0f}s")
        return False

    def land(self) -> bool:
        from pymavlink import mavutil
        self.log("landing")
        self.command_long(mavutil.mavlink.MAV_CMD_NAV_LAND)
        return self.wait_ack(mavutil.mavlink.MAV_CMD_NAV_LAND)


def fly(tasking: Tasking, *, dwell_s: float, tol_m: float, timeout_s: float,
        use_reposition: bool, do_return: bool, stagger_s: float) -> bool:
    """One vehicle's whole mission. Runs on its own thread."""
    link = Link(tasking.vehicle)
    if stagger_s:
        # departures are staggered so six vehicles do not climb through each other's
        # layer at the same moment
        time.sleep(stagger_s)
    if not link.connect():
        return False
    if not link.set_mode("GUIDED") or not link.arm():
        return False
    if not link.takeoff(tasking.cruise_alt_m):
        return False

    for leg in tasking.legs:
        link.log(f"-> {leg.roi.id} ({leg.roi.type}, aruco {leg.roi.marker.aruco_id})")
        if not link.wait_arrival(leg.lat, leg.lon, tasking.cruise_alt_m, tol_m=tol_m,
                                 timeout_s=timeout_s, use_reposition=use_reposition):
            return False
        # drop to the common observation altitude: at cruise the plate may be too few
        # pixels across to detect, and the layers exist only for transit
        if not link.wait_arrival(leg.lat, leg.lon, leg.observe_alt_m, tol_m=tol_m,
                                 timeout_s=timeout_s, use_reposition=use_reposition):
            return False
        link.log(f"on station over {leg.roi.id}, holding {dwell_s:.0f}s")
        time.sleep(dwell_s)
        link.wait_arrival(leg.lat, leg.lon, tasking.cruise_alt_m, tol_m=tol_m,
                          timeout_s=timeout_s, use_reposition=use_reposition)

    if do_return:
        lat, lon = tasking.vehicle.spawn_geo
        link.log("returning to the pad (an explicit leg -- RTL would fly to the forest)")
        link.wait_arrival(lat, lon, tasking.cruise_alt_m, tol_m=tol_m,
                          timeout_s=timeout_s, use_reposition=use_reposition)
    link.land()
    link.log("done")
    return True


def repl(taskings: list[Tasking]) -> int:
    """A small interactive shell. Useful when a mission misbehaves and you want to
    poke one vehicle without restarting the simulation."""
    links = {t.vehicle.sysid: Link(t.vehicle) for t in taskings}
    connected: set[int] = set()
    by_sysid = {t.vehicle.sysid: t for t in taskings}

    def targets(token: str) -> list[int]:
        return sorted(links) if token in ("all", "") else [int(token)]

    print("commands: connect | status | arm | disarm | takeoff <sysid|all> <alt> | "
          "pos | mode <sysid|all> <name> | fly | quit")
    while True:
        try:
            parts = input("swarm> ").split()
        except EOFError:
            return 0
        if not parts:
            continue
        cmd, args = parts[0], parts[1:]
        if cmd in ("quit", "exit"):
            return 0
        if cmd == "connect":
            for sysid, link in links.items():
                if link.connect():
                    connected.add(sysid)
        elif cmd == "status":
            for sysid in sorted(connected):
                print(f"  {sysid}: mode {links[sysid].mode()} "
                      f"pos {links[sysid].global_position()}")
        elif cmd in ("arm", "disarm"):
            for sysid in connected:
                getattr(links[sysid], cmd)()
        elif cmd == "takeoff" and len(args) == 2:
            for sysid in targets(args[0]):
                links[sysid].takeoff(float(args[1]))
        elif cmd == "pos":
            for sysid in sorted(connected):
                print(f"  {sysid}: global {links[sysid].global_position()} "
                      f"local_ned {links[sysid].local_ned()}")
        elif cmd == "mode" and len(args) == 2:
            for sysid in targets(args[0]):
                links[sysid].set_mode(args[1].upper())
        elif cmd == "fly":
            for sysid in sorted(connected):
                fly(by_sysid[sysid], dwell_s=5.0, tol_m=3.0, timeout_s=180.0,
                    use_reposition=False, do_return=False, stagger_s=0.0)
        else:
            print("  ?")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--world", default="rescue",
                    help="world name, run directory, or path to a manifest.json")
    ap.add_argument("--strategy", default="nearest", choices=("nearest", "round_robin"))
    ap.add_argument("--cruise-alt", type=float, default=15.0)
    ap.add_argument("--alt-sep", type=float, default=3.0,
                    help="metres between per-vehicle cruise layers")
    ap.add_argument("--observe-alt", type=float, default=15.0,
                    help="altitude held over an ROI; above ~20 m the marker is too "
                         "few pixels across to detect")
    ap.add_argument("--dwell", type=float, default=5.0,
                    help="seconds on station, so an intermittent camera gets a frame")
    ap.add_argument("--tolerance", type=float, default=3.0)
    ap.add_argument("--timeout", type=float, default=180.0, help="per leg")
    ap.add_argument("--command", default="setpoint", choices=("setpoint", "reposition"))
    ap.add_argument("--rtl", action="store_true",
                    help="fly an explicit leg back to the pad before landing")
    ap.add_argument("--stagger", type=float, default=5.0,
                    help="seconds between vehicle departures")
    ap.add_argument("--dry-run", action="store_true", help="print the plan and stop")
    ap.add_argument("--repl", action="store_true")
    args = ap.parse_args(argv)

    run = load_run(args.world)
    fleet = ardupilot(run)
    skipped = px4(run)
    if skipped:
        print(f"WARNING: skipping {len(skipped)} PX4 vehicle(s) "
              f"({', '.join(v.name for v in skipped)}): this example is ArduPilot only. "
              f"PX4 needs an offboard setpoint stream above 2 Hz before OFFBOARD is "
              f"accepted; drive it over px4_msgs on "
              f"{skipped[0].name}/fmu/in/trajectory_setpoint instead.", file=sys.stderr)
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
    print(f"world      {run.world}  (origin = forest centre, "
          f"{run.origin[0]:.6f}, {run.origin[1]:.6f})")
    print(describe(taskings))
    print()
    if args.dry_run:
        return 0
    if args.repl:
        return repl(taskings)

    results: dict[str, bool] = {}
    threads = [threading.Thread(
        target=lambda t=t, i=i: results.__setitem__(
            t.vehicle.name,
            fly(t, dwell_s=args.dwell, tol_m=args.tolerance, timeout_s=args.timeout,
                use_reposition=args.command == "reposition", do_return=args.rtl,
                stagger_s=args.stagger * i)),
        name=t.vehicle.name, daemon=True) for i, t in enumerate(taskings)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()

    failed = [name for name, ok in results.items() if not ok]
    for name in sorted(results):
        print(f"{name:<26} {'ok' if results[name] else 'FAILED'}")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
