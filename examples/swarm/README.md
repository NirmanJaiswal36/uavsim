# Reference swarm clients

Two small controllers that fly the whole fleet to its assigned regions of interest —
one over raw MAVLink, one over MAVROS. They exist to show what consuming the run
manifest looks like, and to give you something that flies on the first afternoon.

**These are examples, not part of the environment.** Nothing under `uavsim/` or
`registry/` imports them, and deleting this directory changes nothing. Real mission
logic — search patterns, planning, detection, scoring — belongs to the framework you
are testing.

| file | what it is |
|---|---|
| `manifest_io.py` | reads `/tmp/uavsim/<world>/manifest.json` into dataclasses |
| `assign.py` | splits the ROIs across the fleet and picks deconflicted cruise layers |
| `pymavlink_swarm.py` | the controller, over raw MAVLink (no ROS needed) |
| `mavros_swarm.py` | the same mission, over MAVROS |

`manifest_io` and `assign` import neither pymavlink nor rclpy, so both controllers fly
the identical plan and `tests/test_examples.py` can check the interesting part in CI
with no simulator.

## Run it

```bash
./bin/uavsim up scenarios/forest_rescue.yaml

# see the plan without touching a vehicle
python3 examples/swarm/pymavlink_swarm.py --world rescue --dry-run

# fly it
python3 examples/swarm/pymavlink_swarm.py --world rescue \
        --cruise-alt 15 --alt-sep 3 --observe-alt 15 --dwell 5
```

For the MAVROS path, start MAVROS first — `sim.launch.py` does not:

```bash
ros2 launch uavsim_bringup mavros.launch.py world:=rescue     # second terminal
ros2 topic echo /ardupilot_iris_gimbal_0/mavros/state --once  # connected: true
python3 examples/swarm/mavros_swarm.py --world rescue --cruise-alt 15 --alt-sep 3
```

`pymavlink_swarm.py --repl` gives a small interactive shell (`connect`, `status`,
`arm`, `takeoff <sysid|all> <alt>`, `pos`, `mode`, `fly`) for poking one vehicle
without restarting the simulation.

## Four things that will bite you

**Home is the centre of the forest, not the pad.** Every ArduPilot SITL here is
started with `-l <world origin>` (`uavsim/adapter.py`), because `ArduPilotPlugin`
reports `WorldPose` — NED relative to the Gazebo world origin — and a per-vehicle home
would double-count the spawn offset and put the EKF tens of metres out. The world
origin is the forest centre. So `LOCAL_POSITION_NED` (0, 0) is the forest centre, a
vehicle on the apron reports itself ~426 m away from its own home, and **bare `RTL`
flies into the middle of the forest.** Both examples land in place, or fly an explicit
return leg with `--rtl`.

**ENU in MAVROS, NED in pymavlink.** `local_position/pose` is `x = east, y = north,
z = up` (the same axes as Gazebo); `LOCAL_POSITION_NED` is `x = north, y = east,
z = down`. The two scripts' "local x" are different axes. Both therefore command in
the *global* frame, consuming the manifest's lat/lon directly, which removes the
question entirely.

**MAVROS cannot share the pymavlink port.** `endpoints.mavlink` (14650 + 10·index) is a
single UDP socket; whichever process binds it first wins and the other gets silence.
`scenarios/forest_rescue.yaml` therefore asks for a dedicated output per vehicle:

```yaml
backends:
  ardupilot:
    extra_out: ["127.0.0.1:1475{index}"]
```

14750+ is clear of PX4's 14540–14599 band. `extra_out` only exists when MAVProxy does,
so a headless run (`supervisor: subprocess`) falls back to SITL's own TCP port — also
one client at a time.

**The markers are only readable low.** The plate is 2 m and the gimbal camera is
640×480 with a 2.0 rad FOV, so the marker spans 27 px at 15 m and falls under OpenCV's
~3 px-per-module floor by 25 m. That is why the cruise layers are 3 m apart rather than
10, and why each leg descends to `--observe-alt` and holds for `--dwell` seconds: at
~0.3 RTF the camera topic runs at roughly 3 Hz, so detection from a moving vehicle is
intermittent.

## PX4 is skipped

Both examples warn and skip PX4 vehicles. PX4 offboard needs an `OFFBOARD_CONTROL_MODE`
plus setpoint stream running above 2 Hz *before* the mode switch is accepted, uses
different arming and mode semantics, and in this repo is meant to be driven over DDS —
see `endpoints.ros2_topics` and `endpoints.dds_namespace` in the manifest
(`<vehicle>/fmu/in/trajectory_setpoint`). Mixing that into a MAVLink script would
double its size and halve its reliability.
