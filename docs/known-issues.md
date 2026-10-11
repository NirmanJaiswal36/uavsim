# Known issues

What does not work yet, and what is known-good. Nothing here is hidden in a footnote —
if it is broken, it is listed.

## ArduPilot per-frame parameters are never applied automatically

Worth knowing if you write your own ArduPilot vehicle type: uavsim starts SITL with
`--model JSON`, and the SITL binary resolves its per-frame default parameters from the
`--model` value rather than the `-f` frame name. ArduPilot's own
`Tools/autotest/default_params/<frame>.parm` is therefore **never applied**. Any
airframe that needs motor mixing or mount configuration must list its own param file in
`backend_config.param_files` (as `ardupilot_iris_gimbal` does with
`config/gazebo-iris-gimbal.parm`) — see [`extending.md`](extending.md).

## Vehicle types that have not been flight-tested

`ardupilot_iris`, `px4_x500_depth` and `static_iris` are shipped and covered by the
test suite — their
definitions validate, allocate and render to well-formed SDF — but no one has confirmed
them in the air. They are a starting point, not a guarantee.

Flight-tested: `ardupilot_iris_gimbal` and `px4_x500`, as a mixed 4 + 2 fleet.

## The ROS 2 launch file needs `UAVSIM_ROOT` when installed

`ros2/uavsim_bringup/launch/sim.launch.py` finds the uavsim checkout by walking up from
its own location. That works from the source tree, but colcon installs it to
`share/uavsim_bringup/launch/`, where there is nothing to find. Set `UAVSIM_ROOT` to the
repository root in that case; the launch file raises an error naming the variable rather
than failing obscurely.

## `uavsim down` can report PX4 as still alive

`down` sometimes prints `WARNING: N px4 still alive`. PX4 SITL exits a second or
two after its DDS agent and Gazebo go, so the check runs slightly too early. Re-check with
`pgrep -x px4` before doing anything about it; in practice the processes are gone.

## Low real-time factor

A 4 + 2 fleet with cameras runs at roughly **0.3 RTF** on a normal workstation, and
camera topics publish at about 3 Hz rather than their nominal rate. Under lock-step
ArduPilot follows simulation time, so flight behaviour stays correct — it just takes
longer in wall-clock terms. Expect this to get worse with more vehicles and more cameras.

The forest's 726 textured trees, and shadow casting over them, are the biggest renderer
cost; the 8 ROI props are primitives and cost comparatively little. The forest is kept
at 500 m × 500 m for this reason: a 1000 m square at the same tree density measured
0.43 RTF against 1.00 for the 500 m one, with four gimbal cameras rendering and no
vehicles (`tools/gen_roi_forest.py` has the table). If you need a faster world for
development or CI, regenerate a smaller one rather than deleting entities by hand:

```bash
python3 tools/gen_roi_forest.py --size 300 --oak 157 --pine 104 --fires 20
```

(and move `DEFAULT_GCS_POSE` in `uavsim/site.py` and the scenarios' `gcs_pose` with it, or
the GCS ends up further than 100 m from the forest edge).

## The first run downloads the tree models

The oak and pine models come from Gazebo Fuel. Gazebo gives no progress indication while
it fetches them, and `Spawner.wait_for_world()` can expire *before* the world has
finished loading — which surfaces as a misleading "world never came up". Warm the cache
once, with network, before timing or scripting anything:

```bash
./bin/uavsim up scenarios/forest_static.yaml      # then uavsim down
```

## Vehicles missing from the GUI but flying fine

If a vehicle is created while the GUI is still building its 3D scene, the GUI never
draws it. The model exists in the server, its autopilot gets GPS lock and it will
fly, but there is nothing on the pad, and `/gui/move_to` logs `Target '<name>' not
found`. The server is up long before the GUI (render engine plus thousands of
scenery visuals), so spawning as soon as the server answers loses the race. How many
vehicles go missing depends on timing, so it comes and goes. PX4 vehicles usually
escape it because PX4 SITL creates its own model later, when its process starts.

`uavsim up` therefore waits for the GUI to advertise `/gui/move_to`, which happens
once its scene exists, before it spawns anything or starts any autopilot. It
warns if the GUI never gets there. Headless runs and `gazebo.start: false` do not
wait. If you start the server yourself, open the GUI before running `uavsim up`.

To check a run, ask the GUI for a vehicle by name. The camera should jump to it:

```bash
gz service -s /gui/move_to --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean \
  --timeout 5000 --req 'data: "ardupilot_iris_gimbal_0"'
```

The default overview camera frames all 24 pads and the forest edge, which puts it
~90 m from the pads, where a 0.5 m quadrotor is a couple of pixels. "Move to", as
below, is how you get a look at one.

## Finding your own vehicles in the entity tree

Gazebo's entity tree lists everything in creation order, and the vehicles are
created **last** -- they are not in the world file at all, the spawner adds them at
runtime over the `create` service. When the old forest's ~565 scenery entities were
flat in the world, the six drones landed at rows 585-590 of 591, under a wall of
near-identical `grasspatch_*` and `Oak_Tree_*` rows. That reads exactly like "the
drones are not spawning", and it cost real debugging time.

The grass, trees and boundary posts therefore live on one static link inside a
`forest_scenery` model, so the top level of the tree is ~27 rows and the vehicles are
visible without scrolling. Individual trees are visuals and collisions of that link
(`Oak_Tree_1_branch`, `Oak_Tree_1_collision`, ...), not models, so they cannot be
selected or moved one by one; nothing in the environment refers to them by name.

To jump the camera to anything, by name:

```bash
gz service -s /gui/move_to --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean \
  --timeout 5000 --req 'data: "ardupilot_iris_gimbal_0"'
```

or right-click the model in the Entity Tree and choose "Move to".

## ArUco markers are not detectable much above 20 m

The plate is 2 m and the gimbal camera is 640×480 with a 2.0 rad horizontal FOV, so the
marker is this many pixels per module (a 4×4 marker is 6 modules across including its
black border, and OpenCV needs roughly 3):

| altitude | GSD | plate | px per module |
|---|---|---|---|
| 10 m | 4.9 cm | 41 px | 6.9 |
| 15 m | 7.3 cm | 27 px | 4.6 |
| 20 m | 9.7 cm | 21 px | 3.4 — marginal |
| 25 m | 12.2 cm | 16 px | 2.7 — below the floor |

So survey at **15 m**. The example controllers cruise in per-vehicle layers 3 m apart and
descend to `--observe-alt` on station for exactly this reason; above 25 m you need
`CameraZoomPlugin` (`max_zoom 125`) or a bigger plate (`targets.rois.marker_size_m`).

Separately, at ~3 Hz effective camera rate and 0.3 RTF a drone at 5 m/s covers ~1.7 m per
frame, so detection from a *moving* vehicle is intermittent. Hold station (`--dwell`).

## `RTL` flies into the forest, not back to the pad

By design, not a bug. Every ArduPilot SITL is started with `-l <world origin>`
(`uavsim/adapter.py`) because `ArduPilotPlugin` reports `WorldPose` — NED relative to the
Gazebo world origin — and a per-vehicle home would double-count the spawn offset and put
the EKF tens of metres out. That origin is the **centre of the ROI**.

So home and `LOCAL_POSITION_NED` (0, 0) are in the trees, a vehicle on the apron reports
itself ~356 m from its own home, and `RTL` does exactly what it says. Land in place, or
fly an explicit leg back to `spawn.geo` from the manifest. Both example controllers do
the latter under `--rtl`.

## MAVROS and the pymavlink example cannot share `14650+10i`

`endpoints.mavlink` is a single UDP socket, so whichever process binds it first wins and
the other gets silence rather than an error. Give MAVROS its own output:

```yaml
backends:
  ardupilot:
    extra_out: ["127.0.0.1:1475{index}"]
```

`scenarios/forest_rescue.yaml` already does. Note that `extra_out` only exists when
MAVProxy does (`supervisor: tmux`); a headless run falls back to SITL's own TCP port,
which is also one client at a time.

## `examples/swarm/*` skips PX4 vehicles

Both reference controllers warn and fly only the ArduPilot half of the fleet. PX4
offboard needs an `OFFBOARD_CONTROL_MODE` plus setpoint stream above 2 Hz *before* the
mode switch is accepted, uses different arming and mode semantics, and in this repo is
meant to be driven over DDS — see `endpoints.ros2_topics` and `endpoints.dds_namespace`
in the manifest (`<vehicle>/fmu/in/trajectory_setpoint`).

## Things that behave correctly but surprise people

These are not bugs, but they cost time the first time:

- **Never start Gazebo paused.** ArduPilot's lock-step handshake deadlocks. `uavsim`
  always passes `-r`.
- **One stalled SITL wedges everything**, including Gazebo, because lock-step has no
  timeout recovery. `uavsim verify --live` checks that simulation time is advancing.
- **A leftover Gazebo server does not fail loudly.** Two servers can both bind the FDM
  ports via `SO_REUSEADDR` and silently split the stream. If a run behaves strangely,
  check for stale processes first — `uavsim down`, then `uavsim verify --live`.
- **Exactly one `<plane>` collision per world.** A second one makes the ground-contact
  LCP singular in DART and freezes the physics step, with no error message.
- **The first run needs network access** to fetch the tree models from Gazebo Fuel.
  See above.
