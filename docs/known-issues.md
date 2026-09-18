# Known issues

What does not work yet, and what is known-good. Nothing here is hidden in a footnote —
if it is broken, it is listed.

## `ardupilot_zephyr` does not take off

**Status:** integrates correctly, does not fly.

What works: the vehicle spawns on the runway, ArduPlane SITL connects over lock-step,
it arms, the elevons respond correctly to attitude demands, and
`uavsim verify --live` passes 10/10.

What does not: it cannot reach rotation speed. At 100 % throttle it accelerates to about
**7.7 m/s** and stays there, against a cruise speed of 18 m/s. It then skids sideways off
the runway rather than tracking the centreline. Starting it airborne does not help, so
this is not purely a ground-handling problem.

Why: the Zephyr airframe has no landing gear and no nosewheel steering — it rests on its
wing — and the propeller lift-drag configuration in
`registry/airframes/zephyr/bindings/ardupilot.sdf.j2` does not produce enough thrust to
overcome ground friction. Fixing it means tuning the lift-drag coefficients, the
propeller thrust and the ground-contact model.

If you need a fixed-wing vehicle today, `px4_advanced_plane` and `px4_rc_cessna` use
PX4's own aerodynamic models rather than this binding, though neither has been
flight-tested here either.

### Fixed along the way

Worth knowing if you write your own ArduPilot vehicle type: uavsim starts SITL with
`--model JSON`, and the SITL binary resolves its per-frame default parameters from the
`--model` value rather than the `-f` frame name. ArduPilot's own
`Tools/autotest/default_params/gazebo-zephyr.parm` was therefore **never applied**, so
the elevons were unmixed and the wing had no chance. The parameters now ship as
`config/gazebo-zephyr.parm` and are referenced from the vehicle type. Any airframe that
needs motor mixing, elevon setup or mount configuration must list its own param file in
`backend_config.param_files` — see [`extending.md`](extending.md).

## Vehicle types that have not been flight-tested

`ardupilot_iris`, `px4_x500_depth`, `px4_standard_vtol`, `px4_advanced_plane`,
`px4_rc_cessna` and `static_iris` are shipped and covered by the test suite — their
definitions validate, allocate and render to well-formed SDF — but no one has confirmed
them in the air. They are a starting point, not a guarantee.

Flight-tested: `ardupilot_iris_gimbal` and `px4_x500`, as a mixed 4 + 2 fleet.

## The ROS 2 launch file needs `UAVSIM_ROOT` when installed

`ros2/uavsim_bringup/launch/sim.launch.py` finds the uavsim checkout by walking up from
its own location. That works from the source tree, but colcon installs it to
`share/uavsim_bringup/launch/`, where there is nothing to find. Set `UAVSIM_ROOT` to the
repository root in that case; the launch file raises an error naming the variable rather
than failing obscurely.

## Low real-time factor

A 4 + 2 fleet with cameras runs at roughly **0.3 RTF** on a normal workstation, and
camera topics publish at about 3 Hz rather than their nominal rate. Under lock-step
ArduPilot follows simulation time, so flight behaviour stays correct — it just takes
longer in wall-clock terms. Expect this to get worse with more vehicles and more cameras.

## Things that behave correctly but surprise people

These are not bugs, but they cost time the first time:

- **Never start Gazebo paused.** ArduPilot's lock-step handshake deadlocks. `uavsim`
  always passes `-r`.
- **One stalled SITL wedges everything**, including Gazebo, because lock-step has no
  timeout recovery. `uavsim verify --live` checks that simulation time is advancing.
- **A leftover Gazebo server does not fail loudly.** Two servers can both bind the FDM
  ports via `SO_REUSEADDR` and silently split the stream. If a run behaves strangely,
  check for stale processes first — `uavsim down --gazebo`, then `uavsim verify --live`.
- **Exactly one `<plane>` collision per world.** A second one makes the ground-contact
  LCP singular in DART and freezes the physics step, with no error message.
- **The first run needs network access** to fetch tree models from Gazebo Fuel.
