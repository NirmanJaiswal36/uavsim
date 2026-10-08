# Architecture

How a scenario file becomes a running world full of aircraft.

```
scenario.yaml ─► Orchestrator ─┬─ WorldBuilder   base world + ROI + GCS + targets
                               ├─ Registry       airframes / backends / payloads / vehicles
                               ├─ Allocator      names, sysids, instance numbers, pads
                               ├─ SdfRenderer    airframe + binding + payloads -> SDF string
                               ├─ Spawner        gz EntityFactory into the running world
                               ├─ Adapter        per-backend processes, readiness, endpoints
                               ├─ Bridge         ros_gz_bridge config from capabilities
                               └─ Manifest       the JSON handed back to you
```

The world file is generated **without vehicles in it**. Aircraft are spawned into the
running simulation afterwards, which is why `uavsim spawn` can add more to a live run and
why no `models/drone_N` directories need to exist on disk.

## The four-part vehicle model

A vehicle type is a named combination of four independent things. Keeping them apart is
what makes "add one YAML file" enough to define a new aircraft.

| part | lives in | describes |
|---|---|---|
| **Airframe** | `registry/airframes/<id>/airframe.yaml` | The physical aircraft: model URI, base link, IMU sensor, footprint, flight envelope, mount points |
| **Binding** | `registry/airframes/<id>/bindings/<backend>.sdf.j2` | The glue between one airframe and one autopilot — joint names *and* which plugin. Airframe-specific and backend-specific, so it lives with the airframe and is named after the backend |
| **Backend** | `registry/backends/<id>/` | The autopilot: `backend.yaml` plus an `adapter.py` |
| **Payload** | `registry/payloads/<id>/` | A mountable sensor or actuator: an SDF fragment, an optional per-backend fragment, and the topics it `provides` |

`registry/vehicles/<id>.yaml` names a combination. Everything is schema-validated against
`registry/schemas/*.json` at load time, so a typo is an error message rather than a
mysterious Gazebo failure.

## Modules

| module | purpose |
|---|---|
| `spec.py` | Frozen dataclasses for registry data, plus the merged `ResolvedVehicle` and its `capabilities()` normaliser |
| `registry.py` | Loads and validates everything under `registry/`, merges a vehicle type, imports backend adapters |
| `adapter.py` | The backend contract: `Pose`, `WorldInfo`, `Instance`, `ProcessSpec`, `SpawnRequest`, `AutopilotAdapter` |
| `site.py` | Pure geometry of the GCS — pad grid, runway slots, headings, and the derived standoff that leaves 200 m of open ground between the runway end and the forest boundary. Single source of truth shared by world rendering and allocation |
| `allocator.py` | Turns fleet requests into concrete instances: unique names, sysids, per-backend indices, pad assignment, singleton-payload enforcement |
| `world.py` | Builds the vehicle-free world from Jinja templates; casualty **and ROI** placement, ENU↔geo conversion, terrain clipping |
| `sdf.py` | Renders one instance's model SDF in memory and validates it as XML |
| `spawner.py` | Creates and removes entities in the running world over gz-transport |
| `supervisor.py` | Runs `ProcessSpec`s under tmux or detached subprocesses; starts Gazebo; stops things by exact process name |
| `bridge.py` | Generates the `ros_gz_bridge` config from payload `provides` declarations |
| `manifest.py` | Writes `manifest.json` |
| `scenario.py` | Loads and validates a scenario YAML |
| `orchestrator.py` | The conductor: `plan()`, `up()`, `add()`, `down()` |
| `verify.py` | Static and live health checks |
| `cli.py` | The argparse front end |

Two directories sit outside the package on purpose:

| directory | purpose |
|---|---|
| `tools/` | Authoring scripts that generate committed data (`gen_roi_forest.py`, `gen_aruco_textures.py`). Kept off the runtime import path, which `ros2/uavsim_bringup/launch/sim.launch.py` depends on staying light — and `gen_aruco_textures.py` needs OpenCV, which must never become a runtime dependency |
| `examples/` | Reference clients that only *consume* the manifest. Nothing under `uavsim/` or `registry/` may import them |

## Capability schema

Resolved per vehicle and written to the manifest. The bridge generator reads only this —
never SDF — which is why adding a payload automatically produces ROS 2 topics.

```yaml
platform:  {class, hover, vtol, launch, cruise_mps, max_mps, endurance_min, mass_kg, footprint_radius_m}
autopilot: {backend, protocols, ros2_native, offboard, sim_time}
sensors:   [{id, kind: camera.rgb|camera.depth|imu|..., rate_hz, gz_topic, ros: {topic, type, direction}}]
actuators: [{id, kind: gimbal.3axis|..., axes, gz_topic, ros: {...}}]
```

## Adapter interface

`uavsim/adapter.py`. An adapter **describes** work; it never runs it. The supervisor and
spawner do that. This is what keeps process management in one place and makes a new
autopilot a self-contained directory.

```python
preflight()                -> list[str]          # missing binaries, unbuilt SITL
validate(vehicle)          -> list[str]          # this type cannot run on this backend
sdf_vars(inst)             -> dict               # ports/topics for the binding template
spawn_request(inst, sdf)   -> SpawnRequest|None  # or None if the backend self-spawns
shared_processes(insts)    -> [ProcessSpec]      # e.g. one uXRCE-DDS agent
processes(inst)            -> [ProcessSpec]      # this vehicle's autopilot
wait_ready(inst, timeout)  -> bool
stop_hints(inst)           -> [str]              # exact names for pkill -x
endpoints(inst)            -> dict               # into the manifest
```

## Port allocation

ArduPilot instance *i* gets FDM `9002+10i` and SITL TCP `5760+10i`, matching
`AP_HAL_SITL`'s own arithmetic, plus a MAVProxy output on `14650+10i`. PX4 instance *i*
gets GCS `18570+i` and offboard `14540+i`, with a single shared `MicroXRCEAgent` on
port 8888.

The ArduPilot MAVProxy base is `14650` rather than the conventional `14550` on purpose.
PX4 SITL binds `14580+instance` for its own onboard MAVLink link, so a `14550` base with
a stride of 10 puts the **4th** ArduPilot vehicle on `14580`, directly on top of the
**1st** PX4 vehicle. The collision is silent — PX4 wins the bind and the ArduPilot
vehicle simply never reaches a GCS — and a 4 × ArduPilot + 2 × PX4 fleet is exactly the
shipped reference scenario. `tests/test_registry.py` asserts that the ArduPilot GCS
ports never enter PX4's `14540-14599` band.

The allocator guarantees uniqueness across a mixed fleet, and `uavsim verify --live`
checks that each FDM port has exactly one owner.

`14650+10i` is a single UDP socket, so exactly one process can bind it. That is fine
until you want MAVROS *and* something else on the same vehicle, at which point the loser
gets silence rather than an error. The answer is `backends.ardupilot.extra_out`, which
adds further MAVProxy outputs and surfaces them in `endpoints.extra_out`:

```yaml
backends:
  ardupilot:
    extra_out: ["127.0.0.1:1475{index}"]
```

`14750+` is chosen for the same reason `14650` is: it is clear of PX4's `14540-14599`
band. `extra_out` only exists when MAVProxy does, so a headless run falls back to SITL's
own TCP port — also one client at a time.

## Ground truth

Two places, split by what the thing *is*:

| where | what | why |
|---|---|---|
| `worlds/fragments/roi_forest.yaml` | trunk positions, fire positions, mission and scenery radii, ground extent | static properties of a *place*; the same for every run in that ROI |
| `manifest.json` (`targets.*`) | casualties, ROIs, ArUco markers — in ENU and WGS84 | per-run, seeded, chosen by the scenario |

`SCHEMA_VERSION` is **2**: version 2 added `targets.rois[]`, each with an `index`, a
`type`, a pose, and a `marker` block carrying the ArUco dictionary, id, size and its own
position. `aruco_id` is reported separately from `index` because a detector reports the
marker it read, and `targets.rois.first_marker_id` lets two scenarios in one world use
disjoint marker sets.

## Generated world data

`worlds/fragments/roi_forest.{sdf,yaml}` and `models/aruco_marker_*/` are generated by
`tools/gen_roi_forest.py` and `tools/gen_aruco_textures.py`, and the output is
**committed**. That is deliberate, and it is the opposite of the `build/` rule above:

- `base_world.sdf.j2` includes the fragment through a Jinja loader rooted at
  `worlds/fragments/`, so generating at build time would put shipped data in gitignored
  `build/` and give every `uavsim world` a dependency on the generator.
- A committed fragment gives a **reviewable diff** when the forest changes — which
  matters, because it changes what every camera in every scenario sees.
- The ArUco PNGs being committed is what lets `uavsim world` run with no OpenCV
  installed.

Reproducibility comes from `--check` instead, which regenerates in memory and
byte-compares; `tests/test_world.py` and `tests/test_registry.py` run it. The exact
regeneration command is recorded in `roi_forest.yaml` under `generator_args`.

The generated fragment nests its grass, trees and boundary posts inside one static
`forest_scenery` model. That is not cosmetic: Gazebo's entity tree lists entities in
creation order and vehicles are created last, so flat scenery puts the fleet at the
bottom of a 591-row list and makes it look like nothing spawned. The axis marker
stays at world level because it carries the world's only navsat sensor, and nesting
it would move that sensor's topic path out from under the PX4 backend.

## Run directory

`$UAVSIM_RUN_DIR/<world>/` (default `/tmp/uavsim/<world>/`):

```
manifest.json          the contract
bridge.yaml            generated ros_gz_bridge config
logs/                  per-process logs (supervisor: subprocess)
<vehicle_name>/        SITL's --use-dir working directory
```

Generated world SDF files go to `build/worlds/<world>.sdf`, alongside the plugin build
output. Both are gitignored — the repository ships templates and data, not artefacts.

## Why the world has exactly one plane collision

A `<plane>` collision shape in Gazebo is an infinite half-space in DART. Two of them make
the ground-contact LCP singular, and the physics step freezes — with no error message.
So the world has exactly one `ground_plane`, and the apron, runway and grass patches are
boxes or visual-only geometry. `uavsim verify` and `tests/test_world.py` both enforce it,
and `tests/test_registry.py` checks that no vehicle model introduces one either.
