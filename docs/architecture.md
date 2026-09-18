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
| `site.py` | Pure geometry of the GCS — pad grid, runway slots, headings. Single source of truth shared by world rendering and allocation |
| `allocator.py` | Turns fleet requests into concrete instances: unique names, sysids, per-backend indices, pad assignment, singleton-payload enforcement |
| `world.py` | Builds the vehicle-free world from Jinja templates; casualty placement, ENU↔geo conversion, terrain clipping |
| `sdf.py` | Renders one instance's model SDF in memory and validates it as XML |
| `spawner.py` | Creates and removes entities in the running world over gz-transport |
| `supervisor.py` | Runs `ProcessSpec`s under tmux or detached subprocesses; starts Gazebo; stops things by exact process name |
| `bridge.py` | Generates the `ros_gz_bridge` config from payload `provides` declarations |
| `manifest.py` | Writes `manifest.json` |
| `scenario.py` | Loads and validates a scenario YAML |
| `orchestrator.py` | The conductor: `plan()`, `up()`, `add()`, `down()` |
| `verify.py` | Static and live health checks |
| `cli.py` | The argparse front end |

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
