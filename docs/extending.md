# Extending uavsim

Four kinds of thing you can add, in increasing order of effort.

## A vehicle type — one YAML file, no code

A vehicle type is just a named combination of an airframe and a backend, optionally with
payloads and configuration.

```yaml
# registry/vehicles/px4_x500_lidar.yaml
id: px4_x500_lidar
airframe: x500
backend: px4
binding_vars: {px4_model: x500_lidar_2d, sys_autostart: 4013}
```

The file stem must equal the `id`. It will show up in `uavsim list vehicles` immediately.

With a payload and autopilot parameters:

```yaml
# registry/vehicles/ardupilot_iris_gimbal.yaml
id: ardupilot_iris_gimbal
description: ArduCopter Iris with a 3-axis gimbal camera.
airframe: iris
backend: ardupilot

payloads:
  - type: gimbal_camera_3axis
    id: gimbal_cam
    mount: belly

backend_config:
  param_files: [config/gazebo-iris-gimbal.parm]
```

> **ArduPilot parameter files matter.** uavsim starts SITL with `--model JSON`, and the
> SITL binary resolves its per-frame defaults from the `--model` value, *not* from the
> `-f` frame name. ArduPilot's own `default_params/<frame>.parm` is therefore never
> applied. Anything the airframe needs — motor mixing, elevon setup, mount configuration
> — has to be listed in `backend_config.param_files`.

## A payload

```
registry/payloads/<id>/
├── payload.yaml                  what it is and what it provides
├── fragment.sdf.j2               the SDF to splice into the model
└── backends/<backend>.sdf.j2     optional: e.g. ArduPilot <control> blocks
```

Declare what it `provides` — sensors and actuators with their kinds, rates and gz topics
— and the bridge generator and manifest pick it up automatically. You do not touch
`bridge.py`.

Set `singleton: true` if the payload uses a global topic and so can only appear once per
run (as `px4_oakd_lite` does for its depth camera). The allocator enforces it.

## An airframe

```
registry/airframes/<id>/
├── airframe.yaml
└── bindings/<backend>.sdf.j2     one per autopilot it supports
```

`airframe.yaml` describes the physical aircraft:

```yaml
id: iris
class: multirotor          # the only class: this environment flies quadcopters
model_uri: model://iris_with_standoffs
base_link: iris_with_standoffs::base_link
imu_sensor: iris_with_standoffs::imu_link::imu_sensor

geometry:
  footprint_radius_m: 0.35
  spawn_z_m: 0.195

performance:
  mass_kg: 1.5
  cruise_mps: 10.0
  max_mps: 15.0
  endurance_min: 18.0
  max_payload_kg: 0.4

bindings:
  ardupilot:
    overlay: bindings/ardupilot.sdf.j2
    vars: {vehicle: ArduCopter, frame: gazebo-iris}
```

Every vehicle takes off from a landing pad at the GCS; there is no runway, and the
schema rejects any `class` other than `multirotor`. `footprint_radius_m` is checked
against the scenario's pad spacing, so an aircraft that would overlap its neighbour is
an error rather than a collision at spawn.

The binding template is where the aircraft meets the autopilot: the `ArduPilotPlugin` or
PX4 plugin block, the lift-drag elements, and the `<control>` channel mapping from servo
outputs to joints. It is the fiddliest part of adding an airframe, and the part worth
copying from an existing one.

## A backend (autopilot)

```
registry/backends/<id>/
├── backend.yaml       settings, defaults, required binaries
└── adapter.py         a subclass of AutopilotAdapter
```

No core file changes. The orchestrator never branches on a backend id — if you need it
to, the capability schema is missing something, so extend that instead.

The adapter only ever *describes* work; the supervisor runs it. See the interface in
[`architecture.md`](architecture.md#adapter-interface) and the two worked examples in
`registry/backends/ardupilot/` and `registry/backends/px4/`. The 17-line
`registry/backends/static/adapter.py` is the minimal case.

## An ROI prop type or a new marker id

A new search-and-rescue prop is three edits and no code:

1. a `{% macro %}` in `worlds/templates/rois.sdf.j2` built from primitives only — no
   Fuel models (they cost first-run download time) and no `<plane>` (there is exactly one
   plane collision per world);
2. the name in `DEFAULT_ROI_TYPES` in `uavsim/world.py`;
3. the same name in the `targets.rois.types` enum in
   `registry/schemas/scenario.schema.json`.

Types cycle in order, so `count: 8` over five types gives two of the first three.

Marker ids beyond 7 need more marker models, and the schema caps `count` at however many
ship:

```bash
python3 tools/gen_aruco_textures.py --ids 0-15
```

then raise `maximum` on `targets.rois.count`. The PNGs and model files are generated and
committed — do not hand-edit them, `--check` will revert you.

## Resizing the forest

`worlds/fragments/roi_forest.{sdf,yaml}` are generated. Change the numbers, not the SDF:

```bash
python3 tools/gen_roi_forest.py --size 500 --oak 436 --pine 290 \
        --fires 40 --seed 20251007
```

The forest is a square of side `--size`, centred on the origin. Keep the tree counts
proportional to the *area* if you want the density unchanged (2.9 trees per 1000 m²).
A 1000 m square is `--size 1000 --oak 1742 --pine 1162`, but measure first: it roughly
halves the real-time factor once cameras are rendering (the generator's docstring has
the numbers).

If you change `--size`, also update `DEFAULT_GCS_POSE` in `uavsim/site.py` and
`gcs_pose` in the scenarios: the GCS sits so that 100 m of open ground remains between
the apron's front edge and the forest edge, and the arithmetic is in that module's
docstring. `uavsim plan` prints the resulting standoff.

## Checking your work

```bash
python3 -m pytest tests/ -q                 # schema, allocation, rendering
python3 tools/gen_roi_forest.py --check     # the committed forest matches the generator
python3 tools/gen_aruco_textures.py --check # ditto the marker assets (needs OpenCV)
./bin/uavsim list vehicles                  # does it appear
./bin/uavsim plan scenarios/<yours>.yaml    # does it allocate
./bin/uavsim plan scenarios/<yours>.yaml --sdf   # what SDF does it render
./bin/uavsim verify scenarios/<yours>.yaml  # static checks
```

`tests/test_registry.py` walks every shipped vehicle type automatically, so a new one is
covered by the existing tests the moment you add the file — including the check that it
renders to well-formed SDF and does not introduce a second `<plane>` collision.
