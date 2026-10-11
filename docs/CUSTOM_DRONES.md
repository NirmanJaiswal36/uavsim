# Deploying your own drones

You are not limited to the drone that ships with the environment. Any quadcopter you
describe to the registry can be listed in your scenario's `fleet:` block, spawned on
the GCS pads, flown by its own ArduPilot SITL, and recorded in the run log like any
other drone.

> **Advisory: use the shipped `ardupilot_iris_gimbal` if you can.**
> It already carries the 3-axis gimbal camera that the mission needs to read the ArUco
> markers, it is flight-tested in this world, and the organisers can run it on the final
> machine with nothing extra from you. A drone of your own:
>
> - **needs a camera to complete the mission.** The simplest way is to mount the shipped
>   gimbal camera on it, as every example below does;
> - **is your responsibility on the final run**, which happens on the organisers'
>   machine: every file it needs (listed under [What to hand in](#what-to-hand-in))
>   has to travel with your entry;
> - **costs simulation speed** if it is heavier to simulate than the Iris.

There are three levels, from least to most work:

| level | you want | you write |
|---|---|---|
| [1](#level-1-more-of-the-shipped-drone) | more drones, or groups with their own names | nothing but the `fleet:` block |
| [2](#level-2-a-tuned-variant-of-the-shipped-drone) | the shipped drone with different autopilot settings | one vehicle file and one parameter file |
| [3](#level-3-your-own-airframe) | your own airframe model | a model, an airframe, a binding, a parameter file, a vehicle file |

Every drone in this environment is flown by **ArduPilot SITL** (ArduCopter). Other
autopilots, and drones with no autopilot, are not supported.

---

## How a fleet entry becomes a drone

```
scenario fleet:  - type: my_quad_gimbal          which vehicle type, how many
                          |
registry/vehicles/my_quad_gimbal.yaml            = airframe + backend + payloads + params
        |                    |                 |
        v                    v                 v
registry/airframes/     registry/backends/   registry/payloads/
  my_quad/                ardupilot/           gimbal_camera_3axis/
    airframe.yaml           (ships)              (ships: the camera)
    bindings/
      ardupilot.sdf.j2   -> the ArduPilotPlugin block that connects the model to SITL
        |
        v
models/my_quad/model.sdf                         the physical model: links, joints, IMU
config/my_quad.parm                              ArduPilot parameters for this frame
```

`uavsim` reads everything under `registry/` at start-up and checks it against the
schemas in `registry/schemas/`. You never edit Python code, the world, or any port
table: names, MAVLink system ids, ports and topics are assigned for you.

---

## Level 1: more of the shipped drone

In your scenario (start from a copy of `scenarios/forest_rescue.yaml`):

```yaml
fleet:
  - type: ardupilot_iris_gimbal
    count: 4
  - type: ardupilot_iris_gimbal
    count: 2
    name_prefix: scout            # scout_0, scout_1: two entries of one type need two names
```

or add drones to a run that is already up:

```bash
./bin/uavsim spawn scenarios/forest_rescue.yaml ardupilot_iris_gimbal --count 2
```

---

## Level 2: a tuned variant of the shipped drone

Say you want the Iris to fly its GUIDED legs at 15 m/s instead of ArduCopter's default.
That is an ArduPilot parameter, so the variant is one parameter file plus one vehicle
file.

**`config/fast_iris.parm`**, your parameters (`NAME value`, one per line, `#` comments):

```
# horizontal speed for GUIDED / AUTO legs, cm/s
WPNAV_SPEED      1500
```

**`registry/vehicles/fast_iris_gimbal.yaml`**, the vehicle type:

```yaml
id: fast_iris_gimbal
description: The shipped Iris with its gimbal camera, flying faster legs.
airframe: iris
backend: ardupilot

payloads:
  - type: gimbal_camera_3axis       # keep the camera: the mission needs it
    id: gimbal_cam
    mount: belly

backend_config:
  # applied in order, later files win. Keep the shipped file first: it configures
  # the motors and the gimbal mount, and without it the drone will not fly properly
  param_files: [config/gazebo-iris-gimbal.parm, config/fast_iris.parm]
```

Then use it in the fleet:

```yaml
fleet:
  - type: fast_iris_gimbal
    count: 4
```

The same `param_files` list can also go straight on a fleet entry, without a new vehicle
file:

```yaml
fleet:
  - type: ardupilot_iris_gimbal
    count: 4
    backend_config:
      param_files: [config/gazebo-iris-gimbal.parm, config/fast_iris.parm]
```

Check it, then fly it:

```bash
./bin/uavsim list vehicles                       # fast_iris_gimbal is listed
./bin/uavsim plan scenarios/my_team.yaml         # ends with "ready"
./bin/uavsim up   scenarios/my_team.yaml --team my_team
```

---

## Level 3: your own airframe

This walks through a complete, working example, `my_quad`, and then explains what to
change to make it really yours. Run every command from the repository root.

The example starts from a copy of the Iris model so that it flies first time. Get it
flying exactly like this before you change anything; then change one thing at a time.

### Step 1: the model

The model is a normal Gazebo SDF model under `models/`. It has to contain a base link,
an IMU sensor, and one link plus one revolute joint per rotor.

```bash
cp -r models/iris_with_standoffs models/my_quad
sed -i 's/iris_with_standoffs/my_quad/g' models/my_quad/model.sdf
sed -i '0,/<name>.*<\/name>/s//<name>my_quad<\/name>/' models/my_quad/model.config
```

The first `sed` renames the model and points its mesh URIs (`model://my_quad/meshes/...`)
at the copied meshes.

### Step 2: the airframe

**`registry/airframes/my_quad/airframe.yaml`**:

```yaml
id: my_quad
description: Our team's quadcopter (started from the Iris).
class: multirotor

model_uri: model://my_quad
base_link: my_quad::base_link
imu_sensor: my_quad::imu_link::imu_sensor

geometry:
  footprint_radius_m: 0.35      # half the widest span, rotors included
  spawn_z_m: 0.195              # base_link height when standing on the pad

mounts:
  belly:                        # where the gimbal camera bolts on
    parent: my_quad::base_link
    pose: [0, -0.01, -0.124923, 90, 0, 90]
    degrees: true

bindings:
  ardupilot:
    overlay: bindings/ardupilot.sdf.j2
    vars:
      vehicle: ArduCopter
      frame: gazebo-iris
```

```bash
mkdir -p registry/airframes/my_quad/bindings
# then save the file above as registry/airframes/my_quad/airframe.yaml
```

### Step 3: the ArduPilot binding

The binding is the part of the model that connects it to ArduPilot: the
`ArduPilotPlugin` block that maps each autopilot output channel to a rotor joint, plus
the per-rotor aerodynamics. For the example, it is the Iris binding with the names
changed:

```bash
sed 's/iris_with_standoffs/my_quad/g' registry/airframes/iris/bindings/ardupilot.sdf.j2 \
  > registry/airframes/my_quad/bindings/ardupilot.sdf.j2
```

### Step 4: the parameters

```bash
cp config/gazebo-iris-gimbal.parm config/my_quad.parm
```

### Step 5: the vehicle type

**`registry/vehicles/my_quad_gimbal.yaml`**:

```yaml
id: my_quad_gimbal
description: Our quad, carrying the shipped 3-axis gimbal camera.
airframe: my_quad
backend: ardupilot

payloads:
  - type: gimbal_camera_3axis
    id: gimbal_cam
    mount: belly

backend_config:
  param_files: [config/my_quad.parm]
```

### Step 6: fly it

Put it in your scenario's fleet, next to or instead of the shipped drone:

```yaml
fleet:
  - type: ardupilot_iris_gimbal
    count: 1
  - type: my_quad_gimbal
    count: 1
```

```bash
./bin/uavsim list vehicles                   # my_quad_gimbal  [my_quad + ardupilot]
./bin/uavsim plan scenarios/my_team.yaml     # the fleet table, then "ready"
./bin/uavsim plan scenarios/my_team.yaml --sdf | less   # the exact model that will be spawned
./bin/uavsim up   scenarios/my_team.yaml --team my_team
```

It spawns on the next pad, gets its own MAVLink endpoint (in the fleet table and the
manifest), and its camera publishes on `/my_quad_gimbal_0/gimbal_cam/image_raw`. Fly it
from its MAVProxy window (`tmux attach -t uavsim-rescue`) or from your code exactly like
the shipped drone. End the run with `uavsim down`; `uavsim review` shows its track.

### Making it really yours

Once the copy flies, change it a step at a time and fly after each step.

**The model (`models/my_quad/model.sdf`)**

- Give every link a realistic `<inertial>` block (mass and inertia). The total mass
  matters to the autopilot's hover throttle.
- Keep an IMU sensor on a link rigidly attached to the base, and keep the
  `imu_sensor` path in `airframe.yaml` pointing at it
  (`<model>::<link>::<sensor>`).
- One link and one revolute joint per rotor, the joint's axis along the rotor shaft.
  The binding refers to these by name.
- Use a `<collision>` on the base so the drone can stand on the pad. **Never add a
  `<plane>` collision**: the world allows exactly one, and a second freezes the physics.
- Meshes go in `models/my_quad/meshes/` and are referenced as `model://my_quad/meshes/...`.

**The binding (`registry/airframes/my_quad/bindings/ardupilot.sdf.j2`)**

- Keep the template variables exactly as they are: `{{ airframe.model_uri }}`,
  `{{ fdm_addr }}`, `{{ fdm_port_in }}`, `{{ lock_step }}`, `{{ airframe.imu_sensor }}`.
  `uavsim` fills them in per drone; hardcoding a port breaks the second drone.
- Keep **`{{ payload_controls }}`** inside the `ArduPilotPlugin` block. That is where the
  gimbal camera's three control channels are inserted.
- One `<control channel="N">` per motor, `N` = the ArduPilot motor output (channel 0 =
  motor 1). Its `<jointName>` is your rotor joint. The sign of `<multiplier>` is the spin
  direction; it must match the motor order ArduCopter expects for your frame type (see
  ArduPilot's "Connect ESCs and Motors" page for the motor numbering).
- One `LiftDrag` pair and one `ApplyJointForce` per rotor, naming your rotor links and
  joints.
- Channels 8, 9 and 10 are taken by the gimbal: do not use them for motors.

**The parameters (`config/my_quad.parm`)**

- **This file is mandatory.** `uavsim` starts SITL with `--model JSON`, and in that mode
  ArduPilot does not load its own per-frame defaults. Everything your frame needs must be
  in your file.
- `FRAME_CLASS` / `FRAME_TYPE` for your geometry (1 / 1 is quad X).
- `MOT_PWM_MIN` / `MOT_PWM_MAX` matching the binding's `servo_min` / `servo_max`.
- If your drone is much heavier or lighter than the Iris, `MOT_THST_HOVER` and the
  attitude gains (`ATC_*`) need tuning.
- Keep the gimbal section (`MNT1_*`, `RC6`–`RC8`, `SERVO9`–`SERVO11`) as it is in the
  copied file, or the camera cannot be pointed.

**The airframe (`registry/airframes/my_quad/airframe.yaml`)**

- `footprint_radius_m`: half your widest span, rotors included. The pads are 6 m apart,
  so it must be under 3 m.
- `spawn_z_m`: the base link's height above the ground when the drone stands on its
  feet. Too low and it spawns into the ground; too high and it drops.
- `mounts.belly`: where the gimbal sits relative to your base link (x y z in metres,
  then roll pitch yaw, in degrees with `degrees: true`). Put it under the body with a
  clear view down.

### Checklist before your first `up`

- [ ] `./bin/uavsim list vehicles` lists your vehicle type.
- [ ] `./bin/uavsim plan <scenario>` ends with `ready`, with no problems listed.
- [ ] `./bin/uavsim plan <scenario> --sdf` shows your model with your joint names and
      the gimbal's `<control channel="8">` to `"10">` blocks inside `ArduPilotPlugin`.
- [ ] Every `model://` URI in your SDF points inside `models/`.
- [ ] No `<plane>` collision anywhere in your model.
- [ ] Your parameter file is listed in `backend_config.param_files`.

### When it does not fly

| symptom | likely cause |
|---|---|
| `plan` reports a schema error | A misspelt or missing field; the message names the file and the field. |
| The drone never sends a heartbeat | The binding hardcodes a port, or lost `{{ fdm_port_in }}`; or the model failed to load (read the Gazebo window's tmux pane). |
| It flips over on take-off | Motor order or spin direction: a `<control>` channel drives the wrong rotor, or a `<multiplier>` sign is wrong. Or `FRAME_CLASS`/`FRAME_TYPE` does not match the binding. |
| It cannot climb, or climbs away | Mass versus thrust: check the inertials, `MOT_THST_HOVER`, and the binding's `<multiplier>` magnitude. |
| It sinks into the pad or bounces at spawn | `spawn_z_m` is wrong, or the base has no collision. |
| No camera image | The vehicle file has no `gimbal_camera_3axis` payload, or the mount name does not exist in the airframe. |
| The camera does not respond to the mount command | The `MNT1_*` / `RC6-8` / `SERVO9-11` parameters are missing, or `{{ payload_controls }}` was removed from the binding. |

---

## What to hand in

For the final run on the organisers' machine, every file your drone needs travels with
your entry. For the example above, that is:

```
models/my_quad/                                  (the whole directory, meshes included)
registry/airframes/my_quad/airframe.yaml
registry/airframes/my_quad/bindings/ardupilot.sdf.j2
registry/vehicles/my_quad_gimbal.yaml
config/my_quad.parm
scenarios/my_team.yaml                           (your fleet)
```

Keep your drone's files in exactly these places and nowhere else. Do not edit any file
that ships with the environment (the shipped drone, the world, `uavsim/`): your drone
has to work on an unmodified copy of it.
