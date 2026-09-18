# uavsim

A ready-made Gazebo environment for testing UAV autonomy: a ground control station
500 m west of a forest search area, and a fleet of whatever aircraft you ask for —
ArduPilot and PX4 in the same world, spawned at runtime from a one-file scenario,
with autopilots, cameras, a ROS 2 bridge and a machine-readable manifest.

**There is no mission logic here on purpose.** No search patterns, no arming sequences,
no planner. This is the environment your framework gets tested *in* — it hands you
vehicles, sensors and ground truth, and stays out of the way.

Built for the MeitY Grand Challenge, useful for any multi-UAV work.

```bash
./bin/uavsim up scenarios/forest_rescue.yaml
```

---

## What you get

- **A ground control station** — 24 landing pads, a 120 m runway pointing at the forest,
  a shelter, a mast and a windsock. Vehicles spawn here, not in the trees.
- **A forest region of interest** — ~100 trees, grass and a 60 m boundary ring at the
  world origin. The world's geographic origin *is* the ROI centre.
- **Optional targets** — fire markers and casualties, placed deterministically from a
  seed so the same seed gives the same layout every run.
- **A fleet you choose** — edit one `fleet:` block. Ports, MAVLink system IDs, gz topics
  and ROS 2 namespaces are all assigned for you.
- **A manifest** — every vehicle's capabilities, endpoints and spawn position, plus the
  ground-truth position of every target, in both ENU and WGS84.

## Requirements

| | |
|---|---|
| **Gazebo** | Harmonic (default), Ionic, Jetty or Garden — [install guide](https://gazebosim.org/docs) |
| **ArduPilot** | An [ArduPilot dev environment](https://ardupilot.org/dev/index.html) with `sim_vehicle.py` on `PATH`, for the `ardupilot_*` vehicles |
| **PX4** | A [PX4 checkout](https://docs.px4.io/main/en/dev_setup/building_px4.html) built for SITL, plus `MicroXRCEAgent`, for the `px4_*` vehicles |
| **ROS 2** | Optional. Humble or newer with `ros_gz_bridge`, for the camera/topic bridge |
| **Tools** | `tmux`, `screen` |

```bash
# plugin build dependencies (Ubuntu, Gazebo Harmonic)
sudo apt update
sudo apt install libgz-sim8-dev rapidjson-dev \
                 libopencv-dev libgstreamer1.0-dev \
                 libgstreamer-plugins-base1.0-dev \
                 gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
                 tmux screen

# python dependencies
pip install -r requirements.txt
```

`uavsim up` and `uavsim spawn` also need the Gazebo transport bindings, which are apt
packages rather than pip ones: `sudo apt install python3-gz-transport13 python3-gz-msgs10`
(adjust the version numbers to your Gazebo release).

## Install

```bash
git clone https://github.com/<you>/uavsim.git
cd uavsim
pip install -r requirements.txt

export GZ_VERSION=harmonic          # or ionic / jetty / garden
cmake -B build -S gz_plugins
cmake --build build -j4
```

That builds the ArduPilot Gazebo plugins into `build/`. `uavsim` finds them there
automatically — you do not need to set `GZ_SIM_SYSTEM_PLUGIN_PATH` or
`GZ_SIM_RESOURCE_PATH` yourself.

Build the SITL binaries you intend to use, once:

```bash
cd ~/ardupilot && ./waf configure --board sitl && ./waf copter   # and ./waf plane
```

`uavsim` always runs SITL with `--no-rebuild`, because two `sim_vehicle.py` instances
running `waf configure` at the same time corrupt each other's build directory.

## Quickstart

```bash
./bin/uavsim up scenarios/forest_rescue.yaml    # world + fleet + autopilots + bridge
tmux attach -t uavsim-rescue                    # one MAVProxy console per vehicle
./bin/uavsim verify scenarios/forest_rescue.yaml --live
./bin/uavsim down scenarios/forest_rescue.yaml --gazebo
```

Inside tmux: `Ctrl-b w` to switch vehicles, `Ctrl-b d` to detach. To fly one:

```
mode guided
arm throttle
takeoff 30
rc 7 1200          # gimbal pitch (RC 6/7/8 = gimbal roll/pitch/yaw)
```

The reference scenario puts 4 gimbal-carrying Iris and 2 PX4 X500 on the pads.
Trees are fetched from [Gazebo Fuel](https://app.gazebosim.org/fuel) the first time you
load a world and cached in `~/.gz/fuel` — **the first run on a fresh machine needs
network access**, after which it works offline.

## Choosing a fleet

Everything you normally change lives in one scenario file:

```yaml
fleet:
  - {type: ardupilot_iris_gimbal, count: 4}
  - {type: px4_x500,              count: 2}
  - {type: ardupilot_zephyr,      count: 1}
```

Re-run `uavsim up`. Nothing else needs editing — no world file, no model directories,
no bridge config.

### Shipped vehicle types

`./bin/uavsim list vehicles`

| id | airframe | autopilot | take-off | payload | status |
|---|---|---|---|---|---|
| `ardupilot_iris_gimbal` | Iris | ArduCopter SITL | pad | 3-axis gimbal camera | flight-tested |
| `px4_x500` | Holybro X500 | PX4 SITL | pad | — | flight-tested |
| `ardupilot_iris` | Iris | ArduCopter SITL | pad | — | not tested |
| `px4_x500_depth` | Holybro X500 | PX4 SITL | pad | OAK-D Lite (RGB + depth) | not tested |
| `px4_standard_vtol` | Standard VTOL | PX4 SITL | pad | — | not tested |
| `px4_advanced_plane` | Fixed wing | PX4 SITL | runway | — | not tested |
| `px4_rc_cessna` | Fixed wing | PX4 SITL | runway | — | not tested |
| `static_iris` | Iris | none | pad | gimbal camera | not tested |
| `ardupilot_zephyr` | Zephyr flying wing | ArduPlane SITL | runway | — | ⚠️ does not take off |

**Status means what has actually been run**, not what is expected to work.
*Flight-tested* means a fleet of them has been brought up, verified live and flown.
*Not tested* means the definition is shipped and validated by the test suite, but nobody
has confirmed it in the air — treat those as a starting point, not a guarantee.
See [`docs/known-issues.md`](docs/known-issues.md).

ArduPilot and PX4 vehicles can share one run. Under lock-step the whole simulation runs
at the slowest ArduPilot SITL's pace — correct, but slower than wall clock.

Adding a new type is one YAML file; see [`docs/extending.md`](docs/extending.md).
Known limitations are listed in [`docs/known-issues.md`](docs/known-issues.md).

### Targets

Both shipped scenarios have targets switched **off**. To place them:

```yaml
targets:
  fires: true
  casualties: {count: 5, seed: 7}
```

Positions land in the manifest as ground truth, clear of trunks and of each other.

## Scenarios

| file | what it is |
|---|---|
| `scenarios/forest_rescue.yaml` | The reference run: 4 × Iris with gimbals + 2 × PX4 X500, GUI, tmux consoles |
| `scenarios/forest_static.yaml` | One Iris, no autopilot, headless — world inspection, screenshots, CI |
| `scenarios/zephyr_demo.yaml` | One ArduPlane flying wing on the runway (see [known issues](docs/known-issues.md)) |

## Commands

| command | what it does |
|---|---|
| `uavsim list [vehicles\|airframes\|backends\|payloads]` | What you can ask for (defaults to `vehicles`) |
| `uavsim plan SCENARIO [--sdf] [--no-check]` | Validate and show the allocation without starting anything |
| `uavsim world SCENARIO` | Build just the world SDF and print its path |
| `uavsim up SCENARIO [--no-wait] [--no-bridge] [--no-check]` | World, fleet, autopilots and bridge |
| `uavsim spawn SCENARIO TYPE [--count N] [--no-wait]` | Add vehicles to a run that is already up |
| `uavsim verify SCENARIO [--live]` | Static checks; `--live` also checks the running sim |
| `uavsim manifest SCENARIO` | Print the run manifest |
| `uavsim down SCENARIO [--gazebo]` | Stop the fleet; `--gazebo` stops the server too |

`bin/uavsim` is a wrapper around `python3 uavsim/cli.py` that also puts
`$ARDUPILOT_ROOT/Tools/autotest` (default `~/ardupilot`) on `PATH`.

## What you get back

Each run writes to `$UAVSIM_RUN_DIR/<world>/` (default `/tmp/uavsim/<world>/`):

- **`manifest.json`** — the contract between uavsim and your framework
- **`bridge.yaml`** — generated `ros_gz_bridge` config, started for you unless `--no-bridge`
- **`logs/`** — per-process logs when `supervisor: subprocess`

```jsonc
{
  "roi":  {"centre_geo": [-35.389138, 149.219188], "mission_radius_m": 60.0},
  "gcs":  {"pose_enu": [-500, 0, 0, 0, 0, 0], "geo": [...], "distance_to_roi_m": 500.0},
  "targets": {"fires": [{"enu": [...], "geo": [...]}], "casualties": [...]},
  "vehicles": [{
    "name": "ardupilot_iris_gimbal_0", "sysid": 1,
    "spawn": {"enu": [...], "geo": [...]},
    "capabilities": {"platform": {...}, "autopilot": {...}, "sensors": [...], "actuators": [...]},
    "endpoints": {"mavlink": "tcp:127.0.0.1:5760", "fdm_udp": 9002}
  }]
}
```

ROS 2 topics follow from the capabilities: camera images on
`/<vehicle>/<payload>/image_raw`, gimbal commands on
`/<vehicle>/<payload>/cmd_{roll,pitch,yaw}`, and PX4 vehicles additionally speak
`px4_msgs` under `/<vehicle>/fmu/...`.

## Running headless

Set `supervisor: subprocess` and `gazebo: {headless: true}` in the scenario. There is no
MAVProxy in this mode — connect straight to SITL on `tcp:127.0.0.1:5760+10·i`. The
manifest's `endpoints.mavlink` always tells you which endpoint applies.

## Running from ROS 2

```bash
# in a colcon workspace
ln -s /path/to/uavsim/ros2/uavsim_bringup src/uavsim_bringup
colcon build --packages-select uavsim_bringup
source install/setup.bash

export UAVSIM_ROOT=/path/to/uavsim
ros2 launch uavsim_bringup sim.launch.py scenario:=forest_rescue
```

`UAVSIM_ROOT` is only needed when the launch file cannot find the checkout by walking up
from its own location, which is the case once colcon has installed it.

## Tests

```bash
python3 -m pytest tests/ -q
```

37 tests covering registry validation, capability merging, allocation invariants, SDF
rendering, world building, casualty placement, bridge generation and manifest content.
They need no Gazebo and no SITL, and run in under a second.

## Things that will bite you

- **Never start Gazebo paused.** ArduPilot's lock-step handshake deadlocks: SITL resends
  the same servo frame while nothing steps. `uavsim` always uses `-r`.
- **One stalled SITL freezes everything.** Lock-step has no timeout recovery, so a dead
  autopilot wedges every other vehicle *and* Gazebo. `uavsim verify --live` checks that
  simulation time is still advancing.
- **A stale Gazebo does not fail loudly.** Two servers can both bind the FDM ports
  (`SO_REUSEADDR`) and silently split the stream. `verify --live` checks each port has
  exactly one owner.
- **There must be exactly one `<plane>` collision in a world.** A plane collision is an
  infinite half-space in DART; a second one makes the ground-contact LCP singular and
  freezes the physics step. The apron, runway and grass patches are boxes or visual-only
  for this reason, and both `verify` and the test suite enforce the rule.
- **The first run needs network** to fetch trees from Gazebo Fuel.
- **Expect a low real-time factor** with many vehicles and cameras. Under lock-step
  ArduPilot follows simulation time, so flight behaviour stays correct — it just runs
  slower than wall clock.

## Documentation

- [`docs/architecture.md`](docs/architecture.md) — how a scenario becomes a running world
- [`docs/extending.md`](docs/extending.md) — adding a vehicle, payload, airframe or autopilot
- [`docs/known-issues.md`](docs/known-issues.md) — what does not work yet
- [`CONTRIBUTING.md`](CONTRIBUTING.md) — how to work on this

## Licence

LGPL-3.0. See [`LICENSE.md`](LICENSE.md).

This repository bundles the [ArduPilot Gazebo plugin](https://github.com/ArduPilot/ardupilot_gazebo)
unmodified in [`gz_plugins/`](gz_plugins/), and third-party aircraft meshes.
See [`NOTICE.md`](NOTICE.md) for full attribution.
