# uavsim participant manual

Everything you need to fly your search-and-rescue solution in the challenge world:
installing it, starting it, connecting your software, scaling your fleet, and handing
in a run.

**Contents**

1. [The challenge world](#1-the-challenge-world)
2. [Install](#2-install)
3. [Quickstart](#3-quickstart)
4. [Your fleet](#4-your-fleet)
5. [Connecting your software](#5-connecting-your-software)
6. [Finding the markers](#6-finding-the-markers)
7. [Run logs and handing in a run](#7-run-logs-and-handing-in-a-run)
8. [Fair-play rules](#8-fair-play-rules)
9. [Troubleshooting](#9-troubleshooting)
10. [Command reference](#10-command-reference)

---

## 1. The challenge world

```
            y (north)
            ^
            |     +--------------------------- 500 m ---------------------------+
            |     |  forest: 726 trees, 10 search-and-rescue sites hidden in it |
   GCS      |     |                                                             |
  apron     |     |                                                             |
 [::::]  ---|-----|-------------------------- (0,0) --------------------------  |---> x (east)
 24 pads    |     |                     world origin = forest centre            |
 x = -356   |     |                                                             |
            |     |                                                             |
            |     +-------------------------------------------------------------+
         x = -365  x = -250                                                  x = 250
          (GCS)    forest edge, 100 m of open ground in front of the apron
```

| | |
|---|---|
| **Search area** | A 500 m × 500 m forest square centred on the world origin: x and y from −250 m to +250 m. 726 trees (oaks with a 5 m canopy radius, pines with 1.6 m). Tree collision shapes reach 6.5 m; cruise at 10 m or more above the ground over the forest. |
| **GCS** | West of the forest at x = −365 m. Its apron has 24 landing pads, 6 m apart, facing the forest. Every drone spawns on a pad. |
| **Geographic origin** | The forest centre: latitude −35.389138, longitude 149.219188, elevation 580 m. |
| **Sites** | 10 sites, two each of five types: survivor, aircraft wreckage, tent, supply crate, signal fire. |
| **Markers** | Each site has one **ArUco 4x4_50** marker (ids **0–9**) printed on a **3 m × 3 m** horizontal white plate lying face up on the ground beside the site's prop. The black marker itself is 2.4 m across. A plate is never under a tree canopy and never under its prop. |
| **Layout** | **Every `uavsim up` places the 10 sites at new random positions.** Marker ids and site types stay the same; where they are does not. Your solution has to search, not remember. |

**Coordinate frames.** Gazebo uses ENU, metres from the forest centre: x east, y north,
z up. MAVLink local positions are NED: `north = y`, `east = x`, `down = −z`.

> **Home is the forest centre, not your pad.** Every ArduPilot instance is started with
> its home at the world origin, so `LOCAL_POSITION_NED` (0, 0) is in the middle of the
> forest, a drone sitting on its pad reports itself about 356 m west of home, and
> **`RTL` flies the drone into the forest**. To come back, fly an explicit leg to your
> spawn point (it is in the manifest, §5.3) and land there.

---

## 2. Install

Tested on **Ubuntu 22.04** with **ROS 2 Humble**, **Gazebo Harmonic** and
**ArduPilot Copter SITL**.

### 2.1 ROS 2 Humble and Gazebo Harmonic

Install ROS 2 Humble ([guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html))
and Gazebo Harmonic ([guide](https://gazebosim.org/docs/harmonic/install_ubuntu)), then:

```bash
sudo apt install ros-humble-ros-gzharmonic \
                 ros-humble-rosbag2-py ros-humble-tf2-msgs \
                 python3-gz-transport13 python3-gz-msgs10 \
                 libgz-sim8-dev rapidjson-dev libopencv-dev \
                 libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
                 gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
                 tmux screen
# optional, if you want MAVROS
sudo apt install ros-humble-mavros ros-humble-mavros-extras

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

ROS 2 must be sourced in every shell you run `uavsim` from: the run log is a ROS 2 bag.

### 2.2 ArduPilot SITL

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git ~/ardupilot
cd ~/ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
./waf configure --board sitl && ./waf copter
```

`uavsim` expects the checkout at `~/ardupilot` (set `ARDUPILOT_ROOT` if yours is elsewhere)
and never rebuilds it for you: build `copter` once, as above.

### 2.3 This repository

```bash
git clone <this repository> uavsim
cd uavsim
pip install -r requirements.txt

export GZ_VERSION=harmonic
cmake -B build -S gz_plugins
cmake --build build -j4
```

That builds the ArduPilot Gazebo plugin into `build/`, where `uavsim` finds it. You do
not need to set any `GZ_SIM_*` variables yourself.

Check the installation without starting anything:

```bash
./bin/uavsim plan scenarios/forest_rescue.yaml      # ends with "ready: run 'uavsim up'"
```

### 2.4 First start

The first time the world loads, Gazebo downloads the two tree models from Gazebo Fuel
and caches them in `~/.gz/fuel`. That needs network access, takes a minute or two and
shows no progress bar. If the first `uavsim up` says the world never came up, run
`uavsim down` and `uavsim up` again: the second start uses the cache.

---

## 3. Quickstart

```bash
./bin/uavsim up scenarios/forest_rescue.yaml --team my_team
```

This builds the world with a fresh site layout, opens the Gazebo window, spawns the fleet
on the apron, starts one ArduPilot SITL per drone, starts the ROS 2 bridge and opens the
run log. It prints the fleet with each drone's MAVLink endpoint.

Fly one drone by hand from its MAVProxy console:

```bash
tmux attach -t uavsim-rescue        # Ctrl-b w: pick a drone's window, Ctrl-b d: detach
```

```
mode guided
arm throttle
takeoff 20
```

End the run (this stops the drones and Gazebo, then closes and packs the run log):

```bash
./bin/uavsim down scenarios/forest_rescue.yaml
```

---

## 4. Your fleet

### 4.1 The shipped drone

| type | what it is |
|---|---|
| `ardupilot_iris_gimbal` | A 3DR Iris quadcopter flown by ArduCopter SITL, carrying a **3-axis gimbal camera** (640×480, 115° horizontal field of view, 10 Hz). |

It is the drone the challenge is built around, and the one we recommend you use: the
gimbal camera is what reads the markers, it is flight-tested in this world, and it runs
identically on the organisers' machine.

### 4.2 More drones

The fleet is one block in the scenario file. To fly six drones instead of four, change
the count and run `uavsim up` again:

```yaml
fleet:
  - type: ardupilot_iris_gimbal
    count: 6
```

Two entries of the same type need different name prefixes:

```yaml
fleet:
  - {type: ardupilot_iris_gimbal, count: 4}
  - {type: ardupilot_iris_gimbal, count: 2, name_prefix: scout}
```

To add drones **to a run that is already up**:

```bash
./bin/uavsim spawn scenarios/forest_rescue.yaml ardupilot_iris_gimbal --count 2
```

They take the next free pads, names, system ids and ports, appear in the manifest, and
are recorded in the run log like the rest.

Limits: the apron has **24 pads**. Every drone and camera costs CPU; with four drones the
simulation already runs well below real time on a typical workstation (§9). ArduPilot
runs in lock-step with Gazebo, so the flight itself stays correct; it just takes longer.

### 4.3 Names, ids and ports

Drones are numbered in fleet order from 0. For drone *i*:

| | value |
|---|---|
| model name / ROS namespace | `ardupilot_iris_gimbal_<i>` (or `<name_prefix>_<n>`) |
| MAVLink system id | *i* + 1 |
| MAVLink for your code (MAVProxy output) | `udp:127.0.0.1:<14650 + 10·i>` |
| second MAVLink output (e.g. for MAVROS) | `127.0.0.1:<14750 + i>` |
| SITL's own port (MAVProxy's) | `tcp:127.0.0.1:<5760 + 10·i>` |

Never hardcode these: read them from the manifest (§5.3), which always matches the run.

### 4.4 Your own drones

You can deploy drones of your own design in this environment by adding them to the
registry and listing them in the fleet, exactly like the shipped one.
**[`CUSTOM_DRONES.md`](CUSTOM_DRONES.md) has the step-by-step guide**, from a tuned
variant of the shipped drone up to your own airframe model.

> **Advisory.** Use the shipped `ardupilot_iris_gimbal` unless you have a strong reason
> not to. It already carries the gimbal camera the mission needs to read the markers, it
> is tested in this world, and the organisers can run it on the final machine with no
> extra files. A drone of your own needs a camera too (the guide shows how to mount the
> shipped one), and every file it needs has to travel with your entry.

---

## 5. Connecting your software

The environment contains no mission logic. Your software talks to the drones the same
way it would talk to real ArduPilot vehicles, and to their cameras over ROS 2.

### 5.1 MAVLink

Connect to each drone's endpoint from the manifest. With pymavlink:

```python
from pymavlink import mavutil
m = mavutil.mavlink_connection("udp:127.0.0.1:14650", source_system=250)
m.wait_heartbeat()                       # check m.target_system == the drone's sysid
```

Each endpoint is a single UDP port that exactly one process can listen on. If two
programs need the same drone (say, your planner and MAVROS), give the second one the
second output (`endpoints.extra_out`, 14750 + i).

Everything ArduCopter supports in GUIDED mode works: `MAV_CMD_NAV_TAKEOFF`,
`SET_POSITION_TARGET_GLOBAL_INT` / `_LOCAL_NED`, `MAV_CMD_DO_REPOSITION`,
missions in AUTO, `MAV_CMD_NAV_LAND`. Remember the home trap in §1.

### 5.2 Camera and gimbal

| | |
|---|---|
| ROS 2 image topic | `/<drone>/gimbal_cam/image_raw` (`sensor_msgs/msg/Image`) |
| Gazebo image topic | `/world/rescue/model/<drone>/model/gimbal_cam/link/pitch_link/sensor/camera/image` |
| Resolution | 640 × 480 |
| Horizontal field of view | 2.0 rad (about 115°) |
| Nominal rate | 10 Hz (fewer in wall-clock terms when the simulation runs slower than real time) |

**Point the camera straight down** with the standard MAVLink mount command, pitch −90°:

```python
m.mav.command_long_send(sysid, 1, mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL, 0,
                        -90, 0, 0,     # pitch, roll, yaw in degrees
                        0, 0, 0,
                        mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING)
```

It stays at that angle until you command another. The limits are pitch −135° to +45°,
roll ±30° and yaw ±160°. Until you command it, the camera looks forward and down
at an angle, partly at the drone's own landing gear.

The gimbal also listens on `/<drone>/gimbal_cam/cmd_{roll,pitch,yaw}`, but ArduPilot
drives the same joints, so use the MAVLink command above rather than those topics.

### 5.3 The manifest

`uavsim up` writes `/tmp/uavsim/rescue/manifest.json` (print it with
`./bin/uavsim manifest scenarios/forest_rescue.yaml`). It is the one place to read the
fleet from:

```jsonc
{
  "world":   {"name": "rescue", "origin": {"latitude_deg": -35.389138, "longitude_deg": 149.219188, "elevation_m": 580.0}},
  "roi":     {"size_m": [500, 500], "mission_extent_enu": [-250, 250, -250, 250],   // xmin, xmax, ymin, ymax
              "boundary_geo": [[lat, lon], ...]},                                    // SW, SE, NE, NW corners
  "gcs":     {"pose_enu": [-365, 0, 0, 0, 0, 0], "geo": [lat, lon], "pads": 24},
  "targets": {"roi_count": 10,
              "marker": {"dictionary": "4x4_50", "size_m": 3.0, "ids": [0, 1, ..., 9]}},
  "vehicles": [{
    "name": "ardupilot_iris_gimbal_0", "sysid": 1,
    "spawn": {"enu": [-356.0, -15.0, 0.195, 0, 0, 0.04], "geo": [lat, lon]},
    "endpoints": {"mavlink": "udp:127.0.0.1:14650", "extra_out": ["127.0.0.1:14750"], ...},
    "capabilities": {"sensors": [{"kind": "camera.rgb", "ros": {"topic": "/ardupilot_iris_gimbal_0/gimbal_cam/image_raw"}}, ...]}
  }]
}
```

The manifest says how many sites there are and what their markers look like. It does
**not** say where they are: finding them is the challenge.

### 5.4 MAVROS (optional)

With the bringup package built in a colcon workspace (`ros2/uavsim_bringup`), one MAVROS
node per drone, on the drones' second outputs:

```bash
ros2 launch uavsim_bringup mavros.launch.py world:=rescue
```

---

## 6. Finding the markers

The camera's ground footprint and the marker's size in the image both depend on
altitude. With the 640×480, 115° camera pointing straight down at a 3 m plate:

| altitude above ground | footprint width | plate in the image | detection in our tests |
|---|---|---|---|
| 15 m | 47 m | 41 px | 30 / 30 |
| 20 m | 62 m | 31 px | 30 / 30 |
| 25 m | 78 m | 25 px | 26 / 30 |
| 30 m | 93 m | 21 px | 30 / 30 |
| 35 m | 109 m | 18 px | 12 / 30 |

(OpenCV's ArUco detector with parameters tuned for small markers, the camera held still
over every plate, three layouts. Your detector, motion blur and the frame rate change
these numbers.)

Practical advice:

- **Search at 15–25 m.** Lower is more reliable; higher covers more ground per pass.
  The trees reach about 7 m, so there is room either way.
- **Slow down or hover to confirm a detection.** Frames arrive at a few per second of
  wall-clock time; a fast drone gets few looks at each plate.
- A marker's id says which marker it is, not where it is. Your solution has to work out
  where each marker is, in the world, from your own drone's position and camera.

---

## 7. Run logs and handing in a run

Every `uavsim up` records the run for the judges, and every `uavsim down` closes the
record. Nothing for you to start.

### 7.1 What is recorded

Where every drone was, 10 times per simulated second: the true position and orientation
of every moving model in the world, your own drones included. That is all: no images and
no telemetry. Times are simulated seconds, so a slow computer does not make a solution
look slow.

### 7.2 Where it goes

```
uavsim_runs/<team>_<UTC start time>/     inside this repository (git-ignored)
    bag/                ROS 2 bag of the poses
    run_info.json       team, scenario, code version, fleet, start and end time
    scenario.yaml       the scenario file exactly as it was run
    ground_truth.json   the site layout, for the judges
    SHA256SUMS
uavsim_runs/<team>_<UTC start time>.tar.gz   the same, packed: this is what you hand in
```

Name your team on the command line (`uavsim up ... --team my_team`) or once in the
scenario:

```yaml
logging:
  team: my_team
```

### 7.3 Ending a run properly

**Always end with `uavsim down`.** It closes the log, writes the checksums, packs the
`.tar.gz` and prints its path and SHA-256:

```
  run log: .../uavsim_runs/my_team_20261011T123901Z (2709 pose samples)
  hand in: .../uavsim_runs/my_team_20261011T123901Z.tar.gz
  sha256:  c9d51230...
```

If the computer crashes or the terminal is killed, run `uavsim down` anyway: it repairs
the bag and packs what was recorded.

### 7.4 Checking your own run

```bash
./bin/uavsim review                  # the newest run: flight times, distance, a top-down map
./bin/uavsim replay --speed 5        # watch it again in the 3D world
```

Both also take a log directory or `.tar.gz`. `review` writes the map next to the log.

---

## 8. Fair-play rules

1. Your solution may use only what a real drone would have: its own camera images and
   its own autopilot telemetry (MAVLink / MAVROS), plus the manifest.
2. Do not read the simulator's ground truth: no Gazebo world, scene or pose topics or
   services (`/world/rescue/...` other than your drones' camera topics), no
   `ground_truth.json`, no run-log contents during a run, no parsing of the world SDF.
3. Do not change the world, the site placement or the logging code.
4. The final run is on the organisers' machine with a layout nobody has seen before.

---

## 9. Troubleshooting

| symptom | cause and fix |
|---|---|
| "world never came up" on the first start | The tree models were still downloading. Run `uavsim down` and `uavsim up` again (§2.4). |
| A drone flies off into the forest when told to return | `RTL` goes home, and home is the forest centre (§1). Fly to your spawn point and land. |
| No heartbeat on a drone's port | Another program is already listening on that UDP port. Use `endpoints.extra_out` for the second program. |
| The whole simulation froze | One stalled ArduPilot instance stops everyone, Gazebo included (lock-step). `uavsim verify scenarios/forest_rescue.yaml --live` tells you which; then `uavsim down` and `up`. |
| Strange behaviour after an earlier run | Something from that run is still alive. `uavsim down`, then `uavsim verify ... --live`. |
| The simulation runs well below real time | Expected with several drones and cameras (a full fleet runs at roughly 0.3× real time on a workstation). Flight behaviour is unaffected; it just takes longer. Fewer drones run faster. |
| A drone is missing in the Gazebo window but flies | Do not start the GUI yourself; `uavsim up` waits for it before spawning. To jump the view to a drone: right-click it in the entity tree and choose "Move to". |
| `uavsim up` says ROS 2 is missing | `source /opt/ros/humble/setup.bash` in that shell. |
| `review` says the bag has no `metadata.yaml` | The run was not ended with `uavsim down`. Run `ros2 bag reindex uavsim_runs/<run>/bag`. |

---

## 10. Command reference

| command | what it does |
|---|---|
| `uavsim up SCENARIO [--team NAME] [--seed N]` | Start a run: world, fleet, autopilots, ROS 2 bridge, run log. `--seed` replays an earlier layout (judges' use). |
| `uavsim down SCENARIO [--keep-gazebo]` | End the run: stop everything, close and pack the run log. |
| `uavsim spawn SCENARIO TYPE [--count N]` | Add drones to a running run. |
| `uavsim manifest SCENARIO` | Print the manifest of the running run. |
| `uavsim verify SCENARIO [--live]` | Check the installation; `--live` also checks the running simulation. |
| `uavsim plan SCENARIO [--sdf]` | Show the fleet allocation without starting anything. |
| `uavsim list [vehicles\|airframes\|payloads]` | What the registry offers. |
| `uavsim review [LOG]` | Summarise a run log and draw its tracks. |
| `uavsim replay [LOG] [--speed X] [--start S]` | Replay a run log in Gazebo. |

`./bin/uavsim` is the entry point; it also puts `~/ardupilot/Tools/autotest` on `PATH`.
