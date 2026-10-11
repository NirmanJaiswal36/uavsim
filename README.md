# uavsim

The simulation environment for the MeitY Grand Challenge search-and-rescue mission: a
ground control station beside a 500 m × 500 m forest, ten search-and-rescue sites hidden
in it (each marked with an ArUco marker), and a fleet of camera-carrying quadcopters
flown by ArduPilot SITL in Gazebo.

**There is no mission logic in the environment.** It gives you drones, cameras and a
world; finding the sites is your job.

```bash
./bin/uavsim up   scenarios/forest_rescue.yaml --team my_team    # start a run
tmux attach -t uavsim-rescue                                     # MAVProxy consoles
./bin/uavsim down scenarios/forest_rescue.yaml                   # end it, pack the run log
```

## What you get

- **The world**: a GCS apron with 24 landing pads, 100 m of open ground, then a forest
  of 726 trees. The forest centre is the world origin and the geographic origin.
- **10 sites, re-placed at random on every `uavsim up`**: two each of survivor,
  wreckage, tent, supply crate and signal fire, each with an ArUco 4x4_50 marker (ids
  0–9) on a 3 m plate lying face up beside it.
- **The drone**: `ardupilot_iris_gimbal`, an Iris quadcopter on ArduCopter SITL with a
  3-axis gimbal camera. Fly as many as you like (`fleet:` in the scenario, or
  `uavsim spawn` while running), or bring your own design.
- **A manifest** with every drone's MAVLink endpoint, camera topic and spawn point.
- **A run log** of where every drone went, packed for hand-in when the run ends.

## Documentation

- **[`docs/MANUAL.md`](docs/MANUAL.md)**: the participant manual. Installation,
  quickstart, the world, connecting your software, the camera, run logs, rules,
  troubleshooting.
- **[`docs/CUSTOM_DRONES.md`](docs/CUSTOM_DRONES.md)**: deploying your own drones in the
  environment, step by step.

## Requirements

Ubuntu 22.04, ROS 2 Humble, Gazebo Harmonic and ArduPilot Copter SITL. See
[the manual](docs/MANUAL.md#2-install) for the exact steps.

## Licence

LGPL-3.0. See [`LICENSE`](LICENSE).

This repository bundles the [ArduPilot Gazebo plugin](https://github.com/ArduPilot/ardupilot_gazebo)
unmodified in [`gz_plugins/`](gz_plugins/), and third-party aircraft meshes.
See [`NOTICE.md`](NOTICE.md) for full attribution.
