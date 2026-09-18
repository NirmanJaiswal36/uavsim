# Third-party notices

`uavsim` is distributed under the LGPL-3.0 (see [`LICENSE.md`](LICENSE.md)). It bundles
the following third-party work.

## ArduPilot Gazebo plugins — `gz_plugins/`

The contents of `gz_plugins/` are a **verbatim, unmodified copy** of
[ArduPilot/ardupilot_gazebo](https://github.com/ArduPilot/ardupilot_gazebo)
at commit `082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5`.

- Copyright (C) ArduPilot Development Team, Rhys Mainwaring and contributors
- Licence: LGPL-3.0 (`gz_plugins/LICENSE.md`)

This provides `ArduPilotPlugin`, `ParachutePlugin`, `CameraZoomPlugin` and
`GstCameraPlugin`. Nothing in that directory has been changed; it is vendored so that
this repository builds from a single clone.

## Aircraft and scenery models — `models/`

| model | origin | authors |
|---|---|---|
| `iris_with_standoffs` | [RotorS](https://github.com/ethz-asl/rotors_simulator) via ardupilot_gazebo | Fadri Furrer (ETH Zurich), John Hsu, Nate Koenig (OSRF) |
| `gimbal_small_3d` | ardupilot_gazebo | ArduPilot Development Team |
| `zephyr` | ardupilot_gazebo / Gazebo model database | Nate Koenig, Cole Biesemeyer |
| `grasspatch_visual` | derived from a MOV.AI grass model | Luis Pinto (MOV.AI) |

Each model directory keeps its original `model.config` with the upstream author and
licence metadata. Refer to those files for the authoritative attribution.

## Parameter files — `config/`

`config/gazebo-iris-gimbal.parm` and `config/gazebo-zephyr.parm` are ArduPilot
parameter sets. The values follow ArduPilot's own SITL defaults
(`Tools/autotest/default_params/`), adapted for the models in this repository.

## Runtime assets fetched at first use

Tree models (`Oak Tree`, `Pine Tree`) are downloaded from
[Gazebo Fuel](https://app.gazebosim.org/fuel) the first time a world is loaded and
cached in `~/.gz/fuel`. They are not redistributed here; their licences are those
stated on Fuel.
