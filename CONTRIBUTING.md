# Contributing

## Getting set up

```bash
pip install -r requirements.txt pytest
export GZ_VERSION=harmonic
cmake -B build -S gz_plugins && cmake --build build -j4
python3 -m pytest tests/ -q
```

The test suite needs neither Gazebo nor a SITL binary, so it is the fastest way to know
you have not broken anything.

## Where things live

| you want to change | edit |
|---|---|
| A vehicle type | `registry/vehicles/<id>.yaml` — one file, no code |
| An aircraft's physical description | `registry/airframes/<id>/airframe.yaml` |
| How an aircraft talks to an autopilot | `registry/airframes/<id>/bindings/<backend>.sdf.j2` |
| Autopilot process management | `registry/backends/<id>/adapter.py` |
| A sensor or actuator | `registry/payloads/<id>/` |
| The GCS layout | `uavsim/site.py` and `worlds/templates/gcs_site.sdf.j2` |
| The forest scenery | `worlds/fragments/roi_forest.sdf` |

See [`docs/architecture.md`](docs/architecture.md) and
[`docs/extending.md`](docs/extending.md).

## Rules that are not negotiable

- **Exactly one `<plane>` collision per world.** A plane collision is an infinite
  half-space in DART; a second one makes the ground-contact LCP singular and freezes the
  physics step. Use boxes or visual-only geometry for anything that sits on the ground.
  `tests/test_world.py` and `uavsim verify` both enforce this.
- **Do not edit `gz_plugins/`.** It is a verbatim copy of upstream ardupilot_gazebo.
  See [`gz_plugins/README.md`](gz_plugins/README.md).
- **No mission logic.** Arming, take-off, search patterns and planning belong to the
  framework being tested, not to this repository.
- **Adapters describe work, they do not perform it.** An adapter returns `ProcessSpec`s
  and `SpawnRequest`s; the supervisor and spawner run them. Keep `subprocess` out of
  `adapter.py`.
- **The core never branches on a backend id.** If you find yourself writing
  `if backend == "px4"` outside `registry/backends/px4/`, the capability schema is
  missing something — extend that instead.

## Tests

Add tests alongside the existing ones in `tests/`. Anything that can be checked without
Gazebo should be, because that is what CI runs.
