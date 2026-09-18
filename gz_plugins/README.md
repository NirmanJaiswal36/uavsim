# Vendored: ArduPilot Gazebo plugins

Everything in this directory is a **verbatim copy** of
[ArduPilot/ardupilot_gazebo](https://github.com/ArduPilot/ardupilot_gazebo)
at commit `082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5`, licensed LGPL-3.0.

**Do not edit these files.** Keeping them byte-identical to upstream is what makes the
licence story clean and lets the copy be refreshed with a straight `git diff` against
upstream. If you need different plugin behaviour, raise it upstream.

## Building

From the repository root:

```bash
export GZ_VERSION=harmonic        # or ionic / jetty / garden
cmake -B build -S gz_plugins
cmake --build build -j4
```

This writes `libArduPilotPlugin.so`, `libParachutePlugin.so`, `libCameraZoomPlugin.so`
and `libGstCameraPlugin.so` into `build/` at the repository root, which is where
`uavsim` looks for them.

## Note on colcon

`CMakeLists.txt` here is upstream's, so it declares the ament package `ardupilot_gazebo`
and its install rules expect upstream's `config/`, `models/` and `worlds/` directories
to sit beside it. They do not in this repository, so **building this directory with
colcon is not supported** — use the plain `cmake` invocation above. If you already have
upstream `ardupilot_gazebo` installed, you can skip building this entirely and point
`GZ_SIM_SYSTEM_PLUGIN_PATH` at your existing plugins.

## Refreshing from upstream

```bash
git clone https://github.com/ArduPilot/ardupilot_gazebo /tmp/apgz
rsync -a --delete /tmp/apgz/{src,include,cmake,hooks}/ gz_plugins/
cp /tmp/apgz/{CMakeLists.txt,package.xml,LICENSE.md} gz_plugins/
```

Then update the commit hash above and in [`../NOTICE.md`](../NOTICE.md).
