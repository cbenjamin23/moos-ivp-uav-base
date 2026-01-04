# ArduPilot Gazebo Plugin Setup

> **Note:** This documentation is only relevant if you're using the **ArduPilot SITL + Gazebo** simulation approach (`useMoosSimPid: false`). If you're using the lightweight MOOS-IvP Simulator approach with pMarineViewer, you don't need Gazebo. See [Installation & Setup](Installation_&_Setup.md) for more information.

## About Gazebo

This project uses the newer **Gazebo Sim 8** (Ionic version, often referred to as Gazebo 11 or later versions) rather than Gazebo Classic (usually referring to earlier versions like Gazebo 9 and 10).

For more information on Gazebo environmental variables, see: https://answers.gazebosim.org//question/29153/some-questions-about-uri-in-sdf-files/

## Plugin Modification

When the plugin is downloaded modify it to make it compatible with gazebo ionic by adding to the file `ardupilot_gazebo/CMakeLists.txt`

```bash
# Ionic
elseif("$ENV{GZ_VERSION}" STREQUAL "ionic" OR NOT DEFINED "ENV{GZ_VERSION}")
find_package(gz-cmake3 REQUIRED)
set(GZ_CMAKE_VER ${gz-cmake3_VERSION_MAJOR})
gz_find_package(gz-common5 REQUIRED)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

gz_find_package(gz-rendering9 REQUIRED)
set(GZ_RENDERING_VER ${gz-rendering9_VERSION_MAJOR})
gz_find_package(gz-sim9 REQUIRED)
set(GZ_SIM_VER ${gz-sim9_VERSION_MAJOR})
message(STATUS "Compiling against Gazebo Ionic")
```
to the test case `if("$ENV{GZ_VERSION}" STREQUAL "harmonic" OR NOT DEFINED "ENV{GZ_VERSION}")`

**Note:** also remove `OR NOT DEFINED "ENV{GZ_VERSION}"` to the first if case to make Gazebo Ionic default.







build it before use:

```bash
export GZ_VERSION=ionic
cd ardupilot_gazebo
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
```

Configure the environment:

```bash
echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/gz_ws/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}' >> ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/gz_ws/ardupilot_gazebo/models:$HOME/gz_ws/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc
```

*Important:* Verify the path to the folder

The rest of the installment can be found in the [Install guide](https://ardupilot.org/dev/docs/sitl-with-gazebo.html)
