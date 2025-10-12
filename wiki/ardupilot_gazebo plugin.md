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
