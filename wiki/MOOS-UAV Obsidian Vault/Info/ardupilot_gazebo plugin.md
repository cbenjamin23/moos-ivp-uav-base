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




pseudo code:

config_file= ..../....            #.yaml
default_models_folder = ..../.
default_worlds_file = ..../.
destination_models_folder = ..../....
destination_worlds_folder = ..../....


In the config_file, a number of vehicle is defined to be used with different names and configurations. This should be read into params.

define the destination world name based on the number of vehicles.

if the destination_worlds_folder cointains the destination world name, exit. 

for every vehicle in the defined to be used:

if params.model_name already exist in destination_models_folder, skip this step and continue.

else:
copy the folder of skywalker_x8 (default_models_folder) to the destination folder with a new name given by the params.model_name.

create an sdf_content that will incoperate some of the parameters .
Save this to destination_models_folder/params.model_name/model.config.

create another sdf_content that will incoperate some of the other parameters .
Save this to destination_models_folder/params.model_name/model.sdf.

end if;

end for;

copy the default_worlds_file to the destination_worlds_folder.

create another sdf_content that will incoperate some of the other parameters .
In this case, we will loop through the drone models and add them to the world sdf string before saving. 

Save this to destination_worlds_folder/ with the destination file name.



