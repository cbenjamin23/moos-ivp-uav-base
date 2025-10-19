
# MAVSDK Setup & Usage (Optional - For Custom MAVSDK Development)

> **Note:** This guide is **optional** and only needed if you want to build your own custom MAVSDK applications to gain experience working with MAVSDK directly. The MAVSDK library is already included as a submodule in moos-ivp-uav-base and is built automatically with the main project. Most users can skip this guide and proceed directly to using the system.

## When to use this guide:
- You want to build standalone MAVSDK example applications for learning
- You need to develop custom MAVSDK applications outside of the moos-ivp-uav-base project
- You want to experiment with MAVSDK features independently

## When to skip this guide:
- You just want to use the moos-ivp-uav-base system (MAVSDK is already set up)
- You're following the standard installation process
- You don't plan to write custom MAVSDK code

---

### Install

#### Standalone install:

Clone from fork of official MAVSDK

**https:**

	git clone https://github.com/Scarter668/MAVSDK.git;
	git submodule update --init --recursive

**ssh**:

	git clone git@github.com:Scarter668/MAVSDK.git;
	git submodule update --init --recursive


**CHECKOUT** the version `v2.12.6-Ardupilot`
**NOTE:** MAVSDK only supports Mavlink 2 which is backwards compatible with Mavlink 1


#### Install with moos

Read instructions [Installation_&_Setup](Installation_&_Setup.md)

### Building the library

[Official guide](https://mavsdk.mavlink.io/main/en/cpp/guide/build.html)

- *Configuration step*
		- In folder `~/MAVSDK/`run:
	
	`cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DCMAKE_BUILD_TYPE=Debug -Bbuild/default -H. -DBUILD_SHARED_LIBS=ON` 
	
	The first flag `-DCMAKE_EXPORT_COMPILE_COMMANDS=1` is to generate a `compile_command.json` for IntelliSense Configuration in vscode 
	- If not restricted by memory/compute consider including:
	
			-DASAN=ON -DUBSAN=ON -DLSAN=ON
	- *Build step*
	
			cmake --build build/default -j16

		- Install system wide
			
				sudo cmake --build build/default --target install


Summary: 
```
cd <path-to>/MAVSDK;
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DCMAKE_BUILD_TYPE=Debug -Bbuild/default -H. -DBUILD_SHARED_LIBS=ON -DSUPERBUILD=ON;
sudo cmake --build build/default --target install;
cd -
```

Flag info can be found in the official guide



### Alias (Optional - For VSCode Development)

This alias is useful if you're developing MAVSDK applications in VSCode and need IntelliSense support. It builds apps in moos and updates the compile_commands from this repo and submodule MAVSDK for VSCode to locate the apps.

```
alias bldm='sudo .;
			bld;
			mavsdk_build_install;
			cd ~/moos-ivp-uav-base/scripts;
			./merge_compile_commands.sh;
			cd -'
```


### Building an example/app (Optional - For Learning MAVSDK)

Building and running MAVSDK example applications is useful for gaining hands-on experience with MAVSDK features and understanding how to interact with drones programmatically. This is **not required** for using moos-ivp-uav-base, but can help you understand the underlying MAVSDK library better.

This should be called in the folder where the .cpp`.cpp` file is defined (in MAVSDK):


Go to an example program: `cd examples/takeoff_and_land/`

```bash
cmake -Bbuild -H.;
cmake --build build -j8;
```

*Notice* the configure step `cmake -Bbuild -H.` that should be called first and only once

Summary:
```bash
cmake --build build -j8;
MAVSDK_CALLBACK_DEBUGGING=1 MAVSDK_COMMAND_DEBUGGING=1 MAVSDK_PARAMETER_DEBUGGING=1 ./build/<app> <args> 
```

where `<app>` is you MAVSDK app, eg. `fly_mission_ex`, `<arg>` is the argument of the application, eg. `udp://:14550`. To redirect the output to a file append the following `> output.log 2>&1`.

**Additional debug var:** *`MAVSDK_MESSAGE_HANDLER_DEBUGGING=1 `*

*EX:*

```bash
cmake --build build -j8; 
 MAVSDK_CALLBACK_DEBUGGING=1 MAVSDK_COMMAND_DEBUGGING=1 MAVSDK_PARAMETER_DEBUGGING=1 ./build/fly_mission_ex udp://:14550 > output.log 2>&1
```

or 
```bash
cmake --build build -j8;  MAVSDK_CALLBACK_DEBUGGING=1 MAVSDK_COMMAND_DEBUGGING=1 MAVSDK_PARAMETER_DEBUGGING=1 ./build/fly_mission_ex serial:///dev/ttySAC0:115200
```



