

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

#### Install with moos

Read instructions [[MAIN System architecture]]

### Building the library

[Official guide](https://mavsdk.mavlink.io/main/en/cpp/guide/build.html)

- *Configuration step*
		- In folder ~/MAVSDK/ run:
	
					cmake -DCMAKE_BUILD_TYPE=Debug -Bbuild/default -H. -DBUILD_SHARED_LIBS=ON 
	
	- If not restricted by memory/compute consider including:
				
					-DASAN=ON -DUBSAN=ON -DLSAN=ON
	- *Build step*
	
			cmake --build build/default -j16

		- Install system wide
			
				sudo cmake --build build/default --target install


Summary: 
```
cd ~/MAVSDK;
cmake -DCMAKE_BUILD_TYPE=Debug -Bbuild/default -H. -DBUILD_SHARED_LIBS=ON -DSUPERBUILD=ON;
sudo cmake --build build/default --target install;
cd -
```

Flag info can be found in the official guide

### Building an example/app

This should be called in the folder where the .cpp`.cpp` file is defined (in MAVSDK)
```
cd examples/takeoff_and_land/
cmake -Bbuild -H.
cmake --build build -j8
```


Summary:
```
cmake --build build -j8;
MAVSDK_CALLBACK_DEBUGGING=1 MAVSDK_COMMAND_DEBUGGING=1 MAVSDK_PARAMETER_DEBUGGING=1 ./build/<app> <args> 
```

where `<app>` is you MAVSDK app, eg. `fly_mission_ex`, `<arg>` is the argument of the application, eg. `udp://:14550`. To redirect the output to a file append the following `> output.log 2>&1`.

*EX:*

```
cmake --build build -j8; 
MAVSDK_CALLBACK_DEBUGGING=1 MAVSDK_COMMAND_DEBUGGING=1 MAVSDK_PARAMETER_DEBUGGING=1 ./build/fly_mission_ex udp://:14550 > output.log 2>&1
```






