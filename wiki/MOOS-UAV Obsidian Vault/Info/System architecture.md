
Installed Repositories:

# ArduPilot (to run arduplane) with MavProxy
- ~/ardupilot/
- [Install guide](https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux)
- [More info](ArduPilot%20&%20MavProx.md)
# ardupilot_gazebo (*gazebo Harmonic*)
-  ~/gz_ws/src/ardupilot_gazebo/
- [Install guide](https://ardupilot.org/dev/docs/sitl-with-gazebo.html)
# MissionPLanner
- ~/Mission_Planner/
- [Install guide](https://ardupilot.org/planner/docs/mission-planner-installation.html)
# MAVSDK
- ~/MAVSDK/
- [Install guide](https://mavsdk.mavlink.io/main/en/cpp/quickstart.html)

### Building the [library](https://mavsdk.mavlink.io/main/en/cpp/guide/build.html)
- *Configuration step*
		- In folder ~/MAVSDK/ run:
	
					cmake -DCMAKE_BUILD_TYPE=Debug -Bbuild/default -H. -DBUILD_SHARED_LIBS=ON 
	
	- Too little memory to include:
				
					-DASAN=ON -DUBSAN=ON -DLSAN=ON
	- *Build step*
	
			cmake --build build/default -j16

		- Install system wide
			
				sudo cmake --build build/default --target install

### Building an example/app


		cd examples/takeoff_and_land/
		cmake -Bbuild -H.
		cmake --build build -j8






Not use moos at the time


- Can run on raspy and have a full autonomy system
- very lightweight
- collision avoidence,
- object avoidence
- path allocations
- voronoi apps to locate 
- swarmtoolbox laltes release in moos
- CMU UAV researchgroup - # Sebastian Scherer from CMU



