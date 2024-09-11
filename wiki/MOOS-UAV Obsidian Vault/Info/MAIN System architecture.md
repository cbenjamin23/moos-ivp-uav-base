
# moos-ivp-uav


This repository consist of a submodule for #MAVSDK 


Clone the [repo](https://github.com/Scarter668/moos-ivp-uav.git) (with ssh) and the submodules:

	git clone git@github.com:Scarter668/moos-ivp-uav.git;
	cd moos-ivp-uav;
	git submodule update --init --recursive



## Install MOOS-IVP trunk

This will be moved to github eventually, but can currently be found [here](https://oceanai.mit.edu/ivpman/pmwiki/pmwiki.php?n=Lab.ClassSetup#sec_course_software)


Download via `svn`:

	svn co https://oceanai.mit.edu/svn/moos-ivp-aro/trunk moos-ivp
Checkout and update:

	cd moos-ivp;
	svn update

Then build moos:

	./build.sh

Verify that it is properly installed with:

	$ which MOOSDB
	/Users/you/moos-ivp/bin/MOOSDB
	$ which pHelmIvP 
	/Users/you/moos-ivp/bin/pHelmIvP

AND/OR run an example mission:
```
cd ./ivp/missions/s1_alpha;
pAntler --MOOSTimeWarp=10 alpha.moos
```



---




# Other installed Repositories:

## ArduPilot (to run arduplane) with MavProxy
- ~/ardupilot/
- [Install guide](https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux)
	- **CHECKOUT** version `Plane-4.5` (branch)
- [More info](ArduPilot%20&%20MavProx.md)
## ardupilot_gazebo (*gazebo Harmonic*)
- Install the [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install_ubuntu/) simulator binary:
	- Test that it runs `gz sim -v4 -r shapes.sdf`
- In folder: ~/gz_ws/src/ardupilot_gazebo/
- [Install guide](https://ardupilot.org/dev/docs/sitl-with-gazebo.html)
## MissionPLanner
- ~/Mission_Planner/
- [Install guide](https://ardupilot.org/planner/docs/mission-planner-installation.html)
- [[Mission Planner info]]
## MAVSDK
- ~/MAVSDK/
- [[Install, Build & Run MAVSDK]]



---
Relevant topics: [[Troubleshooting]]
