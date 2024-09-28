
# moos-ivp-uav


This repository consist of a submodule for #MAVSDK 


Clone the [repo](https://github.com/Scarter668/moos-ivp-uav.git) (with ssh) and the submodules:

```bash
git clone git@github.com:Scarter668/moos-ivp-uav.git;
cd moos-ivp-uav;
git submodule update --init --recursive
```


### Setup bash aliases

The file `moos-ivp-uav/scripts/setup_bash_aliases_moos.sh` defines and sets up some useful aliases. 
Make the file executable

```bash
chmod +x ~/moos-ivp-uav/scripts/setup_bash_aliases_moos.sh
```

and source it from your bashrc. 

## Install MOOS-IVP trunk

This will be moved to github eventually, but can currently be found [here](https://oceanai.mit.edu/ivpman/pmwiki/pmwiki.php?n=Lab.ClassSetup#sec_course_software)


Download via `svn`:

```bash
svn co https://oceanai.mit.edu/svn/moos-ivp-aro/trunk moos-ivp
```

Checkout and update:

```bash
cd moos-ivp;
svn update
```

Then build moos:

```bash
./build.sh
```

Verify that it is properly installed with:

```bash
which MOOSDB
which pHelmIvP 
```

It should produce:

		$ which MOOSDB
		/Users/you/moos-ivp/bin/MOOSDB
		$ which pHelmIvP 
		/Users/you/moos-ivp/bin/pHelmIvP

AND/OR run an example mission:
```
cd ./ivp/missions/s1_alpha;
pAntler --MOOSTimeWarp=10 alpha.moos
```

Add path to bin folder in moos-ivp-uav to bashrc: `PATH=$PATH:~/moos-ivp-uav/bin`

**NOTE**: When building moos-ivp, remember to use the `-m` flag to only build nongui apps: `.build.sh -m`


---




# Other installed Repositories:

## ArduPilot (to run arduplane) with MavProxy
- ~/ardupilot/
- [Install guide](https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux)
	- Run `git submodule update --init --recursive`
	- Run `install-prereqs-ubuntu.sh` before switching branch! More in [[Troubleshooting]]
	- **CHECKOUT** version `Plane-4.1.2` (branch)
- [More info](ArduPilot%20&%20MavProx.md)
## ardupilot_gazebo (*gazebo Harmonic*)
- Install the [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install_ubuntu/) simulator binary:
	- Test that it runs `gz sim -v4 -r shapes.sdf`
- In folder: ~/gz_ws/src/ardupilot_gazebo/
- [Install guide](https://ardupilot.org/dev/docs/sitl-with-gazebo.html)
- Install the SITL_Models from cloning repo into home
```bash 
git clone git@github.com:ArduPilot/SITL_Models.git
```
And add to bashrc:
```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:\
$HOME/SITL_Models/Gazebo/models:\
$HOME/SITL_Models/Gazebo/worlds
```


## MissionPLanner
- ~/Mission_Planner/
- [Install guide](https://ardupilot.org/planner/docs/mission-planner-installation.html)
- [[Mission Planner info]]
## MAVSDK 

**No need to install as it comes as a submodule with moos-ivp-uav)**
- ~/MAVSDK/
- [[Install, Build & Run MAVSDK]]



---
Relevant topics: [[Troubleshooting]]
