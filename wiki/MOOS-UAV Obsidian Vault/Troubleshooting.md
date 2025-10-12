


## SITL fails

Try to first clean the build.  From `~/ardupilot` run `./waf clean` and run again.

Run the command 


```bash
~/ardupilot/Tools/environment_install/install-prereqs-ubuntu.sh -y
```

This wil setup things again.

If it doesn't work and `sim_vehicle.sh` is not on the path, then add it manually to `.profile `file:

```bash
export PATH=/opt/gcc-arm-none-eabi-6-2017-q2-update/bin:$PATH export PATH=$HOME/ardupilot/Tools/autotest:$PATH
```




## Gazebo not running/ dying after start

There might be a gazebo server/client running. Identify the PID with the command

```bash
ps aux | grep gz
```
and then kill it using `kill <PID>`

For MissionPlanner: 

```bash
ps aux | grep mono
```
	
and then kill it using `kill <PID>`


# Libuavcan issues

Happens when compiling Ardupilot using `sim_vehicle.py`, eg.:

    cfg.srcnode.find_dir('modules/uavcan/libuavcan/include').abspath()
	AttributeError: 'NoneType' object has no attribute 'abspath

Try to init submodules:
```bash
git submodule update --init --recursive
```

If it is stuck cloning for a long time clone `libcanard.git` and `uavcan.git` manually and try again!
```bash
cd ardupilot;
git clone https://github.com/UAVCAN/libcanard.git modules/libcanard
git clone https://github.com/UAVCAN/uavcan.git modules/uavcan
```

After this it is **IMPORTANT** to modify the UAVCAN Library Code as described in [ArduPilot & MavProx](Info/ArduPilot%20&%20MavProx.md)


## How to search in files:

	Example from dune
	
		grep -ri “matrix” <path_to_dune_src>
		- ./Maneuver/CoverArea/Task.cpp: Math::Matrix m_rows; // etc




# Debugging Arduplane

If arduplane crashes and you need to debug, here are some steps you can follow:

Run the start simulator script  with -gdb flag to debug it too: `./start_simulation.sh --gdb` 

Run the script: `gdb_attach_arduplane.sh` in `scripts/` after which you can enter `continue`  / `c` from the gdb to continue to run the program.


**Note**: 
- Arduplane runs from `~/ardupilot/build/sitl/bin` by default. If not, this needs to be changed in the script
- Starting the simulation should be done *First*
- Arduplane processes command with functions defined here `~/ardupilot/libraries/GCS_MAVLink/GCS_Common.cpp`
- To debug pArduBridge  add the alias:
  
```bash
alias launchSkywalker_gdb='cd ~/moos-ivp-uav/missions/UAV_Fly;
                       gdb --args env MAVSDK_CALLBACK_DEBUGGING=1 MAVSDK_COMMAND_DEBUGGING=1 MAVSDK_PARAMETER_DEBUGGING=1 pArduBridge targ_skywalker.moos;'
```


# MOOS IVP SWARM

### Fails building

Ensure that the MOOSIVP_SOURCE_TREE_BASE in the `~/moos-ivp-swarm/CMakeLists.txt` is properly set by commenting in the appropriate `PATHS` varible. It should look like this:

```cmake
find_path( MOOSIVP_SOURCE_TREE_BASE
			NAMES build-ivp.sh build-moos.sh configure-ivp.sh
			PATHS "../moos-ivp" "../../moos-ivp" "../../moos-ivp/trunk/" "../moos-ivp/trunk/"
			# PATHS "../../moos-ivp-git" "../moos-ivp-git/"
			DOC "Base directory of the MOOS-IvP source tree"
			NO_DEFAULT_PATH
)
```

Edit the `string2NodeRecord(report, true)` to `string2NodeRecord(report)` to make it compatible with the new function definitions

If you experience linker issues with `geodaid`, comment out the library.




# VSCode not showing included MOOS files

If you get red squiggles under you include files in VScode. Intellisense is not properly set up.
Solution: 
```bash
cd ~/moos-ivp
./build.sh
cd build/MOOS/MOOSCore
sudo make install
```
This will install the header folder/structure to your `/usr/local/include` folder and make it accessible for Vscode intellisense to read from

# Changing IP on Odroid XU4

Navigate to the folder `/etc/netplan/` and change the `.yaml` file  configuration with the updated IP

```bash
sudo nano /etc/netplan/<file_name>.yaml
```
change the IP to the desired one. <file_name> is for instance`ntnu-x8-005`
to some IP eg: `10.0.60.115`

then run 
```bash
sudo netplan apply
```
To verify what connection configuration you have, run:

```bash
nmcli connection show
```

# Selecting right port on Cube Orange Autopilot

For UART serial communication with the Cube Orange ArduPilot and the Odroid XU4, the GPS2 is the serial 4 port in MissionPlanner's parameter list.  It is defined as UART4  [here](https://docs.cubepilot.org/user-guides/autopilot/the-cube-module-overview) and more spesific [here](https://ardupilot.org/plane/docs/common-serial-options.html)
