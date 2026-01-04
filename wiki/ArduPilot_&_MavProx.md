
# ArduPilot & MavProxy

> **Note:** This documentation is relevant only if you're using the **ArduPilot SITL + Gazebo** simulation approach (`useMoosSimPid: false`) or working with physical hardware. If you're using the lightweight MOOS-IvP Simulator approach (`useMoosSimPid: true`), you don't need ArduPilot or Gazebo. See [Installation & Setup](Installation_&_Setup.md) for more information about the two simulation approaches.

# ArduPilot

## Version Compatibility

This project was tested with **ArduPilot Plane version 4.6.3**, but should be forward-compatible. The specific parameter list for Plane-4.6.3 can be found [here](https://ardupilot.org/plane/docs/parameters-Plane-stable-V4.6.3.html) (web) or [here](https://autotest.ardupilot.org/Parameters/ArduPlane/apm.pdef.xml) (xml).

**Important Note:** Some parameter names can and have changed from version to version. Before using a different plane version, verify that the following required parameters are available and have the same names:

- `AIRSPEED_CRUISE`
- `AIRSPEED_MAX`
- `AIRSPEED_MIN`

If these parameters have different names in your version, you will need to update the configuration accordingly.

For all parameters across all vehicles, look [here](https://ardupilot.org/dev/docs/mavlink-get-set-params.html), which also provides information about the Mavlink Interface with ArduPilot



The ArduPilot spesific commands can be found  [here](https://mavlink.io/en/messages/ardupilotmega.html#enumerated-types) (web) or in [xml](https://github.com/mavlink/mavlink/blob/master/message_definitions/v1.0/ardupilotmega.xml) version.




Airspeed in simulation is set to the following values: 
```
AIRSPEED_CRUISE 15
AIRSPEED_MAX 19
AIRSPEED_MIN 11
```

## Launching the Simulation

### Using start_simulation.sh Script

The simulation can be launched using the `scripts/start_simulation.sh` script. This script provides a convenient way to start the ArduPilot SITL (Software In The Loop) simulation along with Gazebo for visualization.

**Basic Usage:**
```bash
cd ~/moos-ivp-uav-base/scripts
./start_simulation.sh
```

**What the script does:**

The script reads configuration from the `missionConfig.yaml` file (located in `missions/UAV_Fly/`) and:

1. **Generates drone SDF files** from the configuration using `generate_drone_sdf.py`
2. **Starts Gazebo** with the appropriate number of vehicles based on the configuration
3. **Launches ArduPilot SITL instances** for each drone in a tmux session named `drone_sim`
4. Sets up each drone with:
   - Unique system ID
   - Custom starting location (lat/lon/altitude/heading)
   - Simulation speed/time warp
   - MAVLink connection ports

**Important Configuration: useMoosSimPid Parameter**

The behavior of the simulation depends on the `useMoosSimPid` parameter in `missionConfig.yaml`:

- **When `useMoosSimPid: false`**: 
  - The script launches ArduPilot SITL instances
  - Gazebo is started to connect to the ArduPilot instances
  - pArduBridge will communicate with these ArduPilot instances
  - This provides full flight dynamics simulation through ArduPilot

- **When `useMoosSimPid: true`**:
  - ArduPilot SITL and Gazebo are NOT launched
  - The MOOS-IvP vehicle simulator is used instead
  - pArduBridge will not launch
  - This is a lighter-weight simulation option

**Script Options:**
```bash
./start_simulation.sh [OPTIONS]
  --gdb           : Start ArduPilot SITL in GDB mode (debugging)
  --mp            : Start Mission Planner
  --just_gz       : Start Gazebo only (no ArduPilot SITL)
  --parm=filename : Use specified parameter file for ArduPilot SITL
  --swarm=N       : Start Gazebo with N vehicles
  --config        : Use configuration file to generate drone SDF files (default)
  --force, -f     : Force overwrite of existing drone SDF files and restart Gazebo
  --clean         : Clean up simulation files
  --ks            : Kill existing tmux session and exit
  --nt            : Open new terminal windows for the simulation
  --verbose, -v   : Verbose output
```

### Using pMarineViewer

**pMarineViewer** is a MOOS-IvP application for visualizing vehicle missions and operations. It works similarly regardless of the `useMoosSimPid` setting:

- It connects to the MOOS database to visualize vehicle positions, paths, and mission data
- It provides a graphical interface to monitor drone positions and mission progress
- It can be used to send commands to the vehicles
- For detailed pMarineViewer usage, see the [pArduBridge documentation](pArduBridge/QUICKSTART.md)

**Launching pMarineViewer:**

pMarineViewer is typically launched as part of the shoreside/ground station:
```bash
cd ~/moos-ivp-uav-base/missions/UAV_Fly
./launch_shoreside.sh
```

#### SIM_OPOS parameters
```default
SIM_OPOS_ALT     106.099998
SIM_OPOS_HDG     45.000000
SIM_OPOS_LAT     63.397518
SIM_OPOS_LNG     10.143532
```
Defined in `sim.parm` but not used, as the location is given as a location parameters when calling sim_vehicles.py



---

If ardupilot does not  compile because of usage of python 3.10 or newer, fix the generated error by doing the following:

#### Modify the UAVCAN Library Code

Since the error is due to an outdated reference in the `uavcan` library, you can manually update the Python code in the `uavcan` library to reflect the correct import.

##### Open the file causing the error 

```bash
	code ~/ardupilot/modules/uavcan/libuavcan/dsdl_compiler/pyuavcan/uavcan/transport.py
```
	

 ##### Find the line:
    
    
```python
class ArrayValue(BaseValue, collections.MutableSequence):
```

##### Modify the class definition as follows:
    
    
```python
class ArrayValue(BaseValue, MutableSequence):
```


##### Change the import to use `collections.abc`:
    
    
```python
from collections.abc import MutableSequence
```
    
    

Now run simulator again!



## Setting the default coordinates when running simulation

Under the folder `scripts/simulation`, modify the specific parameters in `mav.parm` to desired values, eg:  
```bash
SIM_OPOS_ALT     106.300000
SIM_OPOS_HDG     45.000000
SIM_OPOS_LAT     63.3975168
SIM_OPOS_LNG     10.1435321
```

*Note*: if the file doesn't exit remember to run the simulation script once before. 


## Ensuring Simulation has the same parameters as hardware Ardupilot


The parameters on the hardware might be different from simulation, therefore causing  oscillation. 

Ensure that the tuning hardware params are loaded when simulating.

Prams copied:

ACRO_LOCKING
AHRS_TRIM_X/Y
ALT_CTRL_ALG
ALT_HOLD_RTL
ARSPD_FBW_MIN/MAX
LIM_PITCH_MAX/MIN
LIM_ROLL_CD

NAVL1_PERIOD `tunable` 
NAVL1_XTRACK_I  `tunable` 
PTCH_RATE_D  `tunable`
PTCH_RATE_FF  `tunable`
PTCH_RATE_**  `tunable`
PTCH_RATE_I  `tunable`
PTCH_RATE_P  `tunable`

```tunables
PTCH_RATE_D      0.003699
PTCH_RATE_FF     0.403978
PTCH_RATE_FLTD   10.000000
PTCH_RATE_FLTE   0.000000
PTCH_RATE_FLTT   2.122066
PTCH_RATE_I      0.403978
PTCH_RATE_IMAX   0.666000
PTCH_RATE_P      0.350000
PTCH_RATE_SMAX   150.000000
```

RLL_RATE_D `tunable`
RLL_RATE_FF `tunable`
RLL_RATE_FLTD `tunable`
RLL_RATE_FLTT `tunable`
RLL_RATE_I `tunable`
RLL_RATE_P `tunable`

```tunables
RLL_RATE_D       0.002845
RLL_RATE_FF      0.181866
RLL_RATE_FLTD    10.000000
RLL_RATE_FLTE    0.000000
RLL_RATE_FLTT    3.183099
RLL_RATE_I       0.181866
RLL_RATE_IMAX    0.666000
RLL_RATE_P       0.096396
RLL_RATE_SMAX    150.000000
```

TECS_PTCH_DAMP `tunable`


```tunables
TECS_PTCH_DAMP   0.900000
TECS_PTCH_FF_K   0.000000
TECS_PTCH_FF_V0  12.000000
TECS_RLL2THR     10.000000
TECS_SINK_MAX    5.000000
TECS_SINK_MIN    2.000000
TECS_SPDWEIGHT   1.000000
TECS_SPD_OMEGA   2.000000
TECS_SYNAIRSPEED 0
TECS_THR_DAMP    0.500000
TECS_TIME_CONST  5.000000
TECS_TKOFF_IGAIN 0.000000
TECS_VERT_ACC    7.000000
```

TRIM_ARSPD_CM
TRIM_THROTTLE 
WP_LOITER_RAD  `old value= 30`
WP_RADIUS  `old value= 30`
YAW2SRV_DAMP `tunable`
YAW2SRV_INT `tunable`

```tunables
YAW2SRV_DAMP     0.100000
YAW2SRV_INT      0.050000

YAW2SRV_RLL      1.000000
YAW2SRV_SLIP     0.000000
YAW2SRV_IMAX     1500
```

## Mission Planner Configuration

### Loading Generic Waypoint Missions

Load a generic waypoint mission from:

	~/ardupilot/Tools/autotest/Generic_Missions/

### PID Tuning Values

The PID values from Mission Planner are as follows (see image reference in original documentation - green indicates important changes from default values).

Also change under Servo Yaw:
	
	Dampening: 0.1 
	Integral: 0.05

For more details on tunable parameters, see the sections above on parameter configuration.


## Changing speed in Ardupilot SITL

speed change in simulation doesn't seem to work. This seems to be a known issue on the [forum](https://discuss.ardupilot.org/search?q=DO_CHANGE_SPEED%20not%20working).




# MavProxy

- UDP forwarding:
	After it has started use the command
	
		output add <destination_IP_address>:<Port>
	to connect to the IP above. Eg `120.0.0.1:14550` for localhost



## Changing the simulation speed[¶](https://ardupilot.org/dev/docs/sitl-with-gazebo.html#changing-the-simulation-speed "Link to this heading")



By default Gazebo will attempt to run the simulation with a Real Time Factor (RTF) of 1. To increase the simulation speed add the following XML block into the world file just after the opening *world* element in the file of the X8 model under `SITL_Models/Gazebo/worlds/skywalker_x8_runway.sdf`:

```sdf
<physics name="1ms" type="ignore">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>-1.0</real_time_factor>
</physics>
```

Then set the simulation speed-up parameter in MAVProxy

> MANUAL> param set SIM_SPEEDUP 1


### Adding wind in simulation
Modify the parameters  in mavproxy with `param set <param> <value>`
```example params
SIM_WIND_DELAY,0
SIM_WIND_DIR,180
SIM_WIND_DIR_Z,0
SIM_WIND_SPD,0
SIM_WIND_T,0
SIM_WIND_T_ALT,60
SIM_WIND_T_COEF,0.01
SIM_WIND_TURB,0
SIM_WOW_PIN,-1
```

## Swarm Capabilities in ArduPilot

### Multiple Vehicles with MavProxy

For detailed information on running multiple vehicles, refer to the [Multiple Vehicles with Mavproxy](https://ardupilot.org/mavproxy/docs/getting_started/multi.html#multi) documentation.

When launching the simulation, the following arguments are important for swarms:
- `--sysid=*` - Sets the system ID for each vehicle
- `-IX` (e.g. `I0`, `I1`, `I2`) - Indicates incrementation of ArduPilot's UDP connection use

For additional swarm setup instructions, see the [Intelligent Quads tutorial](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/swarming_ardupilot.md).
