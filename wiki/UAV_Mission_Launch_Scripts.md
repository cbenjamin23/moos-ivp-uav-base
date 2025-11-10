# UAV Mission Launch Scripts Guide

This guide explains the launch scripts used in UAV missions and how they relate to the YAML configuration file.

## Table of Contents
- [Overview](#overview)
- [Launch Script Structure](#launch-script-structure)
- [Main Launch Script (launch.sh)](#main-launch-script-launchsh)
- [Vehicle Launch Script (launch_vehicle.sh)](#vehicle-launch-script-launch_vehiclesh)
- [Shoreside Launch Script (launch_shoreside.sh)](#shoreside-launch-script-launch_sidesidesh)
- [Comparison with Standard MOOS-IvP](#comparison-with-standard-moos-ivp)

---

## Overview

UAV missions use a multi-script launch system that:
1. Reads configuration from `missionConfig.yaml`
2. Launches multiple vehicle instances with auto-configured ports
3. Launches a ground control station (shoreside)
4. Supports both simulation and field deployment modes

**Key Scripts:**
- `launch.sh` - Main entry point, orchestrates vehicle and shoreside launches
- `launch_vehicle.sh` - Launches a single UAV instance
- `launch_shoreside.sh` - Launches the ground control station

---

## Launch Script Structure

### Dependencies

All launch scripts depend on helper functions:
```bash
source ~/moos-ivp-uav-base/scripts/configfileHelperFunctions.sh
```

This provides functions like:
- `get_global_val` - Extract values from YAML
- `get_val_by_drone_name` - Get drone-specific values
- `get_region_xy` - Convert region coordinates

### YAML Parsing

Scripts use `yq` (YAML query tool) to extract configuration:
```bash
NUM_VEHICLES=$(yq eval '.simulation.number_of_drones' "$CONFIG_FILE")
TIME_WARP=$(yq eval ".simulation.time_warp" "$CONFIG_FILE")
VNAME=$(yq eval ".drones[$i].name" "$CONFIG_FILE")
```

---

## Main Launch Script (launch.sh)

**Location:** `missions/UAV_Fly/launch.sh`

### Purpose
Orchestrates the entire mission launch by:
1. Reading mission configuration from YAML
2. Launching all vehicles sequentially
3. Launching the shoreside/GCS
4. Starting uMAC for mission control

### Usage

```bash
# Launch with defaults (reads missionConfig.yaml)
./launch.sh

# Launch with time warp
./launch.sh 10

# Launch with custom config
./launch.sh --config=myconfig.yaml

# Just generate files without launching
./launch.sh --just_make

# Verbose output
./launch.sh -v
```

### Command-Line Arguments

| Argument | Description | Example |
|----------|-------------|---------|
| `--help`, `-h` | Show help message | `./launch.sh -h` |
| `--verbose`, `-v` | Enable verbose output | `./launch.sh -v` |
| `--just_make`, `-j` | Generate .moos files without launching | `./launch.sh -j` |
| `--config=FILE`, `-c=FILE` | Use custom config file | `./launch.sh -c=test.yaml` |
| `[number]` | Set time warp | `./launch.sh 10` |

### Configuration Flow

```
missionConfig.yaml
    ↓
[Read global parameters]
├─ simulation.number_of_drones → NUM_VEHICLES
├─ simulation.time_warp → TIME_WARP
├─ simulation.ardupilot_ip → ARDUPILOT_IP
├─ simulation.ardupilot_protocol → ARDUPILOT_PROTOCOL
├─ simulation.ardupilot_port_default → ARDUPILOT_PORT (base)
├─ moos.defaultPorts.DB → DEFAULT_PORT_DB (base)
└─ moos.defaultPorts.PSHARE → DEFAULT_PORT_PSHARE (base)
    ↓
[For each drone in drones list]
├─ drones[i].name → VNAME
├─ drones[i].color → COLOR
├─ drones[i].start_orientaton_moos.x → x
├─ drones[i].start_orientaton_moos.y → y
├─ drones[i].start_orientaton_moos.hdg → heading
├─ MOOS_PORT = DEFAULT_PORT_DB + i + 1
├─ PSHARE_PORT = DEFAULT_PORT_PSHARE + i + 1
└─ ARDUPILOT_PORT = ardupilot_port_default + i*10
    ↓
[Launch each vehicle]
./launch_vehicle.sh [arguments]
    ↓
[Launch shoreside]
./launch_shoreside.sh [arguments]
    ↓
[Start mission control]
uMAC targ_shoreside.moos
```

### Script Sections

#### Part 1: Function Definitions and Helpers
```bash
vecho() { if [ "$VERBOSE" != "" ]; then echo "$ME: $1"; fi }
source ~/moos-ivp-uav-base/scripts/configfileHelperFunctions.sh
```

#### Part 2: Global Variables
```bash
TIME_WARP=1
JUST_MAKE=""
VERBOSE=""
VLAUNCH_ARGS="--auto --sim"
SLAUNCH_ARGS="--auto --sim"
CONFIG_FILE="./missionConfig.yaml"
NUM_VEHICLES=1
```

#### Part 3: Command-Line Argument Processing
Parses command-line options and overrides defaults.

#### Part 4: Configuration Reading
Extracts values from YAML using `yq`.

#### Part 5: Vehicle Launch Loop
```bash
for ((i = 0; i < $NUM_VEHICLES; i++)); do
    VNAME=$(yq eval ".drones[$i].name" "$CONFIG_FILE")
    MOOS_PORT=$(($i+1 + $DEFAULT_PORT_DB))
    PSHARE_PORT=$(($i+1 + $DEFAULT_PORT_PSHARE))
    ARDUPILOT_PORT=$(($ARDUPILOT_PORT + $i*10))
    
    ./launch_vehicle.sh --vname=$VNAME --mport=$MOOS_PORT \
                        --pshare=$PSHARE_PORT --ap_port=$ARDUPILOT_PORT \
                        --color=$COLOR --start=$START_POS \
                        $TIME_WARP $VERBOSE $JUST_MAKE
done
```

#### Part 6: Shoreside Launch
```bash
./launch_shoreside.sh $SLAUNCH_ARGS --vnames=${VNAMES} $VERBOSE $TIME_WARP
```

#### Part 7: Mission Control
```bash
uMAC targ_shoreside.moos  # Interactive mission control
kill -- -$$               # Kill all child processes on exit
```

### YAML to Launch Parameter Mapping

| YAML Path | Launch Script Variable | Target Script |
|-----------|------------------------|---------------|
| `simulation.number_of_drones` | `NUM_VEHICLES` | launch.sh |
| `simulation.time_warp` | `TIME_WARP` | All scripts |
| `simulation.ardupilot_ip` | `ARDUPILOT_IP` | launch_vehicle.sh |
| `simulation.ardupilot_port_default` | `ARDUPILOT_PORT` (base) | launch_vehicle.sh |
| `drones[i].name` | `VNAME` | launch_vehicle.sh |
| `drones[i].color` | `COLOR` | launch_vehicle.sh |
| `drones[i].start_orientaton_moos.x` | `START_POS` (x component) | launch_vehicle.sh |
| `drones[i].start_orientaton_moos.y` | `START_POS` (y component) | launch_vehicle.sh |
| `moos.defaultPorts.DB` | `DEFAULT_PORT_DB` (base) | launch.sh |
| `moos.defaultPorts.PSHARE` | `DEFAULT_PORT_PSHARE` (base) | launch.sh |

**Port Calculation:**
- Vehicle MOOSDB port: `DEFAULT_PORT_DB + vehicle_index + 1`
- Vehicle pShare port: `DEFAULT_PORT_PSHARE + vehicle_index + 1`
- ArduPilot MAVLink port: `ardupilot_port_default + vehicle_index * 10`

Example:
```
Default DB port: 9000, Default pShare: 9200, Default ArduPilot: 14550
Vehicle 0 (skywalker):     DB=9001, pShare=9201, ArduPilot=14550
Vehicle 1 (skyfollower):   DB=9002, pShare=9202, ArduPilot=14560
Vehicle 2 (skytrailer):    DB=9003, pShare=9203, ArduPilot=14570
```

---

## Vehicle Launch Script (launch_vehicle.sh)

**Location:** `missions/UAV_Fly/launch_vehicle.sh`

### Purpose
Launches a single UAV instance with:
- MOOSDB
- pHelmIvP (behavior controller)
- pArduBridge (ArduPilot interface) OR uSimMarineV22 (MOOS simulator)
- Supporting processes (pShare, uFldNodeBroker, etc.)

### Usage

```bash
# Launch with defaults from config
./launch_vehicle.sh --id=0 --sim

# Launch with specific parameters
./launch_vehicle.sh --vname=skywalker --mport=9001 --pshare=9201 \
                    --shore=10.0.60.1 --shore_pshare=9200 \
                    --ap_ip=0.0.0.0 --ap_port=14550 --sim

# Field deployment
./launch_vehicle.sh --id=0 --ap_ip=ttySAC0 --ap_port=115200 --ap_protocol=serial
```

### Key Arguments from YAML

| Argument | YAML Source | Description |
|----------|-------------|-------------|
| `--vname` | `drones[i].name` | Vehicle name |
| `--color` | `drones[i].color` | Display color |
| `--start` | `drones[i].start_orientaton_moos.x,y` | Starting position |
| `--mport` | Calculated: `DB + i + 1` | MOOSDB port |
| `--pshare` | Calculated: `PSHARE + i + 1` | pShare port |
| `--ap_ip` | `simulation.ardupilot_ip` | ArduPilot IP |
| `--ap_port` | Calculated: `port_default + i*10` | ArduPilot port |
| `--ap_protocol` | `simulation.ardupilot_protocol` | Connection protocol |
| `--shore` | `moos.shore_ip` | Shoreside IP |
| `--shore_pshare` | `moos.defaultPorts.PSHARE` | Shoreside pShare port |

### Configuration Variables Set

The script reads many additional parameters from YAML:

```bash
# From missionConfig.yaml
MAXSPD=$(get_global_val $CONFIG_FILE field.speed.max)
MINSPD=$(get_global_val $CONFIG_FILE field.speed.min)
CAPTURE_RADIUS=$(get_global_val_in_moosDistance $CONFIG_FILE "missionParams.capture_radius")
SLIP_RADIUS=$(get_global_val_in_moosDistance $CONFIG_FILE "missionParams.slip_radius")
ENCOUNTER_RADIUS=$(get_global_val_in_moosDistance $CONFIG_FILE "missionParams.encounter_radius")
REGION=$(get_region_xy $CONFIG_FILE)
VORONOI_SETPT_METHOD=$(get_global_val $CONFIG_FILE missionParams.voronoi_setpoint_method)
PLANNER_MODE=<determined from voronoi_search_enabled>
```

These are then inserted into the `.moos` file as preprocessor variables using `nsplug`:

```bash
nsplug meta_vehicle.moos targ_$VNAME.moos -f \
    VNAME=$VNAME \
    COLOR=$COLOR \
    REGION=$REGION \
    CAPTURE_RADIUS=$CAPTURE_RADIUS \
    ...
```

---

## Shoreside Launch Script (launch_shoreside.sh)

**Location:** `missions/UAV_Fly/launch_shoreside.sh`

### Purpose
Launches the ground control station with:
- MOOSDB (shoreside database)
- pMarineViewer (visualization and control GUI)
- uFldShoreBroker (vehicle communication broker)
- pGridSearchPlanner, uFldFireSim, etc. (mission-specific processes)

### Usage

```bash
# Launch with defaults
./launch_shoreside.sh --sim --vnames=skywalker:skyfollower:skytrailer

# Field deployment
./launch_shoreside.sh --ip=10.0.60.1 --pshare=9200 --vnames=all:skywalker
```

### Key Arguments from YAML

| Argument | YAML Source | Description |
|----------|-------------|-------------|
| `--ip` | `moos.shore_ip` | Shoreside IP address |
| `--pshare` | `moos.defaultPorts.PSHARE` | Shoreside pShare port |
| `--vnames` | Constructed from `drones[].name` | Colon-separated vehicle names |

### Configuration Variables Set

Many mission parameters are passed to the shoreside:

```bash
REGION=$(get_region_xy $CONFIG_FILE)
FIRE_FILE=$(get_global_val $CONFIG_FILE missionParams.fire_file_default)
MISSION_DURATION=$(get_global_val $CONFIG_FILE missionParams.mission_duration)
GRID_CELL_SIZE=$(get_global_val $CONFIG_FILE missionParams.grid_cell_size)
# ... many more parameters
```

These configure mission-specific processes like:
- `pGridSearchPlanner` - Mission planning
- `uFldFireSim` - Fire/target simulation
- `pMissionOperator` - Automated mission management

---

## File Generation Process

### Using nsplug (Meta-file Preprocessor)

UAV missions use `nsplug` to generate final .moos files from templates:

```bash
nsplug meta_vehicle.moos targ_$VNAME.moos -f \
    VNAME=$VNAME \
    START_POS=$START_POS \
    REGION=$REGION \
    # ... more variables
```

**In meta_vehicle.moos:**
```moos
ServerPort = $(MOOS_PORT)
Community  = $(VNAME)

ProcessConfig = pProxonoi_uav
{
  region = $(REGION)
  vcolor = $(COLOR)
}
```

**Generated targ_skywalker.moos:**
```moos
ServerPort = 9001
Community  = skywalker

ProcessConfig = pProxonoi_uav
{
  region = 100,460:-700,-200:-700,-920:620,-920:700,-222
  vcolor = orange
}
```

### Conditional Compilation

Meta-files support conditional blocks:

```moos
#ifdef USE_MOOS_SIM_PID true
  Run = uSimMarineV22     @ NewConsole = false
  Run = pMarinePIDV22     @ NewConsole = false
#else
  Run = pArduBridge       @ NewConsole = true
#endif
```

This is set in launch scripts:
```bash
if [ "$USE_MOOS_SIM_PID" = "yes" ]; then
    NSFLAGS+=" USE_MOOS_SIM_PID=$USE_MOOS_SIM_PID"
fi
```

---

## Example: Complete Launch Sequence

### 1. User Command
```bash
cd missions/UAV_Fly
./launch.sh 10
```

### 2. launch.sh Reads YAML
```yaml
simulation:
  number_of_drones: 3
  time_warp: 10
drones:
  - name: "skywalker"
    color: "orange"
    start_orientaton_moos: {x: 0, y: 0, hdg: 46}
  - name: "skyfollower"
    # ...
```

### 3. launch.sh Calls launch_vehicle.sh (3 times)
```bash
# Vehicle 0
./launch_vehicle.sh --vname=skywalker --mport=9001 --pshare=9201 \
    --ap_port=14550 --start=0,0 --color=orange 10

# Vehicle 1
./launch_vehicle.sh --vname=skyfollower --mport=9002 --pshare=9202 \
    --ap_port=14560 --start=-200,-10 --color=hotpink 10

# Vehicle 2
./launch_vehicle.sh --vname=skytrailer --mport=9003 --pshare=9203 \
    --ap_port=14570 --start=200,-10 --color=purple 10
```

### 4. Each launch_vehicle.sh:
- Reads additional config from YAML
- Generates targ_$VNAME.moos using nsplug
- Generates targ_$VNAME.bhv using nsplug
- Launches pAntler with generated .moos file

### 5. launch.sh Calls launch_shoreside.sh
```bash
./launch_shoreside.sh --sim --vnames=skywalker:skyfollower:skytrailer 10
```

### 6. launch_shoreside.sh:
- Generates targ_shoreside.moos
- Launches pAntler for shoreside
- Starts pMarineViewer (GUI)

### 7. launch.sh Starts uMAC
```bash
uMAC targ_shoreside.moos
```

User can now interact with the mission via pMarineViewer or uMAC commands.

---

## Related Documentation

- [UAV Mission Configuration Guide](UAV_Mission_Configuration.md) - Detailed YAML reference
- [ARDU Commands Reference](UAV_Mission_ARDU_Commands.md) - Command definitions and usage
- [System Launch Guide](System_Launch_Guide.md) - Step-by-step procedures
