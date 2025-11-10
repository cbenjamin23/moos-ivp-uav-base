# UAV Mission Configuration Guide

This guide provides a comprehensive breakdown of the UAV mission configuration system in moos-ivp-uav-base.

## Table of Contents
- [Mission Configuration YAML File](#mission-configuration-yaml-file)
- [GCS Commands Reference](#gcs-commands-reference)
- [Launch Scripts](#launch-scripts)

---

## Mission Configuration YAML File

The `missionConfig.yaml` file is the central configuration for UAV missions. Below is a line-by-line explanation of each section.

### Simulation Parameters

```yaml
simulation:
  useMoosSimPid: true           # Use MOOS-IvP vehicle simulator (true) or ArduPilot SITL (false)
  time_warp: 10                 # Simulation speed multiplier (1 = real-time, 10 = 10x faster)
  number_of_drones: 3           # Number of UAVs in the mission
  
  ardupilot_ip: "0.0.0.0"       # IP address for ArduPilot SITL (0.0.0.0 = localhost)
  ardupilot_protocol: "udp"     # Communication protocol: "udp", "tcp", or "serial"
  fdm_port_in_default: 9002     # Base port for flight dynamics model input for Gazebo
  ardupilot_port_default: 14550 # Base port for ArduPilot MAVLink communication
                                # (incremented by 10 for each additional drone)
```

### Field Deployment Parameters

```yaml
field:
  home_altitude: 106.1          # Home altitude in meters (AGL)
  ardupilot_ip: "ttySAC0"       # Serial device path for field deployment
  ardupilot_port: "115200"      # Baud rate for serial communication
  ardupilot_protocol: "serial"  # Use serial protocol for field deployment
  speed:
    max: 19                     # Maximum UAV speed in m/s
    min: 11                     # Minimum UAV speed in m/s
```

### MOOS Configuration

```yaml
moos:
  datum:
    lat: 63.3975168             # Latitude of local datum (converts lat/lon to x/y)
    lon: 10.1435321             # Longitude of local datum
  
  # Ground Control Station (Shoreside) parameters
  shore_ip: 10.0.60.1           # IP address of the GCS/shoreside computer
  defaultPorts:
    DB: 9000                    # Base port for MOOSDB (incremented per vehicle)
    PSHARE: 9200                # Base port for pShare (MOOS inter-vehicle comms)
```

### Mission Parameters

```yaml
missionParams:
  log_enabled: false            # Enable/disable mission logging
  mission_duration: 600         # Maximum mission duration in seconds
  
  # Path Planning Algorithm Selection
  voronoi_search_enabled: false # false = use TMSTC*, true = use Voronoi
  voronoi_setpoint_method: gridsearch  # Method for Voronoi: "gridsearch", "center", "centroid"
  
  # TMSTC* Algorithm Parameters
  TMSTC_gridsizeSensorRangeRatio: 0.4  # Grid cell size as ratio of sensor range
  TMSTC_config_vmax: 18                 # Max speed for path planning (m/s)
  TMSTC_config_phi_max_rad: 45          # Max bank angle for turns (degrees)
  TMSTC_point_filtering: true           # Filter already-visited grid cells
  
  # Fire/Target Parameters
  fire_file_default: "fires_field.txt"  # File to load fire locations from
  fire_generate: true                   # Generate random fires (true) or load from file (false)
  fire_count: 40                        # Number of fires to place at mission start
  fire_sep_min: 10                      # Minimum separation between fires (meters)
  fire_color: red                       # Display color for fires
  fire_spawn_count: 0                   # Number of fires to spawn during mission
  fire_spawn_interval: 300:500          # Time range for spawning (min:max seconds)
  
  # Ignored Region Parameters (no-fly zones)
  ignoredRegion_file_default: "ignoredRegions.txt"
  ignoredRegion_generate: true          # Generate random regions or load from file
  ignoredRegion_count: 1                # Number of ignored regions
  ignoredRegion_sep_min: 150            # Minimum separation between regions (meters)
  ignoredRegion_spawn_count: 0          # Regions to spawn during mission
  ignoredRegion_spawn_interval: 300:500 # Time range for spawning
  
  # Mission Operator (for automated multi-mission runs)
  mission_operator_reset_delay: 2       # Delay between mission completion and reset (seconds)
  mission_operator_TMSTC_missions: 2    # Number of TMSTC* missions to run
  mission_operator_voronoi_missions: 0  # Number of Voronoi missions to run
  
  # Operating Region Definition
  use_moos_region: false        # Use MOOS XY region coordinates (true) or lat/lon region coordinates (false)
  region_XY: 100,460:-700,-200:-700,-920:620,-920:700,-222  # XY polygon vertices
  region_lonlat: |              # Lat/lon polygon vertices (used if use_moos_region=false)
    10.1356602, 63.4006697
    10.1259184, 63.3971146
    10.1233006, 63.3922521
    10.1484919, 63.3894265
    10.1663446, 63.3964228
    10.1657867, 63.4000164
    10.1519251, 63.4015728
  
  # Waypoint Behavior Parameters
  capture_radius: 30            # Distance to waypoint considered "captured" (meters)
  slip_radius: 75               # Distance for waypoint approach transition (meters)
  
  # Grid Search Visualization Parameters
  grid_cell_size: 10            # Size of each grid cell (meters)
  grid_cell_max_count: 20       # Max visits before cell is fully colored
  grid_cell_decay_time: 5       # Time for visit count to decay (seconds, 0=no decay)
  
  # Sensor Model Parameters
  sensor_detect_pd: 1.0         # Probability of detection (1.0 = 100%) in inner circle
  sensor_radius_min: 20         # Inner radius with full detection probability (meters)
  sensor_radius_max: 30         # Outer radius with linearly decreasing detection (meters)
  sensor_color: white           # Display color for sensor footprint
  sensor_altitude_max: 130      # Altitude where sensor has max range (meters)
  sensor_radius_fixed: true     # Keep radius constant regardless of altitude
  
  # Collision Avoidance Parameters
  encounter_radius: 100         # Distance to detect other UAVs (meters)
  near_miss_radius: 50          # Near-miss detection threshold (meters)
  collision_radius: 10          # Collision detection threshold (meters)
```

### Drone Definitions

Each drone in the swarm needs an entry with the following parameters:

```yaml
drones:
  - name: "skywalker"           # Unique vehicle name (used as MOOS community name)
    color: "orange"             # Display color in pMarineViewer
    start_orientaton_moos:      # Starting position in MOOS coordinates
      x: 0                      # X position (meters, +x = East)
      y: 0                      # Y position (meters, +y = North)
      hdg: 46                   # Initial heading (degrees, 0=North, 90=East)
    # Note: MOOS internally doubles distances, so 50m = 100 MOOS units
  
  - name: "skyfollower"
    color: "hotpink"
    start_orientaton_moos:
      x: -100
      y: -5
      hdg: 36
  
  # Additional drones follow the same pattern...
```

**Important Notes:**
- Drone names must be unique and are used as MOOS community names
- Starting positions use MOOS coordinates where +X is East and +Y is North
- MOOS internally scales distances when convering from lat/lon to XY with a factor of 2
- Heading is in degrees: 0°=North, 90°=East, 180°=South, 270°=West
- The number of drones defined should match `simulation.number_of_drones`
- ArduPilot ports and FDM ports are auto-assigned based on drone index

---

## Related Documentation

- [ARDU Commands Reference](UAV_Mission_ARDU_Commands.md) - Complete list of Ground Control Station commands
- [Launch Scripts Guide](UAV_Mission_Launch_Scripts.md) - Explanation of launch script parameters and usage
- [System Launch Guide](System_Launch_Guide.md) - Step-by-step launch procedures
