# pArduBridge - ArduPilot to MOOS-IVP Interface

> **Note:** pArduBridge is used to interface between MOOS-IVP and ArduPilot. This is **only required** when using the ArduPilot SITL + Gazebo simulation approach (`useMoosSimPid: false`) or when working with physical hardware. If you're using the lightweight MOOS-IvP Simulator approach, you don't need pArduBridge. See [Installation & Setup](../Installation_&_Setup.md) for more information about simulation approaches.

## Overview

`pArduBridge` is a MOOS application that interfaces between the MOOS-IVP autonomy system and ArduPilot autopilot software running on UAVs (Unmanned Aerial Vehicles). It acts as a bridge, translating MOOS commands into MAVLink commands for the autopilot and publishing telemetry data from the UAV back to the MOOS community.

## Purpose

The primary purposes of pArduBridge are:

1. **Command Translation**: Convert high-level MOOS autonomy commands into low-level ArduPilot commands
2. **Telemetry Publishing**: Provide UAV state information to the MOOS community
3. **State Management**: Manage the UAV's autopilot and helm states
4. **Safety Monitoring**: Monitor UAV health and provide warnings when issues arise

## Key Features

- **MAVLink Communication**: Uses the MAVSDK library to communicate with ArduPilot via MAVLink protocol
- **Multi-threaded Architecture**: Separate threads for command processing and main MOOS loop
- **Flexible Connectivity**: Supports TCP, UDP, and Serial connections to the autopilot
- **Helm Integration**: Integrates with MOOS-IVP helm for autonomous mission execution
- **Warning System**: Built-in warning and error reporting system
- **Setpoint Management**: Thread-safe management of desired speed, course, and altitude

---

## Configuration

### MOOS Configuration Block

```
ProcessConfig = pArduBridge
{
  AppTick   = 4                                   // Application tick rate (Hz)
  CommsTick = 4                                   // Communication tick rate (Hz)
  
  // Connection Configuration
  ArduPilotURL = 0.0.0.0:14550                    // Connection string to autopilot
  url_protocol = udp                              // Protocol: udp, tcp, or serial
  
  // Vehicle Configuration
  prefix = NAV                                    // Prefix for published MOOS variables
  vname = ALPHA                                   // Vehicle name
  vcolor = yellow                                 // Vehicle color for visualization
  
  // Operational Configuration
  is_simulation = false                           // Set to true when running in simulation
  command_groundSpeed = false                     // If true, command ground speed instead of airspeed
}
```

### Configuration Parameters

| Parameter | Type | Description | Example |
|-----------|------|-------------|---------|
| `ArduPilotURL` / `ardupiloturl` / `url` | string | Connection address for autopilot | `0.0.0.0:14550` or `ttySAC0:115200` |
| `url_protocol` | string | Connection protocol: `udp`, `tcp`, or `serial` | `udp` |
| `prefix` | string | Prefix for all published telemetry variables | `NAV` |
| `vname` | string | Vehicle name identifier | `ALPHA` |
| `vcolor` | string | Color for vehicle visualization | `yellow` |
| `is_simulation` | boolean | Flag indicating simulation mode | `false` |
| `command_groundSpeed` | boolean | Command ground speed vs airspeed | `false` |

### Connection Examples

**UDP Connection (default for SITL):**
```
ArduPilotURL = 0.0.0.0:14550
url_protocol = udp
```

**Serial Connection:**
```
ArduPilotURL = ttySAC0:115200
url_protocol = serial
```

**TCP Connection:**
```
ArduPilotURL = 192.168.1.100:5760
url_protocol = tcp
```

---

## MOOS Variables

### Subscriptions (Input Variables)

pArduBridge subscribes to the following MOOS variables to receive commands:

#### Helm Commands (from pHelmIvp)
| Variable | Type | Description |
|----------|------|-------------|
| `DESIRED_HEADING` | double | Desired course/heading in degrees (0-360) |
| `DESIRED_SPEED` | double | Desired speed in m/s |
| `DESIRED_ALTITUDE` | double | Desired altitude AGL in meters |

#### Direct Commands
| Variable | Type | Description |
|----------|------|-------------|
| `FLY_WAYPOINT` | string | Command to fly to a specific waypoint |
| `DO_TAKEOFF` | string | Command to initiate takeoff sequence |
| `ARM_UAV` | string | Arm/disarm the UAV (`on` or `off`) |
| `RETURN_TO_LAUNCH` | string | Command UAV to return to launch location |
| `LOITER` | string | Command UAV to loiter at position (`here` or `default`) |
| `CHANGE_SPEED` | double | Increment/decrement speed by value |
| `CHANGE_COURSE` | double | Increment/decrement course by value (degrees) |
| `CHANGE_ALTITUDE` | double | Increment/decrement altitude by value (meters) |
| `RESET_SPEED_MIN` | string | Reset speed to minimum airspeed |

#### Waypoint and State Management
| Variable | Type | Description |
|----------|------|-------------|
| `NEXT_WAYPOINT` | string | Set next waypoint (format: `lat=X,lon=Y,x=X,y=Y,vname=NAME`) |
| `HELM_STATUS` | string | Helm status (`on` or `off`) |
| `AUTOPILOT_MODE` | string | Desired autopilot mode |
| `MOOS_MANUAL_OVERRIDE` | string | Manual override flag (`true` or `false`) |

#### Ground Control Station Commands
| Variable | Type | Description |
|----------|------|-------------|
| `ARDU_COMMAND` | string | Direct commands from ground control station |

#### Survey Missions
| Variable | Type | Description |
|----------|------|-------------|
| `SURVEY` | string | Survey mission commands |

#### Visualization
| Variable | Type | Description |
|----------|------|-------------|
| `VIZ_HOME` | string | Visualization commands for home location |

### Publications (Output Variables)

pArduBridge publishes telemetry and status information with a configurable prefix (default: `UAV`):

#### Position and Navigation
| Variable | Type | Description |
|----------|------|-------------|
| `<PREFIX>_LAT` | double | Current latitude (degrees) |
| `<PREFIX>_LON` | double | Current longitude (degrees) |
| `<PREFIX>_X` | double | Local X coordinate (meters) |
| `<PREFIX>_Y` | double | Local Y coordinate (meters) |
| `<PREFIX>_HEADING` | double | Current heading/course over ground (degrees) |
| `<PREFIX>_SPEED` | double | Speed over ground (m/s) |
| `<PREFIX>_ALTITUDE` | double | Altitude above ground level (meters) |
| `<PREFIX>_DEPTH` | double | Negative altitude (for compatibility) |

#### Additional Telemetry
| Variable | Type | Description |
|----------|------|-------------|
| `DEPLOY` | string | Deployment status |

**Note**: Replace `<PREFIX>` with the configured prefix (e.g., if `prefix = NAV`, variables become `NAV_LAT`, `NAV_LON`, etc.)

---

## Usage Instructions

### 1. Basic Startup

1. **Configure the mission file**: Create or edit your `.moos` file with the pArduBridge configuration block
2. **Ensure ArduPilot is running**: Start your ArduPilot SITL or connect to hardware autopilot
3. **Launch pArduBridge**: 
   ```bash
   pArduBridge mission.moos
   ```

### 2. Command-Line Options

```bash
pArduBridge mission.moos [OPTIONS]
```

| Option | Description |
|--------|-------------|
| `--alias=<name>` | Launch with custom process name |
| `--example` / `-e` | Display example configuration |
| `--help` / `-h` | Display help message |
| `--interface` / `-i` | Display MOOS interface (subscriptions/publications) |
| `--version` / `-v` | Display version information |

### 3. Operational Workflow

#### Initial Setup
1. Connect to ArduPilot autopilot
2. Wait for telemetry subscription
3. Poll autopilot parameters
4. Register home location
5. Start command sender thread

#### During Operation
1. **Helm Active Mode**: pArduBridge receives `DESIRED_HEADING`, `DESIRED_SPEED`, `DESIRED_ALTITUDE` from helm and sends to autopilot
2. **Manual Commands**: Direct commands like `FLY_WAYPOINT`, `LOITER`, `RETURN_TO_LAUNCH` can be issued
3. **Telemetry Publishing**: UAV state continuously published to MOOS community

#### Autopilot Helm Modes

pArduBridge manages several helm states:

| Mode | Description |
|------|-------------|
| `HELM_PARKED` | Helm is parked, vehicle inactive |
| `HELM_INACTIVE` | Helm not in control, manual override active |
| `HELM_INACTIVE_LOITERING` | Loitering with helm inactive |
| `HELM_ACTIVE` | Helm active but no specific behavior running |
| `HELM_TOWAYPT` | Actively navigating to waypoint |
| `HELM_RETURNING` | Returning to launch location |
| `HELM_SURVEYING` | Executing survey pattern |
| `HELM_VORONOI` | Executing Voronoi pattern |

### 4. Common Operations

#### Arming the UAV
```
// From pMarineViewer or another MOOS app
Notify("ARM_UAV", "on");
```

#### Flying to a Waypoint
```
Notify("FLY_WAYPOINT", "true");
```

#### Loitering at Current Position
```
Notify("LOITER", "here");
```

#### Return to Launch
```
Notify("RETURN_TO_LAUNCH", "true");
```

#### Adjusting Speed
```
// Increase speed by 2 m/s
Notify("CHANGE_SPEED", 2.0);

// Decrease speed by 1 m/s
Notify("CHANGE_SPEED", -1.0);
```

---

## Architecture Overview

For detailed architectural information, see [ARCHITECTURE.md](ARCHITECTURE.md).

### Main Components

1. **ArduBridge (Main Application)**
   - Inherits from `AppCastingMOOSApp`
   - Handles MOOS communication
   - Manages autopilot helm state machine
   - Coordinates between helm and autopilot

2. **UAV_Model**
   - Manages MAVLink connection via MAVSDK
   - Runs command sender thread
   - Handles telemetry subscription
   - Executes commands on autopilot

3. **SetpointManager**
   - Thread-safe storage of desired values
   - Tracks changes in setpoints
   - Provides polling interface

4. **WarningSystem**
   - Monitors conditions and raises warnings
   - Reports warnings to MOOS community
   - Manages time-based warnings

### Threading Model

- **Main Thread (MOOS Loop)**: Handles MOOS communication, state management, processes incoming mail
- **UAV_Model Command Sender Thread**: Processes command queue, sends commands to autopilot, manages telemetry

---

## Safety Considerations

1. **Warning System**: pArduBridge includes a comprehensive warning system that monitors for:
   - Connection loss
   - Unhealthy UAV state
   - Command failures
   - Timeout conditions

2. **Health Checks**: Before arming, the system verifies:
   - UAV health status
   - Connection status
   - GPS fix quality

3. **Manual Override**: The `MOOS_MANUAL_OVERRIDE` variable allows immediate manual control

---

## Troubleshooting

### Connection Issues

**Problem**: Cannot connect to autopilot
- Verify ArduPilot is running
- Check connection string and protocol
- Ensure firewall allows connection
- For serial: check device permissions

**Problem**: Telemetry not updating
- Check AppTick and CommsTick rates
- Verify MOOS community is reachable
- Check for warning messages

### Command Issues

**Problem**: Commands not being executed
- Verify UAV is armed and healthy
- Check autopilot mode (must be in GUIDED for many commands)
- Review warning messages for details

**Problem**: Helm not controlling UAV
- Ensure `HELM_STATUS = on`
- Verify `MOOS_MANUAL_OVERRIDE` is false
- Check that desired values are being published by helm

---

## Development and Updates

This documentation should be updated as pArduBridge evolves. Key areas that may change:

- Additional MAVLink commands and telemetry
- New helm modes and state transitions
- Enhanced safety features
- Performance optimizations

---

## See Also

- [ARCHITECTURE.md](ARCHITECTURE.md) - Detailed architecture and design
- [ArduPilot Documentation](https://ardupilot.org/)
- [MOOS-IVP Documentation](https://oceanai.mit.edu/moos-ivp/)
- [MAVSDK Documentation](https://mavsdk.mavlink.io/)

---

## Credits

**Author**: Steve Carter Feujo Nomeny  
**Organization**: NTNU, MIT  
**Email**: scnomeny@mit.edu
