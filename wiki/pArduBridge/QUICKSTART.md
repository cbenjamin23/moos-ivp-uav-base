# pArduBridge Quick Start Guide

> **Note:** pArduBridge is used to interface between MOOS-IVP and ArduPilot (`useMoosSimPid: false`). This is required when working with physical hardware or ArduPilot SITL. Gazebo is optional - ArduPilot SITL can run with or without Gazebo for enhanced physics simulation. If you're using the lightweight MOOS-IvP Simulator, you don't need pArduBridge.

This guide will help you get pArduBridge up and running quickly.

---

## Prerequisites

1. **MOOS-IVP** installed and configured
2. **ArduPilot** SITL or hardware autopilot running
3. **MAVSDK** library installed (should be built with the project)
4. **Mission file** with proper LatOrigin and LongOrigin configured

---

## Step-by-Step Setup

### 1. Start ArduPilot

#### For SITL (Software In The Loop):
```bash
# Navigate to ArduPilot directory
cd ~/ardupilot/ArduPlane
# Start SITL
sim_vehicle.py -v ArduPlane --console --map
```

SITL typically listens on UDP port 14550.

#### For Hardware:
Connect your flight controller via USB or telemetry radio. Note the connection details (e.g., `/dev/ttyUSB0:57600`).

---

### 2. Configure Your Mission File

Create or edit your `.moos` mission file:

```
// Mission file example: alpha.moos

ServerHost = localhost
ServerPort = 9000
Community  = alpha

LatOrigin  = 42.358456      // Your operation area latitude
LongOrigin = -71.087589     // Your operation area longitude

//------------------------------------------
// pArduBridge Configuration

ProcessConfig = pArduBridge
{
  AppTick   = 4
  CommsTick = 4

  // For SITL
  ArduPilotURL = 0.0.0.0:14550
  url_protocol = udp
  
  // For Hardware (example)
  // ArduPilotURL = ttyUSB0:57600
  // url_protocol = serial

  prefix = NAV
  vname  = alpha
  vcolor = yellow
  
  is_simulation = true
  command_groundSpeed = false
}

//------------------------------------------
// Other required processes...

ProcessConfig = pHelmIvp
{
  // ... helm configuration
}

ProcessConfig = pMarineViewer
{
  // ... viewer configuration
}
```

---

### 3. Build pArduBridge

```bash
# Navigate to repository root
cd ~/moos-ivp-uav-base

# Build
./build.sh

# The executable will be in build/bin/
```

---

### 4. Launch the Mission

#### Option A: Using pAntler (recommended)

```bash
# Launch entire mission
pAntler alpha.moos
```

#### Option B: Launch pArduBridge separately

```bash
# In separate terminals:
MOOSDB alpha.moos
pArduBridge alpha.moos
pHelmIvp alpha.moos
pMarineViewer alpha.moos
# ... other processes
```

---

### 5. Verify Connection

Once running, check the pArduBridge console output:

```
✓ Connected to ArduPilot
✓ Telemetry subscribed
✓ Home location registered
✓ Command sender thread started
```

In pMarineViewer, you should see:
- Vehicle marker at current position
- Telemetry updates (NAV_X, NAV_Y, NAV_HEADING, etc.)
- Home location marker

---

## Basic Operations

### Arming the UAV

From pMarineViewer or uPokeDB:
```
Notify("ARM_UAV", "on");
```

Watch for health checks to pass before the UAV arms.

### Enabling the Helm

```
Notify("HELM_STATUS", "on");
```

This allows the helm to send desired speed, course, and altitude to the UAV.

### Manual Commands

#### Takeoff (if implemented)
```
Notify("DO_TAKEOFF", "true");
```

#### Fly to Waypoint
```
Notify("FLY_WAYPOINT", "true");
```

#### Loiter at Current Position
```
Notify("LOITER", "here");
```

#### Return to Launch
```
Notify("RETURN_TO_LAUNCH", "true");
```

#### Adjust Speed
```
# Increase by 2 m/s
Notify("CHANGE_SPEED", 2.0);

# Decrease by 2 m/s
Notify("CHANGE_SPEED", -2.0);
```

#### Adjust Altitude
```
# Increase by 10 meters
Notify("CHANGE_ALTITUDE", 10.0);

# Decrease by 10 meters
Notify("CHANGE_ALTITUDE", -10.0);
```

---

## Common Issues and Solutions

### Issue: "Failed to connect to ArduPilot"

**Solutions:**
1. Verify ArduPilot is running
2. Check the connection string in your .moos file
3. For UDP: ensure port 14550 is not blocked by firewall
4. For serial: ensure user has permission to access device
   ```bash
   sudo usermod -a -G dialout $USER
   # Then log out and back in
   ```

### Issue: "No telemetry updates"

**Solutions:**
1. Check AppTick and CommsTick are set (typically 4 Hz)
2. Verify MOOSDB is running
3. Check for error messages in pArduBridge console
4. Ensure ArduPilot is streaming telemetry

### Issue: "UAV not responding to helm commands"

**Solutions:**
1. Verify HELM_STATUS is "on"
   ```
   Notify("HELM_STATUS", "on");
   ```
2. Check that MOOS_MANUAL_OVERRIDE is false
3. Ensure UAV is armed and healthy
4. Verify helm is publishing DESIRED_HEADING, DESIRED_SPEED, DESIRED_ALTITUDE

### Issue: "Commands not executing"

**Solutions:**
1. Check autopilot mode (use AppCast)
2. Verify UAV is armed
3. Some commands require GUIDED mode - pArduBridge will attempt to switch automatically
4. Check warning messages in AppCast output

---

## Monitoring Status

### Using AppCast

pArduBridge provides detailed status via AppCast. In pMarineViewer:

1. Right-click on the vehicle
2. Select "AppCast" → "pArduBridge"

You'll see:
- Current helm autonomy mode
- Position (Lat/Lon and X/Y)
- Altitude MSL and AGL
- Speed, heading, and course
- Comparison of measured values, helm desired values, and UAV targets
- Waypoint information
- Debug flags and command status

### Using uPokeDB

Monitor specific variables:
```bash
uPokeDB alpha.moos
# Then type variable names to see their values
NAV_X
NAV_Y
NAV_HEADING
NAV_SPEED
AUTOPILOT_MODE
```

---

## Next Steps

Once you have basic operation working:

1. **Configure Behaviors**: Set up MOOS-IVP behaviors for waypoint following, loitering, etc.
2. **Tune Parameters**: Adjust autopilot parameters for your UAV platform
3. **Add Safety Features**: Configure geofences and return-to-launch conditions
4. **Integrate Sensors**: Add camera, lidar, or other sensor processing applications

---

## Further Reading

- [README.md](README.md) - Comprehensive pArduBridge documentation
- [ARCHITECTURE.md](ARCHITECTURE.md) - Detailed architecture and design
- [MOOS-IVP Documentation](https://oceanai.mit.edu/moos-ivp/)
- [ArduPilot Documentation](https://ardupilot.org/)

---

## Getting Help

If you encounter issues:

1. Check the console output for error messages
2. Review the AppCast for detailed status
3. Verify your configuration matches the examples
4. Check that all prerequisites are installed and running
5. Review the log file: `~/moos-ivp-uav-base/missions/pArduBrigeLog_<vname>.log`

---

## Example Mission Files

Example mission files are typically located in:
```
~/moos-ivp-uav-base/missions/
```

These provide working examples you can adapt for your use case.

---

**Happy Flying!**

For updates and contributions, visit the repository:
https://github.com/cbenjamin23/moos-ivp-uav-base
