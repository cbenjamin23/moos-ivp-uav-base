# UAV ArduPilot Commands Reference

This guide documents the ArduPilot (ARDU) commands used in UAV missions and how to implement them in MOOS configuration files.

## Table of Contents
- [Overview](#overview)
- [ARDU Command List](#ardu-command-list)
- [Using Commands in .moos Files](#using-commands-in-moos-files)
- [Examples](#examples)

---

## Overview

ARDU commands are special MOOS variables that control UAV behavior during missions. They are typically:
- Sent from the shoreside/GCS to vehicles via `pShare` and `uFldShoreBroker`
- Processed by behaviors defined in `.bhv` files
- Triggered by buttons in `pMarineViewer` or through `pRealm` commands

**Key MOOS Variables:**
- `ARDU_COMMAND` - Main command variable for UAV control
- `DEPLOY`, `RETURN`, `DO_SURVEY`, `LOITER` - Behavior activation flags (used with MOOS simulator)
- `ARM_UAV` - Arm/disarm the UAV
- `AUTOPILOT_MODE` - Current mode of the autopilot

---

## ARDU Command List

### Core Flight Commands

| Command | Description | Effect |
|---------|-------------|--------|
| `DO_TAKEOFF` | Initiate takeoff sequence | UAV arms and takes off to mission altitude (currently only works in simulation) |
| `RETURN_TO_LAUNCH` | Return to home position | UAV flies back to starting location and lands |
| `AUTOLAND` | Automatic landing | UAV enters AUTOLAND mode (ArduPilot mode 26) and performs automatic landing based on takeoff direction. Creates landing approach waypoints automatically. **Requires ArduPilot Plane master branch** (not available in stable releases as of end of January 2026). |
| `LOITER` | Hold current position | UAV enters loiter mode at current location |
| `FLY_WAYPOINT` | Fly to specified waypoint | UAV navigates to a target waypoint or resumes waypoint mission |

### Mission Commands

| Command | Description | Effect |
|---------|-------------|--------|
| `DO_VORONOI` | Start Voronoi search pattern | Activates Voronoi-based area coverage algorithm |
| `SURVEY` | Start survey pattern | Activates predefined survey behavior |
| `RESET_SPEED_MIN` | Reset speed to minimum | Sets UAV speed to configured minimum |

### Visualization Commands

| Command | Description | Effect |
|---------|-------------|--------|
| `VIZ_HOME` | Visualize home position | Displays home location marker in viewer |

### Behavior Control Variables

These variables control behavior activation (primarily used with MOOS simulator):

| Variable | Type | Description |
|----------|------|-------------|
| `DEPLOY` | Boolean | Activates main mission behaviors |
| `RETURN` | Boolean | Activates return-to-home behavior |
| `DO_SURVEY` | Boolean | Activates survey pattern |
| `LOITER` | Boolean | Activates loiter/hold behavior |
| `MOOS_MANUAL_OVERRIDE` | Boolean | Manual control override flag |

### Autopilot Mode States

The `AUTOPILOT_MODE` variable indicates the current flight mode:

- `HELM_INACTIVE` - System inactive
- `HELM_INACTIVE_LOITERING` - Inactive but loitering
- `HELM_PARKED` - On ground, disarmed
- `HELM_TOWAYPT` - Flying to waypoint
- `HELM_RETURNING` - Returning to launch
- `HELM_SURVEYING` - Executing survey pattern
- `HELM_LOITERING` - Holding position

**Note:** When AUTOLAND is active, the autopilot is in ArduPilot's AUTOLAND mode (mode 26), but the helm state remains `HELM_INACTIVE` since the landing is handled entirely by ArduPilot.

---

## Using Commands in .moos Files

### 1. Bridging Commands to Vehicles (Shoreside)

In `meta_shoreside.moos`, use `uFldShoreBroker` to bridge commands:

```moos
ProcessConfig = uFldShoreBroker
{
  // Quick-bridge: broadcasts these variables to all vehicles
  qbridge = ARDU_COMMAND, DEPLOY, RETURN, DO_SURVEY, LOITER
  qbridge = PROX_POLY_VIEW
  
  // Individual bridges with aliases (if needed)
  // bridge = src=RETURN_ALL, alias=RETURN
  // bridge = src=RETURN_$V, alias=RETURN
}
```

**Explanation:**
- `qbridge` - Quick bridge, sets up the bridge similar to the individual procedure (but is a shorthand)
- Variables sent as `VAR_ALL` are automatically broadcast to all vehicles
- `$V` is replaced with vehicle name for individual targeting

### 2. Defining Buttons (Shoreside)

In `pMarineViewer` configuration, buttons send commands when clicked:

```moos
ProcessConfig = pMarineViewer
{
  // Button format: button_N = LABEL # VAR1=value # VAR2=value # ...
  
  button_1 = DEPLOY  # DEPLOY_ALL=true # MOOS_MANUAL_OVERRIDE_ALL=false # RETURN_ALL=false
  button_2 = RETURN  # RETURN_ALL=true # DEPLOY_ALL=false
  button_3 = SURVEY  # DO_SURVEY_ALL=true # DEPLOY_ALL=false # RETURN_ALL=false
  button_4 = LOITER  # LOITER_ALL=true # DEPLOY_ALL=false # RETURN_ALL=false
  
  // UAV-specific buttons
  button_1 = RTL_ALL     # ARDU_COMMAND_ALL=RETURN_TO_LAUNCH 
  button_3 = TKOFF_ALL   # ARM_UAV_ALL=true # ARDU_COMMAND_ALL=DO_TAKEOFF
  button_4 = SURVEY_ALL  # ARDU_COMMAND_ALL=SURVEY  
  button_5 = LOITER_ALL  # ARDU_COMMAND_ALL=LOITER
  button_6 = TOWYP_ALL   # ARDU_COMMAND_ALL=FLY_WAYPOINT
  button_7 = AUTOLAND_ALL # ARDU_COMMAND_ALL=AUTOLAND
  button_8 = DO_VORONOI_ALL # ARDU_COMMAND_ALL=DO_VORONOI
}
```

**Naming Convention:**
- `VAR_ALL` broadcasts to all vehicles
- `VAR_<VNAME>` targets specific vehicle (e.g., `DEPLOY_skywalker`)

### 3. Defining pRealm Commands (Shoreside)

`pRealm` allows sending commands from the command line or scripts:

```moos
ProcessConfig = pRealm
{
  // Command format: cmd = label=NAME, var=VARIABLE, sval=VALUE, receivers=targets
  
  cmd = label=TAKEOFF, var=ARDU_COMMAND, sval=DO_TAKEOFF, receivers=all:$(VNAMES)
  cmd = label=TOWAYPOINT, var=ARDU_COMMAND, sval=FLY_WAYPOINT, receivers=all:$(VNAMES)
  cmd = label=RTL, var=ARDU_COMMAND, sval=RETURN_TO_LAUNCH, receivers=all:$(VNAMES)
  cmd = label=LOITER/HOLD, var=ARDU_COMMAND, sval=LOITER, receivers=all:$(VNAMES)
  cmd = label=AUTOLAND, var=ARDU_COMMAND, sval=AUTOLAND, receivers=all:$(VNAMES)
  cmd = label=SPEED_TO_MIN, var=ARDU_COMMAND, sval=RESET_SPEED_MIN, receivers=all:$(VNAMES)
  cmd = label=DO_VORONOI, var=ARDU_COMMAND, sval=DO_VORONOI, receivers=all:$(VNAMES)
  cmd = label=SURVEY, var=ARDU_COMMAND, sval=SURVEY, receivers=all:$(VNAMES)
}
```

**Usage:** Send commands via uPokeDB or command line:
```bash
uPokeDB targ_shoreside.moos REALM_CMD="cmd=TAKEOFF"
```

### 4. Processing Commands in Behaviors (Vehicle)

In `.bhv` files, behaviors respond to these commands through conditions:

```moos
//-------- Behavior Mode Definitions --------

initialize DEPLOY = false
initialize RETURN = false
initialize DO_SURVEY = false
initialize LOITER = false

// Define when to activate behaviors
set BHV_MODE = INACTIVE {
  DEPLOY != true
  RETURN != true
  DO_SURVEY != true
  LOITER != true
} ACTIVE

set BHV_MODE = SURVEY {
  BHV_MODE = ACTIVE
  DO_SURVEY = true
  DEPLOY != true
  RETURN != true
} 

set BHV_MODE = VORONOI {
  BHV_MODE = ACTIVE
  DEPLOY = true
  RETURN != true
  DO_SURVEY != true
} 

set BHV_MODE = TOWAYPT {
  BHV_MODE = ACTIVE
  RETURN = true
  DEPLOY != true
} 

set BHV_MODE = RETURN {
  BHV_MODE = ACTIVE
  AUTOPILOT_MODE = HELM_RETURNING
}
```

### 5. ARDU Command Processing (Vehicle)

When using ArduPilot integration, behaviors can trigger ARDU commands:

```moos
//-------- Behavior: Waypoint --------
Behavior = BHV_Waypoint
{
  name      = waypt_survey
  pwt       = 100
  condition = BHV_MODE = SURVEY
  endflag   = ARDU_COMMAND=LOITER    // Send command when behavior completes
  
  // Behavior parameters...
}

//-------- Behavior: Return to Launch --------
Behavior = BHV_Waypoint
{
  name      = waypt_return
  pwt       = 100
  condition = BHV_MODE = RETURN
  endflag   = ARDU_COMMAND=RETURN_TO_LAUNCH  // Trigger RTL in autopilot
  
  speed     = 12
  radius    = 8.0
  points    = $(START_POS)
}

//-------- Behavior: Survey Mission (with AUTOLAND on completion) --------
Behavior = BHV_Waypoint
{
  name      = waypt_survey
  pwt       = 100
  condition = BHV_MODE = SURVEY
  endflag   = ARDU_COMMAND=AUTOLAND  // Trigger AUTOLAND when survey completes
  
  speed     = 12
  radius    = 8.0
  points    = $(SURVEY_POINTS)
}
```

---

## Examples

### Example 1: Complete Shoreside Configuration

```moos
//-------- meta_shoreside.moos --------

ProcessConfig = uFldShoreBroker
{
  // Bridge commands to all vehicles
  qbridge = ARDU_COMMAND, PROX_POLY_VIEW
  qbridge = DEPLOY, RETURN, DO_SURVEY, LOITER
}

ProcessConfig = pMarineViewer
{
  // Deployment buttons
  button_1 = DEPLOY  # DEPLOY_ALL=true # MOOS_MANUAL_OVERRIDE_ALL=false
  button_2 = RETURN  # RETURN_ALL=true # DEPLOY_ALL=false
  
  // UAV-specific buttons
  button_3 = TAKEOFF # ARM_UAV_ALL=true # ARDU_COMMAND_ALL=DO_TAKEOFF
  button_4 = RTL     # ARDU_COMMAND_ALL=RETURN_TO_LAUNCH
  button_5 = SURVEY  # ARDU_COMMAND_ALL=SURVEY
}

ProcessConfig = pRealm
{
  cmd = label=RTL, var=ARDU_COMMAND, sval=RETURN_TO_LAUNCH, receivers=all:$(VNAMES)
  cmd = label=TAKEOFF, var=ARDU_COMMAND, sval=DO_TAKEOFF, receivers=all:$(VNAMES)
}
```

### Example 2: Vehicle Behavior Response

```moos
//-------- meta_vehicle.bhv --------

initialize DEPLOY = false
initialize RETURN = false

// Inactive when not deployed or returning
set BHV_MODE = INACTIVE {
  DEPLOY != true
  RETURN != true
} ACTIVE

// Active mission mode
set BHV_MODE = VORONOI {
  BHV_MODE = ACTIVE
  DEPLOY = true
  RETURN != true
} 

// Return mode
set BHV_MODE = RETURN {
  BHV_MODE = ACTIVE
  RETURN = true
}

Behavior = BHV_Waypoint
{
  name      = waypt_return
  condition = BHV_MODE = RETURN
  endflag   = ARDU_COMMAND=AUTOLAND  // Use AUTOLAND instead of RTL for automatic landing
  
  speed     = 12
  radius    = 8.0
  points    = $(RETURN_POS)
}
```

### Example 3: Mission Completion Triggers

```moos
ProcessConfig = pMissionOperator
{
  // Automatically trigger AUTOLAND at mission end
  finish_flag = ARDU_COMMAND_ALL=AUTOLAND
  finish_flag = DEPLOY_ALL=false
  finish_flag = RETURN_ALL=false
}
```

**Note:** AUTOLAND is now the default landing mode for mission completion. It provides automatic landing based on takeoff direction, eliminating the need for pre-programmed landing sequences.

---

## Command Flow Diagram

```
┌─────────────────────────────────────────────────────┐
│ Ground Control Station (Shoreside)                  │
│                                                     │
│  User Input:                                        │
│  ├─ pMarineViewer buttons → VAR_ALL=value          │
│  ├─ pRealm commands → VAR=value                    │
│  └─ uPokeDB commands → VAR=value                   │
│                                                     │
│  ▼                                                  │
│  uFldShoreBroker (qbridge) ───────────────┐         │
└───────────────────────────────────────────│─────────┘
                                            │
                    Network (pShare)        │
                                            ▼
┌─────────────────────────────────────────────────────┐
│ Vehicle (UAV)                                       │
│                                                     │
│  uFldNodeBroker ─► MOOSDB ─► pHelmIvP              │
│                                  │                  │
│                                  ▼                  │
│                          Read .bhv conditions       │
│                          (DEPLOY, RETURN, etc.)     │
│                                  │                  │
│                                  ▼                  │
│                          Activate Behaviors         │
│                          (endflag triggers)         │
│                                  │                  │
│                                  ▼                  │
│                          Output ARDU_COMMAND         │
│                          (e.g., ARDU_COMMAND=AUTOLAND)│
│                                  │                  │
│                                  ▼                  │
│                          pArduBridge                │
│                          (ArduBridge::OnNewMail)    │
│                                  │                  │
│                                  ▼                  │
│                          UAV_Model                  │
│                          (commandAutoland)           │
│                                  │                  │
│                                  ▼                  │
│                          MAVSDK Action Plugin        │
│                          (set_flight_mode_autoland) │
│                                  │                  │
│                                  ▼                  │
│                          SystemImpl                 │
│                          (FlightMode::Land →        │
│                           PlaneMode::Autoland)      │
│                                  │                  │
│                                  ▼                  │
│                          MAVLink Command            │
│                          (MAV_CMD_DO_SET_MODE,       │
│                           mode=26)                   │
│                                  │                  │
│                                  ▼                  │
│                          ArduPilot                  │
│                          (ModeAutoLand execution)   │
└─────────────────────────────────────────────────────┘
```

## AUTOLAND Command Flow

The AUTOLAND command follows this specific flow:

1. **User Interface**: Button click in pMarineViewer or pRealm command
   - Publishes `ARDU_COMMAND_ALL=AUTOLAND` to MOOS DB

2. **Command Reception**: ArduBridge receives command
   - `ArduBridge::OnNewMail()` processes `ARDU_COMMAND=AUTOLAND`
   - Sets flag `m_do_autoland = true`

3. **Command Processing**: ArduBridge iterate loop
   - `ArduBridge::Iterate()` checks `m_do_autoland` flag
   - Calls `autoland_async()` which queues command in UAV_Model thread

4. **UAV Model Execution**: Command sent to ArduPilot
   - `UAV_Model::commandAutoland()` validates authority and in-air status
   - Calls `m_action_ptr->set_flight_mode_autoland()`

5. **MAVSDK Action Plugin**: Mode conversion
   - `ActionImpl::set_flight_mode_autoland()` converts to `FlightMode::Land`
   - Calls `SystemImpl::set_flight_mode(FlightMode::Land)`

6. **System Implementation**: ArduPilot mode mapping
   - `SystemImpl::make_command_ardupilot_mode()` converts `FlightMode::Land` → `ardupilot::PlaneMode::Autoland` (mode 26)
   - Constructs MAVLink `MAV_CMD_DO_SET_MODE` command with param2 = 26

7. **ArduPilot Execution**: Automatic landing
   - ArduPilot receives mode change command
   - Enters `ModeAutoLand` (mode 26)
   - Sets up landing waypoints based on captured takeoff direction
   - Executes automatic landing sequence

### AUTOLAND Requirements

- **Takeoff Direction**: AUTOLAND requires that takeoff direction was captured. This happens automatically when taking off in:
  - TAKEOFF, FBWA, MANUAL, TRAINING, ACRO, STABILIZE modes
  - During NAV_TAKEOFF in AUTO mode
- **In Air**: UAV must be flying (not on ground)
- **ArduPilot Version**: **Requires ArduPilot Plane master branch** (as of end of January 2026, AUTOLAND is only available in the master branch, not in stable releases like 4.6.x or 4.7.x)
- **Not for QuadPlanes**: AUTOLAND is not available for QuadPlanes

---

## Technical Details: AUTOLAND Implementation

### How AUTOLAND Works

AUTOLAND is implemented using the MAVSDK flight mode system with automatic mode conversion:

1. **Command Reception**: `ARDU_COMMAND=AUTOLAND` is received by `ArduBridge::OnNewMail()` (line 258 in `ArduBridge.cpp`)

2. **Async Processing**: `ArduBridge::Iterate()` processes the command asynchronously (line 510), calling `autoland_async()` (line 1420)

3. **UAV Model Execution**: `UAV_Model::commandAutoland()` (line 491) validates the command and calls the MAVSDK Action plugin

4. **MAVSDK Action Plugin**: `ActionImpl::set_flight_mode_autoland()` (line 568 in `action_impl.cpp`) converts to `FlightMode::Land`

5. **Mode Conversion**: `SystemImpl::flight_mode_to_ardupilot_plane_mode()` (line 928 in `system_impl.cpp`) maps `FlightMode::Land` → `ardupilot::PlaneMode::Autoland` (mode 26)

6. **MAVLink Command**: `MAV_CMD_DO_SET_MODE` is constructed with:
   - `param1` = mode flags (CUSTOM_MODE_ENABLED | SAFETY_ARMED if armed)
   - `param2` = 26 (AUTOLAND mode number)

7. **ArduPilot Execution**: ArduPilot receives the command and enters `ModeAutoLand`, which:
   - Validates takeoff direction is captured
   - Creates landing approach waypoints based on takeoff direction
   - Executes automatic landing sequence

### Mode Mapping

The implementation uses a smart mapping strategy:
- **MAVSDK Level**: `FlightMode::Land` (generic landing mode)
- **ArduPilot Level**: `ardupilot::PlaneMode::Autoland` (mode 26)

This allows the same `Land` flight mode to work for:
- Multicopters: Uses standard Land mode
- Fixed-wing (Plane): Uses AUTOLAND mode (26)

### Key Implementation Files

- **Command Handling**: `src/pArduBridge/ArduBridge.cpp` (lines 258-262, 510-541, 1420-1432)
- **UAV Interface**: `src/pArduBridge/UAV_Model.cpp` (lines 491-522)
- **MAVSDK Action**: `MAVSDK/src/mavsdk/plugins/action/action_impl.cpp` (lines 568-575)
- **Mode Conversion**: `MAVSDK/src/mavsdk/core/system_impl.cpp` (lines 754-765, 793-865, 928-955)
- **Mode Definitions**: `MAVSDK/src/mavsdk/core/ardupilot_custom_mode.h` (line 79: `Autoland = 26`)

---

## Related Documentation

- [UAV Mission Configuration Guide](UAV_Mission_Configuration.md) - YAML configuration details
- [Launch Scripts Guide](UAV_Mission_Launch_Scripts.md) - Launch script usage
- [System Launch Guide](System_Launch_Guide.md) - Complete launch procedures
- [pArduBridge Architecture](pArduBridge/ARCHITECTURE.md) - Detailed architecture and command flow
