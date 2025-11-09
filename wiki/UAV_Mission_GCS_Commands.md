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
  button_7 = DO_VORONOI_ALL # ARDU_COMMAND_ALL=DO_VORONOI
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
  endflag   = ARDU_COMMAND=RETURN_TO_LAUNCH
  
  speed     = 12
  radius    = 8.0
  points    = $(RETURN_POS)
}
```

### Example 3: Mission Completion Triggers

```moos
ProcessConfig = pMissionOperator
{
  // Automatically trigger RTL at mission end
  finish_flag = ARDU_COMMAND_ALL=RETURN_TO_LAUNCH
  finish_flag = DEPLOY_ALL=false
  finish_flag = RETURN_ALL=true
}
```

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
│                                  │                  │
│                                  ▼                  │
│                          Output ARDU_COMMAND         │
│                                  │                  │
│                                  ▼                  │
│                          pArduBridge                │
│                                  │                  │
│                                  ▼                  │
│                          ArduPilot (Flight Control) │
└─────────────────────────────────────────────────────┘
```

---

## Related Documentation

- [UAV Mission Configuration Guide](UAV_Mission_Configuration.md) - YAML configuration details
- [Launch Scripts Guide](UAV_Mission_Launch_Scripts.md) - Launch script usage
- [System Launch Guide](System_Launch_Guide.md) - Complete launch procedures
