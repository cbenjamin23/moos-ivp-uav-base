# pArduBridge Architecture

This document provides detailed architectural information about the pArduBridge application, including component design, threading model, and data flow.

---

## Table of Contents

1. [System Architecture](#system-architecture)
2. [Component Overview](#component-overview)
3. [Threading Model](#threading-model)
4. [Data Flow](#data-flow)
5. [State Machine](#state-machine)
6. [Class Relationships](#class-relationships)
7. [Sequence Diagrams](#sequence-diagrams)

---

## System Architecture

### High-Level Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                         MOOS Community                              │
│  (pHelmIvp, pMarineViewer, pNodeReporter, etc.)                    │
└────────────────┬────────────────────────────────────────────────────┘
                 │ MOOS Variables
                 │ (DESIRED_HEADING, DESIRED_SPEED, etc.)
                 ▼
┌─────────────────────────────────────────────────────────────────────┐
│                        pArduBridge                                  │
│  ┌──────────────────────────────────────────────────────────────┐  │
│  │                    ArduBridge                                │  │
│  │  (Main MOOS App - AppCastingMOOSApp)                        │  │
│  │                                                              │  │
│  │  ┌────────────────┐  ┌──────────────────┐  ┌─────────────┐ │  │
│  │  │ SetpointManager│  │  WarningSystem   │  │  Geodesy    │ │  │
│  │  │ (Thread-Safe)  │  │                  │  │             │ │  │
│  │  └────────────────┘  └──────────────────┘  └─────────────┘ │  │
│  │                                                              │  │
│  │  ┌──────────────────────────────────────────────────────┐  │  │
│  │  │              UAV_Model                               │  │  │
│  │  │  ┌──────────────────────────────────────────────┐   │  │  │
│  │  │  │    Command Sender Thread                     │   │  │  │
│  │  │  │  - Command Queue Processing                  │   │  │  │
│  │  │  │  - Periodic Desired Values Sending           │   │  │  │
│  │  │  │  - Parameter Polling                         │   │  │  │
│  │  │  └──────────────────────────────────────────────┘   │  │  │
│  │  │                                                      │  │  │
│  │  │  ┌──────────────────────────────────────────────┐   │  │  │
│  │  │  │           MAVSDK Interface                   │   │  │  │
│  │  │  │  - Action Plugin                             │   │  │  │
│  │  │  │  - Telemetry Plugin                          │   │  │  │
│  │  │  │  - Mission Plugin                            │   │  │  │
│  │  │  │  - Param Plugin                              │   │  │  │
│  │  │  │  - MAVLink Passthrough                       │   │  │  │
│  │  │  └──────────────────────────────────────────────┘   │  │  │
│  │  └──────────────────────────────────────────────────────┘  │  │
│  └──────────────────────────────────────────────────────────────┘  │
└────────────────┬────────────────────────────────────────────────────┘
                 │ MAVLink Protocol (TCP/UDP/Serial)
                 ▼
┌─────────────────────────────────────────────────────────────────────┐
│                      ArduPilot Autopilot                            │
│                      (Flight Controller)                            │
└─────────────────────────────────────────────────────────────────────┘
```

### Component Layers

1. **MOOS Layer**: Interface with MOOS community
2. **Application Layer**: ArduBridge main application logic
3. **Control Layer**: SetpointManager, WarningSystem, State Machine
4. **UAV Interface Layer**: UAV_Model with threading and command queuing
5. **Communication Layer**: MAVSDK with MAVLink protocol
6. **Hardware/Simulator Layer**: ArduPilot autopilot

---

## Component Overview

### 1. ArduBridge (Main Application Class)

**File**: `ArduBridge.h` / `ArduBridge.cpp`

**Responsibilities**:
- Inherits from `AppCastingMOOSApp` for MOOS integration
- Manages MOOS variable subscriptions and publications
- Implements autopilot helm state machine
- Coordinates high-level operations (takeoff, waypoint navigation, RTL, loiter)
- Handles asynchronous command execution with promise/future pattern
- Visualizes vehicle status in pMarineViewer

**Key Members**:
```cpp
class ArduBridge : public AppCastingMOOSApp
{
    // Core Components
    UAV_Model m_uav_model;
    ThreadSafeVariable<SetpointManager> m_helm_desiredValues;
    std::shared_ptr<WarningSystem> m_warning_system_ptr;
    CMOOSGeodesy m_geodesy;
    
    // State Management
    AutopilotHelmMode m_autopilot_mode;
    
    // Command Flags
    bool m_do_fly_to_waypoint;
    bool m_do_takeoff;
    bool m_do_return_to_launch;
    std::pair<bool, std::string> m_do_loiter_pair;
    std::pair<bool, double> m_do_change_speed_pair;
    std::pair<bool, double> m_do_change_course_pair;
    std::pair<bool, double> m_do_change_altitude_pair;
};
```

**Key Methods**:
- `OnNewMail()`: Process incoming MOOS messages
- `Iterate()`: Main application loop
- `OnStartUp()`: Initialize and connect to autopilot
- `sendDesiredValuesToUAV()`: Send setpoints to UAV
- `postTelemetryUpdate()`: Publish telemetry to MOOS
- `goToHelmMode()`: Manage helm state transitions

---

### 2. UAV_Model (UAV Interface Class)

**File**: `UAV_Model.h` / `UAV_Model.cpp`

**Responsibilities**:
- Manages connection to ArduPilot via MAVSDK
- Runs separate command sender thread
- Processes command queue
- Subscribes to telemetry streams
- Polls autopilot parameters
- Executes MAVLink commands
- Maintains thread-safe telemetry state

**Key Members**:
```cpp
class UAV_Model
{
    // MAVSDK Components
    std::shared_ptr<mavsdk::Mavsdk> m_mavsdk_ptr;
    std::shared_ptr<mavsdk::System> m_system_ptr;
    std::shared_ptr<mavsdk::Action> m_action_ptr;
    std::shared_ptr<mavsdk::Telemetry> m_telemetry_ptr;
    std::shared_ptr<mavsdk::Mission> m_mission_ptr;
    std::shared_ptr<mavsdk::Param> m_param_ptr;
    
    // Threading
    std::thread m_thread;
    std::atomic<bool> m_running;
    std::queue<std::unique_ptr<CommandBase>> m_command_queue;
    std::mutex m_queue_mutex;
    std::condition_variable m_thread_cv;
    
    // Telemetry State (Thread-Safe)
    ThreadSafeVariable<mavsdk::Telemetry::Position> mts_position;
    ThreadSafeVariable<mavsdk::Telemetry::EulerAngle> mts_attitude_ned;
    ThreadSafeVariable<mavsdk::Telemetry::VelocityNed> mts_velocity_ned;
    ThreadSafeVariable<mavsdk::Telemetry::Battery> mts_battery;
    ThreadSafeVariable<mavsdk::Telemetry::FlightMode> mts_flight_mode;
};
```

**Key Methods**:
- `connectToUAV()`: Establish connection to autopilot
- `subscribeToTelemetry()`: Subscribe to telemetry streams
- `startCommandSender()`: Start command sender thread
- `runCommandsender()`: Command sender thread main loop
- `pushCommand()`: Add command to queue (thread-safe)
- `commandGuidedMode()`: Switch to guided mode
- `commandAndSetAirSpeed()`: Set airspeed
- `commandAndSetCourse()`: Set course
- `commandAndSetAltitudeAGL()`: Set altitude
- `commandGoToLocationXY()`: Fly to waypoint
- `commandLoiterAtPos()`: Loiter at position
- `commandReturnToLaunchAsync()`: Return to launch

---

### 3. SetpointManager (Setpoint Management)

**File**: `SetpointManager.h`

**Responsibilities**:
- Thread-safe storage of desired values (speed, course, altitude)
- Tracks changes in setpoints
- Provides polling interface that returns changed values only

**Key Methods**:
```cpp
class SetpointManager
{
    void updateDesiredSpeed(double speed);
    void updateDesiredCourse(double course);
    void updateDesiredAltitude(double altitude);
    
    // Returns value only if changed since last poll
    std::optional<double> getDesiredSpeed();
    std::optional<double> getDesiredCourse();
    std::optional<double> getDesiredAltitude();
    
    // Always returns current value
    double readDesiredSpeed() const;
    double readDesiredCourse() const;
    double readDesiredAltitudeAGL() const;
};
```

---

### 4. WarningSystem (Warning and Error Management)

**File**: `WarningSystem.h`

**Responsibilities**:
- Monitor conditions and raise warnings
- Time-based warning management
- Automatic warning retraction when condition clears
- Integration with MOOS warning system

**Key Methods**:
```cpp
class WarningSystem
{
    void queue_monitorWarningForXseconds(const std::string& key, double seconds);
    void queue_monitorCondition(const std::string& key, 
                                std::function<bool()> condition,
                                CallbackType reportCallback,
                                CallbackType retractCallback);
    void checkConditions();  // Called periodically
};
```

---

## Threading Model

### Thread Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                    Main Thread (MOOS Loop)                          │
│  ┌──────────────────────────────────────────────────────────────┐  │
│  │  ArduBridge::Iterate() - Called at AppTick rate             │  │
│  │                                                              │  │
│  │  - Process incoming MOOS mail (OnNewMail)                   │  │
│  │  - Update SetpointManager (thread-safe)                     │  │
│  │  - Process command flags (takeoff, waypoint, RTL, loiter)   │  │
│  │  - Check async operation status (future polling)            │  │
│  │  - Update autopilot helm state machine                      │  │
│  │  - Check warning conditions                                 │  │
│  │  - Publish telemetry to MOOS                                │  │
│  │  - Build and post AppCast report                            │  │
│  └──────────────────────────────────────────────────────────────┘  │
└────────────────┬────────────────────────────────────────────────────┘
                 │ Thread-Safe Communication
                 │ (ThreadSafeVariable, Command Queue)
                 ▼
┌─────────────────────────────────────────────────────────────────────┐
│          UAV_Model Command Sender Thread                            │
│  ┌──────────────────────────────────────────────────────────────┐  │
│  │  UAV_Model::runCommandsender()                               │  │
│  │                                                              │  │
│  │  Initialization:                                             │  │
│  │    - Subscribe to telemetry                                  │  │
│  │    - Poll autopilot parameters                               │  │
│  │                                                              │  │
│  │  Main Loop:                                                  │  │
│  │    - Wait for commands or periodic send signal               │  │
│  │    - Execute commands from queue                             │  │
│  │    - Send desired values periodically (if enabled)           │  │
│  │    - Poll parameters after command execution                 │  │
│  │    - Update flight mode state                                │  │
│  └──────────────────────────────────────────────────────────────┘  │
└────────────────┬────────────────────────────────────────────────────┘
                 │ MAVLink Commands
                 ▼
         ArduPilot Autopilot
```

### Thread Synchronization

**Thread-Safe Components**:
1. **Command Queue**: Protected by `m_queue_mutex`, signaled by `m_thread_cv`
2. **SetpointManager**: Wrapped in `ThreadSafeVariable<T>` template
3. **Telemetry State**: All telemetry members use `ThreadSafeVariable<T>`

**Synchronization Primitives**:
- `std::mutex`: Protects command queue and shared data
- `std::condition_variable`: Signals command availability
- `std::atomic<bool>`: Thread running flag and enable flags
- `ThreadSafeVariable<T>`: Template class providing mutex-protected access

---

## Data Flow

### 1. Command Flow (Helm to UAV)

```
┌─────────────┐
│  pHelmIvp   │ Publishes DESIRED_HEADING, DESIRED_SPEED, DESIRED_ALTITUDE
└──────┬──────┘
       │
       ▼
┌─────────────────────────────────────────────────┐
│  ArduBridge::OnNewMail()                        │
│  - Receives MOOS messages                       │
│  - Updates m_helm_desiredValues                 │
└──────┬──────────────────────────────────────────┘
       │
       ▼
┌─────────────────────────────────────────────────┐
│  ArduBridge::Iterate()                          │
│  - Checks helm state                            │
│  - Enables/disables sendDesiredValues           │
└──────┬──────────────────────────────────────────┘
       │
       ▼
┌─────────────────────────────────────────────────┐
│  UAV_Model::runCommandsender() [Thread]        │
│  - Wakes up periodically                        │
│  - Calls sendDesiredValuesFunction              │
└──────┬──────────────────────────────────────────┘
       │
       ▼
┌─────────────────────────────────────────────────┐
│  ArduBridge::sendDesiredValuesToUAV()           │
│  - Polls SetpointManager for changed values     │
│  - Calls UAV_Model command methods              │
└──────┬──────────────────────────────────────────┘
       │
       ▼
┌─────────────────────────────────────────────────┐
│  UAV_Model command methods                      │
│  - commandAndSetCourse()                        │
│  - commandAndSetAirSpeed()                      │
│  - commandAndSetAltitudeAGL()                   │
└──────┬──────────────────────────────────────────┘
       │
       ▼
┌─────────────────────────────────────────────────┐
│  MAVSDK Plugins                                 │
│  - Send MAVLink commands                        │
└──────┬──────────────────────────────────────────┘
       │
       ▼
┌─────────────┐
│ ArduPilot   │ Executes commands
└─────────────┘
```

### 2. Telemetry Flow (UAV to MOOS)

```
┌─────────────┐
│ ArduPilot   │ Streams telemetry via MAVLink
└──────┬──────┘
       │
       ▼
┌─────────────────────────────────────────────────┐
│  MAVSDK Telemetry Plugin                        │
│  - Receives MAVLink messages                    │
│  - Invokes subscription callbacks               │
└──────┬──────────────────────────────────────────┘
       │
       ▼
┌─────────────────────────────────────────────────┐
│  UAV_Model telemetry callbacks                  │
│  - Updates ThreadSafeVariable members           │
│  - mts_position, mts_attitude_ned, etc.         │
└──────┬──────────────────────────────────────────┘
       │
       ▼
┌─────────────────────────────────────────────────┐
│  ArduBridge::Iterate()                          │
│  - Reads telemetry from UAV_Model               │
│  - Converts coordinates (Lat/Lon to X/Y)        │
│  - Calls postTelemetryUpdate()                  │
└──────┬──────────────────────────────────────────┘
       │
       ▼
┌─────────────────────────────────────────────────┐
│  ArduBridge::postTelemetryUpdate()              │
│  - Publishes NAV_X, NAV_Y, NAV_LAT, NAV_LON,    │
│    NAV_SPEED, NAV_HEADING, NAV_ALTITUDE, etc.   │
└──────┬──────────────────────────────────────────┘
       │
       ▼
┌──────────────────┐
│  MOOS Community  │ Receives telemetry updates
└──────────────────┘
```

### 3. Direct Command Flow

```
┌──────────────────┐
│ pMarineViewer or │ Publishes FLY_WAYPOINT, LOITER, RTL, etc.
│  Other MOOS App  │
└────────┬─────────┘
         │
         ▼
┌─────────────────────────────────────────────────┐
│  ArduBridge::OnNewMail()                        │
│  - Sets command flags (m_do_fly_to_waypoint,    │
│    m_do_loiter_pair, m_do_return_to_launch)     │
└──────┬──────────────────────────────────────────┘
       │
       ▼
┌─────────────────────────────────────────────────┐
│  ArduBridge::Iterate()                          │
│  - Checks command flags                         │
│  - Initiates async operations                   │
└──────┬──────────────────────────────────────────┘
       │
       ▼
┌─────────────────────────────────────────────────┐
│  Async Command Functions                        │
│  - flyToWaypoint_async()                        │
│  - loiterAtPos_async()                          │
│  - rtl_async()                                  │
└──────┬──────────────────────────────────────────┘
       │
       ▼
┌─────────────────────────────────────────────────┐
│  UAV_Model::pushCommand() [Lambda]              │
│  - Adds command to queue                        │
│  - Signals condition variable                   │
└──────┬──────────────────────────────────────────┘
       │
       ▼
┌─────────────────────────────────────────────────┐
│  UAV_Model::runCommandsender() [Thread]        │
│  - Dequeues command                             │
│  - Executes command                             │
│  - Sets promise result                          │
└──────┬──────────────────────────────────────────┘
       │
       ▼
┌─────────────────────────────────────────────────┐
│  ArduBridge::Iterate() [Next iteration]         │
│  - Polls future for result                      │
│  - Updates helm state based on result           │
│  - Reports warnings if failed                   │
└─────────────────────────────────────────────────┘
```

---

## State Machine

### Autopilot Helm State Machine

The ArduBridge manages an autopilot helm state machine that coordinates between the MOOS helm and the ArduPilot autopilot.

```
                    ┌──────────────┐
                    │ HELM_PARKED  │ (Initial state)
                    └──────┬───────┘
                           │ Connect & Setup
                           ▼
                    ┌──────────────┐
          ┌────────▶│HELM_INACTIVE │◀────────┐
          │         └──────┬───────┘         │
          │                │                 │
          │                │ LOITER command  │
          │                ▼                 │
          │         ┌─────────────────────┐  │
          │         │HELM_INACTIVE_       │  │
          │         │  LOITERING          │  │
          │         └─────────────────────┘  │
          │                                  │
          │ HELM_STATUS = off                │ Command fails
          │                                  │
          │                                  │
          │ HELM_STATUS = on                 │
          │         ┌──────────────┐         │
          └─────────┤ HELM_ACTIVE  │─────────┘
                    └──────┬───────┘
                           │
          ┌────────────────┼────────────────┐
          │                │                │
          │ FLY_WAYPOINT   │ RTL            │ SURVEY
          ▼                ▼                ▼
    ┌──────────┐    ┌──────────┐    ┌──────────────┐
    │  HELM_   │    │  HELM_   │    │   HELM_      │
    │ TOWAYPT  │    │RETURNING │    │ SURVEYING    │
    └──────────┘    └──────────┘    └──────────────┘
```

### State Descriptions

| State | Description | Entry Conditions | Exit Conditions |
|-------|-------------|------------------|-----------------|
| `HELM_PARKED` | Initial inactive state | Application start | Successful connection and setup |
| `HELM_INACTIVE` | Helm off, manual override active | HELM_STATUS = off | HELM_STATUS = on or LOITER command |
| `HELM_INACTIVE_LOITERING` | Loitering with helm off | LOITER command while helm off | HELM_STATUS = on |
| `HELM_ACTIVE` | Helm active but idle | HELM_STATUS = on, no commands | Specific command received or helm off |
| `HELM_TOWAYPT` | Navigating to waypoint | FLY_WAYPOINT command | Waypoint reached or command fails |
| `HELM_RETURNING` | Returning to launch | RTL command | Home reached or command fails |
| `HELM_SURVEYING` | Executing survey pattern | SURVEY command | Survey complete or command fails |
| `HELM_VORONOI` | Executing Voronoi pattern | VORONOI command | Pattern complete or command fails |

---

## Class Relationships

### Class Diagram

```
┌──────────────────────────┐
│  AppCastingMOOSApp       │
│  (MOOS-IVP Base Class)   │
└────────────┬─────────────┘
             │ inherits
             ▼
┌──────────────────────────────────────────────────────────┐
│  ArduBridge                                              │
├──────────────────────────────────────────────────────────┤
│  - m_uav_model : UAV_Model                               │
│  - m_helm_desiredValues : ThreadSafeVariable<SetpointMgr>│
│  - m_warning_system_ptr : shared_ptr<WarningSystem>     │
│  - m_geodesy : CMOOSGeodesy                              │
│  - m_autopilot_mode : AutopilotHelmMode                  │
├──────────────────────────────────────────────────────────┤
│  + OnNewMail()                                           │
│  + Iterate()                                             │
│  + OnStartUp()                                           │
│  + sendDesiredValuesToUAV()                              │
│  + postTelemetryUpdate()                                 │
│  + goToHelmMode()                                        │
└──────┬───────────────────────────────┬───────────────────┘
       │ owns                          │ owns
       │                               │
       ▼                               ▼
┌─────────────────────────┐    ┌──────────────────────┐
│  UAV_Model              │    │  SetpointManager     │
├─────────────────────────┤    ├──────────────────────┤
│  - m_mavsdk_ptr         │    │  - desiredSpeed      │
│  - m_system_ptr         │    │  - desiredCourse     │
│  - m_action_ptr         │    │  - desiredAltitude   │
│  - m_telemetry_ptr      │    ├──────────────────────┤
│  - m_thread             │    │  + updateDesired*()  │
│  - m_command_queue      │    │  + getDesired*()     │
│  - mts_position         │    │  + readDesired*()    │
│  - mts_attitude_ned     │    └──────────────────────┘
│  - mts_velocity_ned     │
├─────────────────────────┤
│  + connectToUAV()       │
│  + subscribeToTelemetry│
│  + startCommandSender() │
│  + pushCommand()        │
│  + command*() methods   │
└──────┬──────────────────┘
       │ owns
       ▼
┌─────────────────────────┐
│  WarningSystem          │
├─────────────────────────┤
│  - monitoredConditions  │
│  - warningsActive       │
│  - timeBasedWarnings    │
├─────────────────────────┤
│  + queue_monitorWarning│
│  + queue_monitorCondition│
│  + checkConditions()    │
└─────────────────────────┘
```

---

## Sequence Diagrams

### Startup Sequence

```
MOOS      ArduBridge    UAV_Model    MAVSDK    ArduPilot
  │           │             │           │           │
  │ OnStartUp │             │           │           │
  ├──────────►│             │           │           │
  │           │ connectToUAV│           │           │
  │           ├────────────►│           │           │
  │           │             │  connect  │           │
  │           │             ├──────────►│           │
  │           │             │           │  MAVLink  │
  │           │             │           ├──────────►│
  │           │             │           │◄──────────┤
  │           │             │◄──────────┤           │
  │           │◄────────────┤           │           │
  │           │             │           │           │
  │           │ setUpMission│           │           │
  │           ├────────────►│           │           │
  │           │◄────────────┤           │           │
  │           │             │           │           │
  │           │startCommandSender       │           │
  │           ├────────────►│           │           │
  │           │             │ [Thread]  │           │
  │           │             │ start     │           │
  │           │             │           │           │
  │           │             │subscribeToTelemetry   │
  │           │             ├──────────►│           │
  │           │             │           │           │
  │           │             │pollAllParams          │
  │           │             ├──────────►│           │
  │           │             │           │           │
```

### Command Execution Sequence (Helm Commands)

```
pHelmIvp  MOOS  ArduBridge    SetpointMgr  UAV_Thread  UAV_Model  ArduPilot
   │        │        │              │           │           │          │
   │ Publish│        │              │           │           │          │
   │ DESIRED│        │              │           │           │          │
   │ VALUES │        │              │           │           │          │
   ├───────►│        │              │           │           │          │
   │        │OnNewMail              │           │           │          │
   │        ├───────►│              │           │           │          │
   │        │        │ update       │           │           │          │
   │        │        ├─────────────►│           │           │          │
   │        │        │              │           │           │          │
   │        │Iterate │              │           │           │          │
   │        ├───────►│              │           │           │          │
   │        │        │enable        │           │           │          │
   │        │        │sendValues────┼──────────►│           │          │
   │        │        │              │  [Wakeup] │           │          │
   │        │        │              │           │           │          │
   │        │        │              │           │sendDesiredValues    │
   │        │        │◄─────────────┼───────────┤           │          │
   │        │        │get changed   │           │           │          │
   │        │        ├─────────────►│           │           │          │
   │        │        │◄─────────────┤           │           │          │
   │        │        │              │           │           │          │
   │        │        │ command*()   │           │           │          │
   │        │        ├──────────────┼───────────┼──────────►│          │
   │        │        │              │           │           │ MAVLink  │
   │        │        │              │           │           ├─────────►│
   │        │        │              │           │           │          │
```

### Direct Command Sequence (e.g., FLY_WAYPOINT)

```
GCS/User  MOOS  ArduBridge  UAV_Model  UAV_Thread  ArduPilot
   │        │        │          │           │           │
   │ FLY_   │        │          │           │           │
   │ WAYPOINT        │          │           │           │
   ├───────►│        │          │           │           │
   │        │OnNewMail          │           │           │
   │        ├───────►│          │           │           │
   │        │        │ m_do_fly │           │           │
   │        │        │ =true    │           │           │
   │        │        │          │           │           │
   │        │Iterate │          │           │           │
   │        ├───────►│          │           │           │
   │        │        │ flyToWaypoint_async  │           │
   │        │        │          │           │           │
   │        │        │pushCommand(lambda)   │           │
   │        │        ├─────────►│           │           │
   │        │        │          │ [Enqueue] │           │
   │        │        │          │           │           │
   │        │        │          │  [Wakeup] │           │
   │        │        │          │◄──────────┤           │
   │        │        │          │ Dequeue & │           │
   │        │        │          │ Execute   │           │
   │        │        │          │           │           │
   │        │        │          │commandGoToLocation    │
   │        │        │          ├───────────┼──────────►│
   │        │        │          │           │  MAVLink  │
   │        │        │          │◄──────────┼───────────┤
   │        │        │          │set promise│           │
   │        │        │          │result     │           │
   │        │        │          │           │           │
   │        │Iterate │          │           │           │
   │        ├───────►│ poll     │           │           │
   │        │        │ future   │           │           │
   │        │        │ [result  │           │           │
   │        │        │  ready]  │           │           │
   │        │        │          │           │           │
   │        │        │goToHelmMode          │           │
   │        │        │(TOWAYPT) │           │           │
   │        │        │          │           │           │
```

---

## Design Patterns Used

### 1. Command Pattern
- Commands are encapsulated as lambda functions and queued
- UAV_Model command sender thread executes commands from queue
- Enables asynchronous, thread-safe command execution

### 2. Producer-Consumer Pattern
- Main thread (producer) pushes commands to queue
- Command sender thread (consumer) processes queue
- Condition variable for efficient waiting

### 3. Promise-Future Pattern
- Asynchronous operations return results via std::promise/std::future
- Main thread can poll for completion without blocking
- Used for flyToWaypoint, loiter, RTL operations

### 4. Observer Pattern (Callbacks)
- MAVSDK telemetry uses callbacks for streaming data
- WarningSystem uses callbacks for reporting
- UAV_Model provides callback registration for MOOS trace/events

### 5. State Machine Pattern
- AutopilotHelmMode enum with state transition map
- Validates state transitions
- Executes transition-specific logic

---

## Thread Safety Mechanisms

### ThreadSafeVariable<T> Template

```cpp
template<typename T>
class ThreadSafeVariable {
    T value;
    mutable std::mutex mutex;
    
public:
    void set(const T& v) {
        std::lock_guard<std::mutex> lock(mutex);
        value = v;
    }
    
    T get() const {
        std::lock_guard<std::mutex> lock(mutex);
        return value;
    }
};
```

Used for:
- SetpointManager wrapper
- All telemetry state variables
- Flight mode state
- Position coordinates

### Command Queue Synchronization

```cpp
void pushCommand(Command&& cmd) {
    {
        std::lock_guard lock(m_queue_mutex);
        m_command_queue.push(std::make_unique<CommandWrapper>(std::forward<Command>(cmd)));
    }
    m_thread_cv.notify_one();
}

// In command sender thread
std::unique_lock lock(m_queue_mutex);
m_thread_cv.wait(lock, [this]() { 
    return !m_command_queue.empty() || !m_running || m_sendValuesEnabled; 
});
```

---

## Performance Considerations

### Timing Parameters

- **AppTick**: Typically 4 Hz (250ms period) - Rate of Iterate() calls
- **CommsTick**: Typically 4 Hz - Rate of MOOS communication
- **Telemetry Rate**: Controlled by ArduPilot (typically 1-10 Hz per stream)
- **Command Send Rate**: Configurable, typically when setpoints change or periodically

### Bottlenecks and Optimizations

1. **Command Queue**: Lock-free only at critical sections
2. **Telemetry Updates**: Use ThreadSafeVariable to avoid blocking main thread
3. **SetpointManager**: Only sends changed values, not all values every iteration
4. **Async Commands**: Non-blocking with future polling prevents iteration delays

---

## Future Enhancements

Potential areas for evolution:

1. **Multi-UAV Support**: Managing multiple UAV_Model instances
2. **Enhanced Telemetry**: Additional sensor streams (camera, rangefinder)
3. **Mission Planning**: Dynamic mission upload and modification
4. **Geofencing**: Safety boundaries and return-to-safe-area
5. **Swarm Behaviors**: Coordination between multiple UAVs
6. **Advanced Autonomy**: Integration with computer vision and AI

---

## References

- ArduBridge source code: `src/pArduBridge/ArduBridge.cpp` and `ArduBridge.h`
- UAV_Model source code: `src/pArduBridge/UAV_Model.cpp` and `UAV_Model.h`
- MAVSDK Documentation: https://mavsdk.mavlink.io/
- MAVLink Protocol: https://mavlink.io/
- MOOS-IVP: https://oceanai.mit.edu/moos-ivp/

---

## Credits

**Author**: Steve Carter Feujo Nomeny  
**Organization**: NTNU, MIT  
**Date**: September 9th, 2024
