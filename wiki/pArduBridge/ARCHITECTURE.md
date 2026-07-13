# pArduBridge Architecture

## Data path

```text
pHelmIvP / operator / shoreside
        │ MOOS variables
        ▼
ArduBridge (AppCasting main thread)
  mail validation, bridge state, policies,
  publications, AppCast, visualization
        │ queued callable / setpoint state
        ▼
UAV_Model command-sender thread
  MAVSDK Action, Telemetry, MissionRaw,
  Param, and MavlinkPassthrough plugins
        │ MAVLink over UDP/TCP/serial
        ▼
ArduPilot flight controller
```

Telemetry callbacks update thread-safe state in `UAV_Model`. The MOOS `Iterate()` loop reads that state, publishes navigation/health/policy variables, polls pending mode confirmations, and drains ordered command results.

## Components

### `ArduBridge`

`ArduBridge` inherits `AppCastingMOOSApp` and owns the MOOS boundary.

- `OnNewMail()` validates inputs and records command intent or updated setpoints.
- `Iterate()` performs state transitions, dispatches blocking or queued operations, publishes telemetry, checks confirmations, and posts AppCast reports.
- `OnStartUp()` parses configuration, initializes geodesy, connects MAVSDK, registers home/mission state, and starts the command thread.
- `goToHelmMode()` changes the bridge's ownership state and posts `AUTOPILOT_MODE`.

### `UAV_Model`

`UAV_Model` owns MAVSDK and the flight-controller model.

- One `mavsdk::System` represents the discovered autopilot.
- `Action` handles ARM/DISARM, Copter takeoff, LAND/AUTOLAND, RTL, and native Hold/Loiter.
- `MissionRaw` handles startup/home mission data and Plane SITL takeoff mission behavior.
- `Telemetry` supplies position, velocity, attitude, flight mode, health, GPS, armed state, and landed state.
- `Param` reads Precision Loiter configuration; `Action` and telemetry manage target speed.
- `MavlinkPassthrough` sends Guided and ArduPilot-specific commands and requests message rates.

The command queue serializes operations that should not execute in the MOOS mail callback. MAVSDK callbacks return results to the bridge through thread-safe queues and promise/future pairs.

### `SetpointManager`

`SetpointManager` stores the latest Helm course, speed, and altitude. Changed values are sent only while the bridge state gives the Helm command ownership.

Platform distinction:

- Plane course requests use Guided course-over-ground commands.
- Copter heading requests use `MAV_CMD_CONDITION_YAW` and therefore control yaw, not XY direction.
- Copter speed is groundspeed; Plane speed is normally airspeed.

### `WarningSystem`

`WarningSystem` converts connection, policy, stale-data, and command failures into AppCasting warnings/events. Warnings are diagnostic; the authoritative machine-readable command outcome is `UAV_COMMAND_RESULT` when that command participates in the lifecycle interface.

### `ModeConfirmationTracker`

The tracker stores a command ID and acceptance time. RTL, `LOITER_FC`, and Precision Loiter wait up to five seconds for matching telemetry and require it to remain continuously matched for 0.5 seconds. This dwell filters transient intermediate modes before `CONFIRMED`; an interrupted match restarts the dwell.

`TakeoffConfirmationTracker` preserves the same ID through activation and completion. Copter activation is confirmed from FC Takeoff or landed-state telemetry; completion additionally requires `IN_AIR` and configured relative altitude within 0.5 m. Plane stops at Mission/Takeoff-mode confirmation because mission items, rather than the bridge target, define completion.

### Landing-target observation

`MavlinkPassthrough` subscribes to `LANDING_TARGET` for the connected vehicle system. The model retains source system/component, target metadata, angles, distance, position-valid, and X/Y/Z with a monotonic receive timestamp. `ArduBridge::Iterate()` publishes the observation at AppTick; availability is true only while age is at most 0.5 seconds. This is deliberately observational: target traffic does not itself enable Precision Loiter or change flight mode.

## Authority model

The bridge and flight controller are separate gatekeepers:

```text
request
  → bridge policy decision
  → MAVSDK/MAVLink acceptance
  → flight-controller telemetry confirmation
```

This separation is reflected in lifecycle states. `ACCEPTED` means the command transport/API succeeded; `CONFIRMED` means the expected telemetry was later observed. ArduPilot can reject a command after the bridge allows it because native pre-arm, EKF, GPS, RC, fence, or mode requirements are not duplicated completely in pArduBridge.

## Bridge state versus FC mode

`AUTOPILOT_MODE` is historical naming for the pArduBridge/Helm state machine. It does not mirror ArduPilot's `custom_mode`.

```text
HELM_PARKED
HELM_INACTIVE
HELM_INACTIVE_LOITERING
HELM_ACTIVE
HELM_TOWAYPT
HELM_RETURNING
HELM_SURVEYING
HELM_VORONOI
```

Examples:

- `HELM_INACTIVE_LOITERING` corresponds to the legacy Guided coordinate-hold path.
- Native Copter/Plane Loiter uses `HELM_INACTIVE` because the FC owns the operation.
- Native RTL and LAND/AUTOLAND also use `HELM_INACTIVE`.
- `HELM_RETURNING` identifies the MOOS-controlled return path, not native FC RTL.

## Command sequences

### ARM

```text
ARM_UAV=true
  → evaluate fresh health + armable + health_all_ok
  → evaluate fresh ON_GROUND landed state
  → UAV_COMMAND_RESULT SUBMITTED
  → MAVSDK arm_async
  → ACCEPTED or FAILED
```

### Native RTL or Loiter

```text
ARDU_COMMAND
  → closed mode allowlist
  → SUBMITTED
  → MAVSDK asynchronous Action command
  → ACCEPTED
  → ModeConfirmationTracker
  → CONFIRMED or TIMED_OUT
```

### Copter takeoff

```text
DO_TAKEOFF
  → armed + fresh ON_GROUND policy
  → SUBMITTED
  → configure altitude + MAVSDK takeoff
  → ACCEPTED
  → FC Takeoff/TAKING_OFF/IN_AIR telemetry → CONFIRMED
  → IN_AIR at target altitude tolerance   → COMPLETED
  → activation or completion timeout      → TIMED_OUT
```

### Precision Loiter

```text
PRECISION_LOITER
  → Copter + armed checks
  → read PLND_ENABLED and PLND_TYPE
  → allowed-mode check
  → enter native Loiter when required
  → MAV_CMD_DO_AUX_FUNCTION_ARDUPILOT, function 39 HIGH
  → ACCEPTED
  → confirm native Loiter telemetry
```

The FC does not publish a durable auxiliary-function-39 enabled bit through this implementation. Precision target acquisition is also separate and must be verified from the sensor/MAVLink path.

### Return routing

```text
RETURN request
  ├─ Helm active   → publish home to RETURN_UPDATE → HELM_RETURNING
  └─ Helm inactive → native ArduPilot RTL          → HELM_INACTIVE
```

### Loiter routing

```text
LOITER     → Guided + position target → HELM_INACTIVE_LOITERING
LOITER_FC  → native FC Hold/Loiter    → HELM_INACTIVE
```

## Telemetry and freshness

The bridge requests and consumes MAVLink telemetry through MAVSDK. Policy-relevant samples have explicit maximum ages:

- detailed health: 3 seconds for ARM;
- landed state: 2 seconds for ARM/DISARM.

Health position flags describe estimator readiness and can remain true independently of the current raw GPS fix. Operators must inspect both health and `UAV_GPS_*` quality/freshness.

Landed state originates from `EXTENDED_SYS_STATE`. It is used instead of altitude-threshold inference for ARM/DISARM decisions. LAND is intentionally allowed to proceed with a warning when landed-state telemetry is missing or stale, provided the FC mode is in the LAND allowlist.

## Concurrency rules

- MOOS callbacks do not directly perform long-running command sequences.
- The command queue is protected by a mutex and condition variable.
- telemetry and mission coordinates use `ThreadSafeVariable` wrappers;
- armed, health, freshness, and thread state use atomics;
- command results and mode-confirmation trackers have their own mutexes;
- promise/future objects hand asynchronous completion back to `Iterate()`.

The design assumes one pArduBridge instance owns one MAVSDK connection to one autopilot. Multiple GCS clients require an intentional MAVLink router/topology; competing direct serial readers are not supported.

## Verification boundary

SITL can validate software routing, policy, lifecycle, and mode mapping. It cannot validate the Pi serial device, real firmware configuration, RC/GPS behavior, motors, sensor wiring, Precision Loiter target acquisition, or flight dynamics. Those belong to the [hardware qualification plan](HARDWARE_QUALIFICATION.md).
