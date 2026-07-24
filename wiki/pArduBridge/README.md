# pArduBridge Reference

`pArduBridge` connects one MOOS community to one ArduPilot autopilot through MAVSDK/MAVLink. It translates IvP Helm setpoints and explicit MOOS commands, publishes navigation and flight-controller telemetry, and reports safety-policy and command-lifecycle results.

It supports ArduPlane and ArduCopter. It is not Pixhawk-specific: any ArduPilot controller that provides the required MAVLink messages and commands may work. Firmware, board, port, and parameter differences must still be qualified on the actual vehicle.

Use `pArduBridge` for hardware or ArduPilot SITL. It is normally omitted when a mission uses only the lightweight MOOS vehicle simulator.

## Documentation map

- [Quick start](QUICKSTART.md)
- [Command reference](../UAV_Mission_ARDU_Commands.md)
- [Architecture](ARCHITECTURE.md)
- [Bench and outdoor qualification](HARDWARE_QUALIFICATION.md)

## Configuration

`LatOrigin` and `LongOrigin` must exist at global mission-file scope for latitude/longitude to local-X/Y conversion.

```moos
LatOrigin  = 42.358456
LongOrigin = -71.087589

ProcessConfig = pArduBridge
{
  AppTick   = 10
  CommsTick = 10

  url          = ttyACM0:115200
  url_protocol = serial
  vehicle_type = copter

  vname  = alpha
  vcolor = yellow
  prefix = UAV

  takeoff_altitude = 10
  precision_loiter_enter_loiter = true
  command_groundspeed = true
  is_sim = false
  logger = false
}
```

### Parameters

| Parameter | Accepted values | Meaning |
|---|---|---|
| `url` or `ardupiloturl` | connection address | Required MAVLink endpoint without the protocol prefix. |
| `url_protocol` | `serial`, `udp`, `tcp` | Required. For serial, `pArduBridge` prepends `serial:///dev/`; use `ttyACM0:115200`, not `/dev/ttyACM0:115200`. |
| `vehicle_type` or `vehicle` | Plane aliases: `plane`, `arduplane`, `fixedwing`, `fixed_wing`; Copter aliases: `copter`, `arducopter`, `multicopter` | Selects platform-specific behavior. The compatibility default is Plane; configure this explicitly. |
| `vname` | nonempty string | Required vehicle/community identifier; normalized to lowercase. |
| `vcolor` | color string | Marker color used for visualization. |
| `prefix` | MOOS-variable prefix | Prefix for navigation publications such as `UAV_X`. |
| `takeoff_altitude`, `default_altitude`, or `target_altitude` | positive meters AGL | Copter takeoff altitude and the default target altitude. Copter defaults to 10 m when omitted. |
| `command_groundspeed` or `cmd_gs` | boolean | Also command groundspeed. Copter forces this true; Plane normally commands airspeed. |
| `precision_loiter_enter_loiter` | boolean | Default true. If false, `PRECISION_LOITER` requires the FC to already be in native Loiter. |
| `is_sim` | boolean | For Plane SITL, false means retain the FC mission and register its home; true replaces it with the bridge's built-in simulation mission. Copter always retains the FC mission. |
| `logger` | `true` or `false` | Enables the app's auxiliary text logger. This is separate from MOOS `pLogger`. |

Common endpoints:

```moos
// SITL UDP
url          = 0.0.0.0:14550
url_protocol = udp

// Direct serial on Linux/Pi
url          = ttyACM0:115200
url_protocol = serial

// TCP
url          = 127.0.0.1:5760
url_protocol = tcp
```

## Control ownership

There are two independent authorities:

1. `pArduBridge` decides whether its policy permits a command.
2. ArduPilot independently accepts or rejects the MAVLink command and enforces its pre-arm, mode, EKF, GPS, RC, fence, and failsafe rules.

```text
MOOS request → bridge policy → MAVLink/MAVSDK result → FC telemetry confirmation
```

`AUTOPILOT_MODE` is the bridge's Helm/control state. It is not the raw ArduPilot flight mode. Use `UAV_COMMAND_RESULT`, the AppCast flight-mode field, and an independent MAVLink observer when qualifying actual FC behavior.

### Helm states

| State | Meaning |
|---|---|
| `HELM_PARKED` | Bridge starts parked; Helm is not controlling the vehicle. |
| `HELM_INACTIVE` | Helm is inactive and the FC owns the current operation, such as native Loiter, RTL, or LAND. |
| `HELM_INACTIVE_LOITERING` | Guided coordinate hold requested by `ARDU_COMMAND=LOITER` or `ARDU_COMMAND=HOLD_POSITION`. |
| `HELM_ACTIVE` | Helm enabled with no specialized bridge task. |
| `HELM_TOWAYPT` | Helm-directed waypoint behavior. |
| `HELM_RETURNING` | Helm-directed return behavior. |
| `HELM_SURVEYING` | Helm-directed survey. |
| `HELM_VORONOI` | Helm-directed Voronoi behavior. |

## MOOS inputs

### Helm setpoints

| Variable | Type | Plane | Copter |
|---|---|---|---|
| `DESIRED_HEADING` | double, degrees | Course-over-ground request in Guided. | Absolute yaw request. It does not by itself command horizontal direction of travel. |
| `DESIRED_SPEED` | double, m/s | Airspeed; optionally groundspeed too. | Groundspeed. |
| `DESIRED_ALTITUDE` | double, m AGL | Guided altitude request. | Guided altitude request. |

Because Copter interprets `DESIRED_HEADING` as yaw, a rich MOOS `BHV_Loiter` path is not automatically reproduced by yaw plus groundspeed. Copter horizontal pattern following requires position or velocity guidance appropriate to that behavior. Explicit Guided waypoints do use position targets and have been exercised in SITL.

### Direct inputs

| Variable | Format | Behavior |
|---|---|---|
| `ARM_UAV` | boolean | Requests ARM when true and DISARM when false. |
| `RETURN_TO_LAUNCH` | boolean | When true, selects MOOS return routing if the Helm is active; otherwise requests native FC RTL. |
| `AUTOLAND` | boolean | When true, requests Copter LAND or Plane AUTOLAND through the LAND policy. |
| `ARDU_COMMAND` | command string | Main explicit-command interface; see the command reference. |
| `NEXT_WAYPOINT` | `lat=...,lon=...,x=...,y=...,vname=<name|all>` | Defines the direct FC target used by `ARDU_COMMAND=FLY_WAYPOINT` when the Helm is inactive. |
| `HELM_STATUS` | boolean | Enables or disables Helm ownership. Turning it off requests legacy Guided hold at the current position. |
| `AUTOPILOT_MODE` | bridge state string | Requests a bridge-state transition; intended for mission coordination, not raw FC mode selection. |
| `MOOS_MANUAL_OVERRIDE` | string boolean | `true` parks the Helm and initiates the return path. |
| `CHANGE_SPEED` | double | Adds a speed increment to the current target. |
| `CHANGE_COURSE` | double | Adds a course increment. |
| `CHANGE_ALTITUDE` | double | Adds an altitude increment and publishes `CONST_ALTITUDE_UPDATE`. |
| `DEAD_MAN_POST_INTERRUPT` | any | Reports a dead-man warning and initiates the return path. |

`FLY_WAYPOINT`, `DO_TAKEOFF`, `LOITER`, `SURVEY`, `RESET_SPEED_MIN`, and `VIZ_HOME` are registered for compatibility, but their implemented command path is `ARDU_COMMAND=<value>`. Do not rely on posting those variable names directly.

## Explicit commands

| `ARDU_COMMAND` value | Result |
|---|---|
| `DO_TAKEOFF` | Requires an already-armed vehicle. Copter requires fresh `ON_GROUND` telemetry, invokes MAVSDK takeoff at `takeoff_altitude`, confirms takeoff telemetry, and completes at the target altitude. Plane starts its current bridge/SITL mission and confirms Mission/Takeoff mode. |
| `FLY_WAYPOINT` | With Helm active, requests Guided mode and enters `HELM_TOWAYPT` without requiring or duplicating a waypoint; the active Helm behavior owns its route. With Helm inactive, requires `NEXT_WAYPOINT` and sends that position target directly to the FC. Reports rejection, submission, and acceptance/failure; arrival is not yet a lifecycle state. |
| `RETURN_TO_LAUNCH` or `RETURN` | With Helm inactive and the vehicle armed, requests native FC RTL and confirms stable RTL telemetry. With Helm active, publishes the home point to `RETURN_UPDATE`. |
| `LOITER` | Legacy Guided coordinate hold. Copter moves to and holds the target; Plane orbits the target in Guided. Reports submission and FC target acceptance/failure. |
| `LOITER_FC` | Requests native FC Loiter: Copter position hold or Plane orbit at the current point. Confirms `Hold` telemetry. |
| `PRECISION_LOITER`, `PRECISION_LOITER_ON`, `PRECISION_LOITER_ENABLE` | Copter only. Requires armed state, `PLND_ENABLED=1`, nonzero `PLND_TYPE`, and an allowed autonomy-controlled mode. Enters native Loiter unless configured not to, then enables ArduPilot auxiliary function 39. |
| `PRECISION_LOITER_OFF`, `PRECISION_LOITER_DISABLE` | Copter only. Disables auxiliary function 39. Allowed even when disarmed or after an external mode change. |
| `AUTOLAND` | Copter invokes LAND; Plane invokes native AUTOLAND. Uses the conservative LAND policy below. |
| `RESET_SPEED_MIN` | Commands the polled minimum speed. |
| `SURVEY` | Requires Helm ownership; enters Guided and sets `HELM_SURVEYING`. |
| `DO_VORONOI` | Requires Helm ownership; enters Guided and sets `HELM_VORONOI`. |
| `VIZ_HOME` | Republishes a valid home marker and reports `REJECTED,HOME_UNAVAILABLE` when no marker can be posted; it sends no flight command. |

See [UAV ArduPilot Commands](../UAV_Mission_ARDU_Commands.md) for examples and operational distinctions.

## Safety policies

### ARM

ARM is submitted only when all of the following are true:

- detailed health telemetry is available and no more than 3 seconds old;
- MAVSDK reports `is_armable` and aggregate health OK;
- landed-state telemetry is available and no more than 2 seconds old;
- landed state is `ON_GROUND`.

ArduPilot may still reject ARM. Repeating ARM while already armed returns `NO_OP`.

### DISARM

DISARM is submitted only with fresh landed-state telemetry reporting `ON_GROUND`. Airborne, taking-off, landing, unknown, unavailable, and stale states are rejected. Repeating DISARM while already disarmed returns `NO_OP`.

### LAND/AUTOLAND

Routine LAND is allowed only from MAVSDK `Guided`, `Offboard` (ArduCopter Guided), `Mission`, or `Hold` (native Loiter). Pilot-controlled, RTL, unknown, and future unlisted modes default-deny. Already-on-ground or already-landing requests return `NO_OP`.

Unlike DISARM, LAND may proceed when landed-state telemetry is unavailable, stale, or unknown because suppressing a requested landing can be less safe. The result detail begins with `READY_...` and a warning is raised when proceeding with limited confirmation.

### Mode-changing autonomy commands

Native RTL and `LOITER_FC` use the bridge's closed autonomy-mode allowlist: Mission, Hold, Land, Guided, and Offboard. RTL additionally requires the vehicle to be armed. An explicit `LOITER_FC` may override RTL only while armed with fresh, healthy local/global/home position telemetry. Manual/Stabilized, unknown, and unlisted modes remain rejected unless the operation is already active and returns `NO_OP`.

## Command lifecycle

`UAV_COMMAND_RESULT` has this stable comma-separated format:

```text
id=<integer>,command=<token>,status=<token>,detail=<token-or-message>
```

Statuses:

| Status | Meaning |
|---|---|
| `SUBMITTED` | Bridge policy passed and command submission is beginning. |
| `ACCEPTED` | MAVSDK or ArduPilot acknowledged the command. This alone is not mode confirmation. |
| `CONFIRMED` | FC telemetry continuously reported the expected mode for 0.5 seconds, or reported the command-specific takeoff activation state. |
| `TIMED_OUT` | The expected activation or completion evidence was not observed by its deadline. |
| `REJECTED` | The bridge policy blocked submission. |
| `FAILED` | Submission occurred but MAVSDK/MAVLink reported failure. |
| `NO_OP` | The requested terminal state was already active. |
| `COMPLETED` | A motion command with a defined completion condition reached it. Currently used when Copter takeoff reaches its configured altitude. |

The same command ID is retained across a multi-stage lifecycle. Copter takeoff uses `SUBMITTED → ACCEPTED → CONFIRMED → COMPLETED`; activation must be observed within 5 seconds and target altitude within 60 seconds. Completion requires FC `IN_AIR` telemetry and relative altitude at least `takeoff_altitude - 0.5 m`. Plane takeoff confirms Mission/Takeoff mode but does not claim altitude completion because its uploaded mission owns that definition.

Every recognized `ARDU_COMMAND` now produces `UAV_COMMAND_RESULT`. Commands that only update MOOS or visualization state report an honest terminal `ACCEPTED`/`REJECTED`; they do not claim FC confirmation. `FLY_WAYPOINT` and legacy `LOITER` report whether the Guided target was accepted, not physical arrival. Unknown command tokens report `REJECTED,UNHANDLED_COMMAND`.

Precision Loiter `CONFIRMED` means the PLND parameters were configured, auxiliary function 39 was accepted, and FC Loiter telemetry was observed. It does not prove target acquisition; use the separately observed `UAV_LANDING_TARGET_*` freshness and pose publications for that evidence.

## Publications

### Prefixed navigation variables

If `prefix=UAV`, the app publishes:

| Variable | Meaning |
|---|---|
| `UAV_LAT`, `UAV_LON` | WGS84 position in degrees. |
| `UAV_X`, `UAV_Y` | Local-grid position in meters using `LatOrigin`/`LongOrigin`. |
| `UAV_HEADING` | GPS course over ground in degrees. |
| `UAV_SPEED` | GPS speed over ground in m/s. |
| `UAV_ALTITUDE` | Relative altitude in meters. |
| `UAV_DEPTH` | Negative relative altitude for MOOS compatibility. |

### Unprefixed health, GPS, and policy variables

| Variable family | Variables |
|---|---|
| Health | `UAV_HEALTH_AVAILABLE`, `UAV_HEALTH_GYRO`, `UAV_HEALTH_ACCEL`, `UAV_HEALTH_MAG`, `UAV_HEALTH_LOCAL_POSITION`, `UAV_HEALTH_GLOBAL_POSITION`, `UAV_HEALTH_HOME_POSITION`, `UAV_IS_ARMABLE`, `UAV_HEALTH_ALL_OK`, `UAV_HEALTH_AGE` |
| Vehicle state | `UAV_IS_ARMED`, `UAV_LANDED_STATE_AVAILABLE`, `UAV_LANDED_STATE`, `UAV_LANDED_STATE_AGE` |
| GPS | `UAV_GPS_AVAILABLE`, `UAV_GPS_FIX_TYPE`, `UAV_GPS_SATELLITES`, `UAV_GPS_HDOP`, `UAV_GPS_VDOP`, `UAV_GPS_AGE` |
| Landing target | `UAV_LANDING_TARGET_AVAILABLE`, `UAV_LANDING_TARGET_AGE`, source system/component IDs, target number/frame/type, position-valid flag, angles, distance, and X/Y/Z |
| Policy | `UAV_ARM_POLICY_READY`, `UAV_ARM_POLICY_REASON`, `UAV_DISARM_POLICY_READY`, `UAV_DISARM_POLICY_REASON`, `UAV_LAND_POLICY_READY`, `UAV_LAND_POLICY_REASON` |
| Command | `UAV_COMMAND_RESULT` |

GPS fix types use the MAVSDK enum: 0 No GPS, 1 No Fix, 2 Fix 2D, 3 Fix 3D, 4 DGPS, 5 RTK float, and 6 RTK fixed. Health position flags are estimator-health flags; they are not a substitute for checking GPS fix, satellites, DOP, and freshness.

Landed states are `UNKNOWN`, `ON_GROUND`, `IN_AIR`, `TAKING_OFF`, and `LANDING`. They come from FC `EXTENDED_SYS_STATE` through MAVSDK and are authoritative for bridge ARM/DISARM policy, subject to availability and freshness.

`UAV_LANDING_TARGET_AVAILABLE=1` means a `LANDING_TARGET` message from the vehicle's MAVLink system was received within the last 0.5 seconds. It becomes zero when traffic goes stale. Distance and X/Y/Z are reported exactly as supplied; inspect `UAV_LANDING_TARGET_POSITION_VALID` before treating the position fields as valid. These variables are observational and do not automatically change Precision Loiter policy or flight mode.

### Coordination and visualization

The app also publishes `AUTOPILOT_MODE`, `MOOS_MANUAL_OVERRIDE`, `RETURN_UPDATE`, `TOWAYPT_UPDATE`, `SURVEY_UPDATE`, `CONST_ALTITUDE_UPDATE`, and `VIEW_*` markers/vectors.

## Verification status

The current Plane/Copter implementation has completed a one-time logged SITL regression covering health and stale telemetry, landed-state transitions, ARM/DISARM, LAND policy, command lifecycles, MOOS return versus native RTL, Guided versus native Loiter, Precision Loiter readiness, routed `LANDING_TARGET` freshness/pose telemetry, Copter waypoint movement, Plane mission takeoff, and Plane AUTOLAND. No permanent SITL harness or CTest targets are kept in the app directory.

SITL does not qualify the real serial path, controller firmware, RC/GPS configuration, propulsion wiring, PLND sensor, or target acquisition. Follow the [hardware qualification checklist](HARDWARE_QUALIFICATION.md) before flight.

## Command-line inspection

```bash
pArduBridge --help
pArduBridge --example
pArduBridge --interface
```

Build from the repository root with `./build.sh`, or configure/build the existing CMake project according to the repository setup documentation.
