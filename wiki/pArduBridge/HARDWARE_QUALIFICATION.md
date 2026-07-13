# pArduBridge Hardware Qualification

This checklist qualifies the Pi-to-flight-controller connection and the current pArduBridge command contract. It separates tests that are safe and meaningful indoors from tests that require outdoor GPS/RC conditions. It is not permission to fly.

Record the date, pArduBridge commit, Pi OS, MAVSDK version, ArduPilot vehicle/firmware version, controller model, serial device, and parameter-file backup with the results.

## Stop conditions

Stop immediately and remove vehicle power if any of the following occurs:

- unexpected motor/ESC activity;
- loss of the planned RC/manual override or inability to disarm;
- an unexpected FC mode or repeated unexplained mode changes;
- conflicting software owns the serial port;
- stale or contradictory telemetry;
- unexpected vehicle movement, heating, smoke, odor, or wiring fault.

Do not disable or spoof ArduPilot pre-arm checks for qualification. A rejection is useful evidence; bypassing the check prevents validation of the real safety chain.

## Phase A — indoor bench

### Physical setup

- Remove all propellers.
- Secure the airframe.
- Isolate propulsion/ESC power when the wiring permits it while keeping the Pi and flight controller powered.
- Keep an RC transmitter available and know the disarm/kill control.
- Keep physical access to the power connector.
- Confirm the intended flight-controller serial/USB port and disconnect competing GCS applications.

### Pi and source

```bash
hostname
ip -br address
git -C ~/moos-ivp-uav-base status --short --branch
git -C ~/moos-ivp-uav-base log -1 --oneline
git -C ~/moos-ivp-uav-base pull --ff-only
cd ~/moos-ivp-uav-base
./build.sh
pArduBridge --help
pArduBridge --example
pArduBridge --interface
```

Do not pull over local modifications without reviewing them. Confirm the Pi is running the intended commit before interpreting results.

### Serial ownership and raw MAVLink

```bash
ls -l /dev/serial/by-id /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
fuser /dev/ttyACM0 2>/dev/null
```

Use one MAVLink reader at a time unless a router is deliberately configured. Establish raw evidence for:

- heartbeat system/component IDs;
- ArduPilot vehicle type and firmware version;
- armed bit and custom mode;
- `SYS_STATUS`, `GPS_RAW_INT`, `HOME_POSITION`, and `EXTENDED_SYS_STATE`;
- `STATUSTEXT` pre-arm failures;
- `LANDING_TARGET` traffic if a precision-landing sensor is expected.

### Read-only pArduBridge baseline

Launch the ordinary vehicle MOOS community with `vehicle_type`, serial endpoint, `vname`, and `prefix` explicitly configured. Before sending commands, verify:

| Evidence | Expected indoor result |
|---|---|
| Navigation | Prefixed latitude/longitude and X/Y update when the FC supplies position. |
| Health | `UAV_HEALTH_AVAILABLE=1`; individual flags match raw MAVLink/MAVSDK evidence. Position flags may remain false indoors. |
| GPS | `UAV_GPS_AVAILABLE=1` means samples exist, not that a fix exists. Fix type, satellites, DOP, and age must be plausible. |
| Landed state | Available, fresh, and `ON_GROUND`. |
| Arm policy | Usually blocked indoors if ArduPilot is not armable; reason must match observed health. |
| AppCast | Configured vehicle type and connection endpoint are correct. |
| Landmark | `UAV_LANDING_TARGET_AVAILABLE=1` only while the target is detected; source component, distance/pose, and age agree with raw MAVLink. |

### Parameter inspection

Save parameters before changing anything. Inspect at minimum:

- arming/pre-arm and safety-switch configuration;
- RC failsafe and GCS failsafe;
- serial protocol/baud for the connected port;
- RTL altitude and behavior;
- fence configuration;
- LAND/AUTOLAND-related parameters;
- `PLND_ENABLED`, `PLND_TYPE`, orientation, camera/sensor position, limits, and timeout;
- motor interlock and idle-spin behavior before any armed test.

`PRECISION_LOITER` requires `PLND_ENABLED=1` and nonzero `PLND_TYPE`. That only proves backend configuration. Separately verify `UAV_LANDING_TARGET_AVAILABLE`, age, source component, position validity, pose/distance changes, stale transition, and raw MAVLink traffic while moving or covering the target.

### Indoor command matrix

Remain disarmed unless ArduPilot naturally reports the vehicle armable and the operator explicitly authorizes a brief props-off test.

| Request | Expected bridge result | Independent FC check |
|---|---|---|
| `ARM_UAV=false` while disarmed | `DISARM,NO_OP,ALREADY_DISARMED` | Armed bit remains false. |
| `AUTOLAND` on ground | `LAND,NO_OP,ALREADY_ON_GROUND` | No mode or actuator surprise. |
| ARM with failed prerequisites | `ARM,REJECTED,<reason>` or FC `FAILED` | `STATUSTEXT` explains the native blocker. |
| `PRECISION_LOITER` while disarmed | `REJECTED,NOT_ARMED` | No auxiliary/mode activation. |
| Precision Loiter with disabled PLND | `REJECTED,PLND_DISABLED` or `PLND_TYPE_NONE` | Parameters agree. |
| Native RTL while disarmed | `REJECTED,NOT_ARMED`. | FC mode does not change. |
| `LOITER_FC` while RTL | Allowed only when armed with fresh healthy local/global/home position telemetry. | FC settles in native Loiter for at least 0.5 s. |
| Repeat `LOITER_FC` | `NO_OP,ALREADY_FC_LOITER` | FC remains in native Loiter. |
| `LOITER` | Bridge enters `HELM_INACTIVE_LOITERING` | FC is Guided and acknowledges reposition, not native Loiter. |

Some GPS-dependent modes may be rejected indoors by ArduPilot even though bridge policy permits submission. Record this as a valid FC rejection, not a bridge failure.

### Optional brief indoor ARM/DISARM

Only perform this when props are removed, propulsion is isolated or motor behavior is known, the vehicle is secured, RC/manual disarm is available, and ArduPilot passes its own checks without spoofing.

1. Confirm `UAV_ARM_POLICY_READY=1` and reason `READY`.
2. Post `ARM_UAV=true` once.
3. Verify `SUBMITTED → ACCEPTED`, FC armed bit, and no unexpected output.
4. Do not issue takeoff.
5. Post `ARM_UAV=false` within seconds.
6. Verify `SUBMITTED → ACCEPTED` and FC armed bit false.

## Phase B — outdoor, props off or propulsion isolated

Move outdoors with the normal GPS antenna orientation, RC connected, and adequate sky view. Repeat the read-only baseline until all expected prerequisites are stable.

### Required baseline

- fresh health and landed-state telemetry;
- 3D or better GPS fix with plausible satellites and DOP;
- home position established;
- RC link and failsafe state normal;
- `UAV_IS_ARMABLE=1`, `UAV_HEALTH_ALL_OK=1`, and `UAV_ARM_POLICY_READY=1`;
- raw MAVLink and pArduBridge agree.

### Positive mode/command matrix

Keep each armed state brief and return to DISARM between groups when practical.

| Operation | Expected pArduBridge evidence | Expected ArduPilot evidence |
|---|---|---|
| ARM | Same ID reaches `SUBMITTED → ACCEPTED`; armed bit true. | FC arms with no bypassed checks. |
| DISARM on ground | `SUBMITTED → ACCEPTED`; armed bit false. | FC disarms. |
| Autonomous takeoff, flight test only | `SUBMITTED → ACCEPTED → CONFIRMED → COMPLETED` with one ID. | FC reports takeoff/in-air and settles within 0.5 m of configured altitude. |
| Airborne DISARM request, simulated only or during separately controlled flight | `REJECTED,NOT_ON_GROUND`. | FC remains armed. |
| Guided waypoint | Vehicle receives the `NEXT_WAYPOINT` position target. | Guided plus accepted `DO_REPOSITION`; target coordinates/altitude correct. |
| Legacy `LOITER` | `HELM_INACTIVE_LOITERING`. | Guided coordinate hold; Plane orbits target, Copter holds target. |
| `LOITER_FC` | `SUBMITTED → ACCEPTED → CONFIRMED`. | Native Copter/Plane Loiter. |
| Precision Loiter | `SUBMITTED → ACCEPTED → CONFIRMED`. | Native Copter Loiter, aux function 39 accepted, and landing target independently observed. |
| Precision Loiter off | `SUBMITTED → ACCEPTED`. | Aux function 39 disable acknowledged. |
| MOOS return | With Helm active, `MOOS_RETURN_WAYPOINT,ACCEPTED,RETURN_UPDATE_POSTED`. | FC remains under Guided/MOOS control. |
| Native RTL | With Helm inactive, `RTL,SUBMITTED → ACCEPTED → CONFIRMED`. | FC mode is RTL and home/RTL parameters are correct. |
| LAND/AUTOLAND | Allowed only from the documented modes; `SUBMITTED → ACCEPTED`. | Copter LAND or Plane AUTOLAND. Final landed state and disarm require an actual controlled landing. |

Mode acceptance on a stationary, props-off vehicle does not validate flight dynamics or trajectory behavior.

## Phase C — controlled flight test

Treat actual takeoff, horizontal guidance, Precision Loiter target following, RTL trajectory, and landing as a separate flight-test plan with an operator, safety observer, defined test area, weather limits, abort modes, geofence, battery limits, and regulatory approval.

Recommended progression:

1. manual/RC takeoff and stable hover or circuit;
2. one short Guided waypoint;
3. native Loiter and return to pilot control;
4. MOOS-guided behavior in a bounded area;
5. Precision Loiter target acquisition without landing;
6. native RTL behavior at safe altitude;
7. LAND/AUTOLAND only after every preceding transition is understood.

## Result record

For each case record:

```text
date/time:
vehicle and firmware:
pArduBridge commit:
MOOS request:
UAV_COMMAND_RESULT sequence:
raw FC mode/armed/landed evidence:
STATUSTEXT or warning:
pass / fail / indeterminate:
notes and follow-up:
```

A test is a pass only when the MOOS request, bridge result, and independent FC observation agree. `ACCEPTED` without the expected telemetry is not a confirmed mode transition.
