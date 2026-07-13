# UAV ArduPilot Command Reference

`ARDU_COMMAND` is pArduBridge's main explicit-command variable. Commands may be posted locally, bridged from shoreside, emitted by a behavior flag, or exposed as a pMarineViewer/pRealm control.

See the [canonical pArduBridge reference](pArduBridge/README.md) for configuration, policies, telemetry, and lifecycle details.

## Command summary

| Command | Plane | Copter | Control/result |
|---|---|---|---|
| `DO_TAKEOFF` | Starts the current bridge/SITL mission; vehicle must already be armed. | MAVSDK takeoff to configured altitude; vehicle must be armed with fresh `ON_GROUND` telemetry. | Plane confirms Mission/Takeoff mode. Copter reports `SUBMITTED → ACCEPTED → CONFIRMED → COMPLETED` at target altitude. |
| `FLY_WAYPOINT` | Guided reposition to `NEXT_WAYPOINT`, or MOOS `TOWAYPT_UPDATE` with Helm active. | Same position-target path. | Reports rejection, submission, and target acceptance/failure. Arrival still requires navigation telemetry. |
| `RETURN_TO_LAUNCH`, `RETURN` | Native RTL with Helm inactive; MOOS return waypoint with Helm active. | Same split. | Native RTL requires armed state and 0.5 s stable RTL telemetry; MOOS return reports `MOOS_RETURN_WAYPOINT`. |
| `LOITER` | Guided orbit around the chosen coordinate. | Guided move-to/hold at the chosen coordinate. | Reports Guided-target acceptance/failure; bridge state `HELM_INACTIVE_LOITERING`. |
| `LOITER_FC` | Native ArduPlane Loiter orbit at current point. | Native ArduCopter Loiter position hold. | `SUBMITTED → ACCEPTED → CONFIRMED`; repeat is `NO_OP`. |
| `PRECISION_LOITER` | Rejected `COPTER_ONLY`. | Native Loiter plus ArduPilot auxiliary function 39. | Requires armed, `PLND_ENABLED=1`, nonzero `PLND_TYPE`; Loiter telemetry confirmed. |
| `PRECISION_LOITER_OFF` | Rejected `COPTER_ONLY`. | Disables auxiliary function 39. | `SUBMITTED → ACCEPTED`; no durable FC enabled-state bit is available. |
| `AUTOLAND` | Native Plane AUTOLAND. | Native Copter LAND. | Conservative LAND policy; submission/acceptance reported. |
| `RESET_SPEED_MIN` | Commands polled minimum airspeed and optional groundspeed. | Commands minimum groundspeed. | Warning/event reporting. |
| `SURVEY` | Requires Helm active; enters Guided and bridge survey state. | Same. | MOOS/Helm-owned behavior. |
| `DO_VORONOI` | Requires Helm active; enters Guided and bridge Voronoi state. | Same. | MOOS/Helm-owned behavior. |
| `VIZ_HOME` | Republishes a valid home marker. | Same. | No FC command; unavailable home is explicitly rejected. |

Accepted aliases are `PRECISION_LOITER_ON`, `PRECISION_LOITER_ENABLE`, and `PRECISION_LOITER_DISABLE`.

## The three loiter operations

These names are intentionally distinct:

| Operation | FC mode | MAVLink behavior | Intended use |
|---|---|---|---|
| MOOS behavior loiter | Guided while Helm commands | Helm setpoints/behavior updates | Flexible MOOS pattern, subject to platform-control semantics. |
| `ARDU_COMMAND=LOITER` | Guided | Position target at current/stored coordinate | Legacy coordinate handoff; Plane orbits, Copter holds. |
| `ARDU_COMMAND=LOITER_FC` | Native Loiter | MAVSDK Hold/Loiter mode | Stable FC-owned hold/orbit independent of MOOS yaw commands. |

For Copter, `DESIRED_HEADING` is yaw. Yaw plus groundspeed does not itself describe an XY circle; a rich Copter MOOS loiter pattern needs appropriate horizontal position or velocity guidance.

Precision Loiter first establishes native Copter Loiter unless `precision_loiter_enter_loiter=false`, in which case the FC must already be in Loiter. It then sends auxiliary function 39. `CONFIRMED` proves the configured backend, accepted auxiliary command, and Loiter mode—not live target acquisition. Use `UAV_LANDING_TARGET_AVAILABLE`, age, source component, validity, distance, and pose as separate acquisition evidence.

## Return routing

The same request has two deliberately different paths:

```text
Helm active
  RETURN_TO_LAUNCH
    → publish home point to RETURN_UPDATE
    → MOOS behavior returns the vehicle
    → result command=MOOS_RETURN_WAYPOINT

Helm inactive
  RETURN_TO_LAUNCH
    → ArduPilot native RTL
    → confirm FC RTL telemetry
    → result command=RTL
```

Use MOOS return when the mission should retain behavior-level routing and collision/planning logic. Use native RTL as an FC-owned return/failsafe path.

## Waypoint format

Set the waypoint before issuing `FLY_WAYPOINT`:

```bash
uPokeDB alpha.moos \
  NEXT_WAYPOINT='lat=42.35855,lon=-71.08750,x=8,y=10,vname=alpha'
uPokeDB alpha.moos ARDU_COMMAND=FLY_WAYPOINT
```

All five keys are parsed. `vname` must match the configured lowercase vehicle name or be `all`. Latitude/longitude are sent to ArduPilot; X/Y are used for MOOS behavior updates and visualization.

## ARM, DISARM, and LAND

ARM/DISARM use `ARM_UAV`, not `ARDU_COMMAND`:

```bash
uPokeDB alpha.moos ARM_UAV=true
uPokeDB alpha.moos ARM_UAV=false
```

ARM requires fresh health, armable/aggregate-health OK, and fresh `ON_GROUND` telemetry. DISARM requires fresh `ON_GROUND` telemetry. The bridge never treats altitude alone as authorization to disarm.

LAND may be requested through either interface:

```bash
uPokeDB alpha.moos AUTOLAND=true
uPokeDB alpha.moos ARDU_COMMAND=AUTOLAND
```

Routine LAND/AUTOLAND is allowed from Guided, Copter Guided-as-Offboard, Mission, and native Loiter/Hold. Pilot-controlled, RTL, unknown, and unlisted modes default-deny. A request on the ground or during an existing landing is a `NO_OP`.

## Command-result interpretation

Example:

```text
id=13,command=RTL,status=SUBMITTED,detail=READY
id=13,command=RTL,status=ACCEPTED,detail=SUCCESS
id=13,command=RTL,status=CONFIRMED,detail=FLIGHT_MODE_RTL
```

- `SUBMITTED`: bridge policy allowed the command.
- `ACCEPTED`: MAVSDK/ArduPilot acknowledged it.
- `CONFIRMED`: expected FC mode telemetry persisted for 0.5 seconds, or command-specific takeoff activation evidence arrived.
- `TIMED_OUT`: expected activation or completion evidence did not arrive by its deadline.
- `REJECTED`: bridge policy prevented submission.
- `FAILED`: MAVSDK/MAVLink reported a failure after submission.
- `NO_OP`: requested terminal state was already active.
- `COMPLETED`: a defined motion objective was reached; currently Copter takeoff altitude.

Do not collapse `ACCEPTED` and `CONFIRMED`. Keep the command ID when logging a lifecycle.

Every recognized `ARDU_COMMAND` produces a result. MOOS-only or visualization commands terminate at `ACCEPTED`/`REJECTED`; they never claim an FC state that cannot be observed. Unknown tokens are `REJECTED,UNHANDLED_COMMAND`.

## Shoreside bridging

Bridge operator inputs to each vehicle using the mission's established `uFldShoreBroker`/`pShare` pattern. At minimum, expose the variables actually used:

```moos
qbridge = ARDU_COMMAND, ARM_UAV, NEXT_WAYPOINT
qbridge = RETURN_TO_LAUNCH, AUTOLAND
```

If the shoreside convention appends `_ALL` or `_<VNAME>`, confirm that the broker aliases the received value back to the vehicle-local variable name.

Example pMarineViewer controls:

```moos
button_1 = ARM_ALL      # ARM_UAV_ALL=true
button_2 = DISARM_ALL   # ARM_UAV_ALL=false
button_3 = FC_LOITER    # ARDU_COMMAND_ALL=LOITER_FC
button_4 = RTL_ALL      # ARDU_COMMAND_ALL=RETURN_TO_LAUNCH
button_5 = LAND_ALL     # ARDU_COMMAND_ALL=AUTOLAND
```

Example pRealm commands:

```moos
cmd = label=FC_LOITER, var=ARDU_COMMAND, sval=LOITER_FC, receivers=all:$(VNAMES)
cmd = label=RTL,       var=ARDU_COMMAND, sval=RETURN_TO_LAUNCH, receivers=all:$(VNAMES)
cmd = label=LAND,      var=ARDU_COMMAND, sval=AUTOLAND, receivers=all:$(VNAMES)
```

## Qualification

SITL has exercised Plane and Copter command routing, but hardware results depend on the real firmware, serial path, RC/GPS/failsafe configuration, and sensors. Use the [bench and outdoor checklist](pArduBridge/HARDWARE_QUALIFICATION.md) before flight.
