# pArduBridge Quick Start

This guide brings up one pArduBridge instance against ArduPilot SITL or a directly attached flight controller. Read the [full reference](README.md) before commanding hardware and use the [hardware qualification checklist](HARDWARE_QUALIFICATION.md) for the Pi/Pixhawk bench.

## 1. Build

```bash
cd ~/moos-ivp-uav-base
./build.sh
which pArduBridge
pArduBridge --help
pArduBridge --example
pArduBridge --interface
```

Ensure the repository `bin` directory is on `PATH` if `which` cannot find the app.

## 2. Choose one MAVLink endpoint

SITL UDP:

```moos
url          = 0.0.0.0:14550
url_protocol = udp
```

Linux/Pi serial:

```moos
url          = ttyACM0:115200
url_protocol = serial
```

For serial, omit `/dev/`; pArduBridge adds it. Confirm the real device with `ls -l /dev/serial/by-id /dev/ttyACM* /dev/ttyUSB*` and ensure no other process owns it.

## 3. Add the MOOS block

```moos
ServerHost = localhost
ServerPort = 9001
Community  = alpha

LatOrigin  = 42.358456
LongOrigin = -71.087589

ProcessConfig = pArduBridge
{
  AppTick   = 10
  CommsTick = 10

  url          = 0.0.0.0:14550
  url_protocol = udp
  vehicle_type = copter

  vname  = alpha
  vcolor = yellow
  prefix = UAV

  takeoff_altitude = 10
  precision_loiter_enter_loiter = true
  is_sim = true
  logger = false
}
```

For hardware, set the serial endpoint and `is_sim=false`. Configure `vehicle_type=plane` for ArduPlane. Always set `vehicle_type` explicitly.

## 4. Start ArduPilot first

Example SITL commands:

```bash
cd ~/ardupilot
sim_vehicle.py -v ArduCopter --console --out=udp:127.0.0.1:14550
```

or:

```bash
sim_vehicle.py -v ArduPlane --model plane --console --out=udp:127.0.0.1:14550
```

Then launch the MOOS community normally:

```bash
pAntler alpha.moos
```

## 5. Verify telemetry before commands

Use AppCast/uMAC or query the MOOSDB:

```bash
uQueryDB alpha.moos \
  --wait=20 \
  --condition='UAV_HEALTH_AVAILABLE=1' \
  --condition='UAV_GPS_AVAILABLE=1' \
  --condition='UAV_LANDED_STATE_AVAILABLE=1'
```

Inspect the current values:

```bash
uXMS alpha.moos \
  UAV_HEALTH_ALL_OK UAV_IS_ARMABLE UAV_HEALTH_AGE \
  UAV_GPS_FIX_TYPE UAV_GPS_SATELLITES UAV_GPS_HDOP UAV_GPS_AGE \
  UAV_LANDED_STATE UAV_LANDED_STATE_AGE \
  UAV_ARM_POLICY_READY UAV_ARM_POLICY_REASON \
  UAV_COMMAND_RESULT AUTOPILOT_MODE
```

Do not interpret `UAV_HEALTH_GLOBAL_POSITION=1` alone as a GPS lock. Check fix type, satellite count, DOP, and age.

## 6. Safe command examples

MOOS variables are uppercase by convention.

```bash
# Arm or disarm through bridge policy
uPokeDB alpha.moos ARM_UAV=true
uPokeDB alpha.moos ARM_UAV=false

# After ARM is accepted, start configured autonomous takeoff
uPokeDB alpha.moos ARDU_COMMAND=DO_TAKEOFF

# Define and fly a Guided waypoint
uPokeDB alpha.moos \
  NEXT_WAYPOINT='lat=42.35855,lon=-71.08750,x=8,y=10,vname=alpha'
uPokeDB alpha.moos ARDU_COMMAND=FLY_WAYPOINT

# Legacy Guided coordinate hold
uPokeDB alpha.moos ARDU_COMMAND=LOITER

# Native FC Loiter: Copter hold or Plane orbit
uPokeDB alpha.moos ARDU_COMMAND=LOITER_FC

# Native RTL when Helm is inactive; MOOS return routing when active
uPokeDB alpha.moos ARDU_COMMAND=RETURN_TO_LAUNCH

# Copter Precision Loiter
uPokeDB alpha.moos ARDU_COMMAND=PRECISION_LOITER
uPokeDB alpha.moos ARDU_COMMAND=PRECISION_LOITER_OFF

# Copter LAND or Plane AUTOLAND
uPokeDB alpha.moos ARDU_COMMAND=AUTOLAND
```

Watch `UAV_COMMAND_RESULT`. An `ACCEPTED` command is not equivalent to `CONFIRMED`. Copter takeoff adds `CONFIRMED` when activation appears and `COMPLETED` at configured altitude; native RTL and Loiter confirmation likewise arrives later with the same command ID.

## 7. Control-path distinctions

- `ARDU_COMMAND=LOITER` is Guided coordinate hold.
- `ARDU_COMMAND=LOITER_FC` is native ArduPilot Loiter.
- `PRECISION_LOITER` is Copter-only native Loiter plus auxiliary function 39.
- Return with Helm active publishes a MOOS return waypoint; return with Helm inactive requests native FC RTL.
- `AUTOPILOT_MODE` describes bridge/Helm ownership, not the raw ArduPilot mode.

## 8. Before hardware flight

Do not jump from a successful SITL launch directly to powered flight. Complete the [bench and outdoor qualification](HARDWARE_QUALIFICATION.md), first with props removed and propulsion isolated, then outdoors with RC/GPS, and only then conduct a separately authorized flight test.
