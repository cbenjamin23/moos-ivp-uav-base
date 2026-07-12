/************************************************************/
/*    NAME: Steve Carter Feujo Nomeny                       */
/*    ORGN: NTNU, MIT                                       */
/*    FILE: ArduBridge_Info.cpp                             */
/*    DATE: September 9th, 2024                             */
/************************************************************/

#include <cstdlib>
#include <iostream>
#include "ArduBridge_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  pArduBridge connects a MOOS community to an ArduPilot vehicle ");
  blk("  through MAVSDK/MAVLink. It publishes navigation, health, GPS, ");
  blk("  landed-state, policy, and command-result data, and translates ");
  blk("  MOOS helm setpoints and explicit commands for Plane or Copter.");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: pArduBridge file.moos [OPTIONS]                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch pArduBridge with the given process name         ");
  blk("      rather than pArduBridge.                           ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of pArduBridge.        ");
  blk("                                                                ");
  blk("Note: If argv[2] does not otherwise match a known option,       ");
  blk("      then it will be interpreted as a run alias. This is       ");
  blk("      to support pAntler launching conventions.                 ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showExampleConfigAndExit

void showExampleConfigAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("pArduBridge Example MOOS Configuration                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = pArduBridge                              ");
  blk("{                                                               ");
  blk("  AppTick   = 10                                                ");
  blk("  CommsTick = 10                                                ");
  blk("                                                                ");
  blk("  url          = 0.0.0.0:14550 // or ttyACM0:115200 for serial ");
  blk("  url_protocol = udp             // udp, tcp, or serial          ");
  blk("  vehicle_type = copter          // copter or plane              ");
  blk("  takeoff_altitude = 10          // meters AGL; Copter default   ");
  blk("  precision_loiter_enter_loiter = true // false requires FC Loiter already ");
  blk("  command_groundspeed = true     // forced true for Copter       ");
  blk("  is_sim = false                 // Plane SITL mission setup only");
  blk("  logger = false                                               ");
  blk("  prefix = UAV                                                 ");
  blk("  vname  = alpha                                               ");
  blk("  vcolor = yellow                                              ");
  blk("                                                                ");
  blk("}                                                               ");
  blk("                                                                ");
  exit(0);
}


//----------------------------------------------------------------
// Procedure: showInterfaceAndExit

void showInterfaceAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("pArduBridge INTERFACE                                    ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  ARDU_COMMAND = FLY_WAYPOINT | DO_TAKEOFF | RESET_SPEED_MIN   ");
  blk("                 RETURN_TO_LAUNCH | LOITER | LOITER_FC         ");
  blk("                 PRECISION_LOITER | PRECISION_LOITER_OFF        ");
  blk("                 AUTOLAND | SURVEY | DO_VORONOI | VIZ_HOME      ");
  blk("  ARM_UAV = true|false                                         ");
  blk("  RETURN_TO_LAUNCH = true|false                                ");
  blk("  AUTOLAND = true|false                                        ");
  blk("  NEXT_WAYPOINT = lat=...,lon=...,x=...,y=...,vname=...        ");
  blk("  HELM_STATUS, AUTOPILOT_MODE, MOOS_MANUAL_OVERRIDE            ");
  blk("  CHANGE_SPEED, CHANGE_COURSE, CHANGE_ALTITUDE, DEAD_MAN_POST_INTERRUPT ");
  blk("  Compatibility registrations: FLY_WAYPOINT, DO_TAKEOFF, LOITER, ");
  blk("    SURVEY, RESET_SPEED_MIN, VIZ_HOME (use ARDU_COMMAND path)   ");
  blk("                                                                ");
  blk("  DESIRED_HEADING                                                              ");
  blk("    Plane: desired course over ground. Copter: absolute yaw heading.            ");
  blk("  DESIRED_SPEED                                                              ");
  blk("    Plane: airspeed by default. Copter: ground speed.                          ");
  blk("  DESIRED_ALTITUDE                                                           ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  <PREFIX>_LAT, <PREFIX>_LON, <PREFIX>_X, <PREFIX>_Y           ");
  blk("  <PREFIX>_HEADING, <PREFIX>_SPEED                             ");
  blk("  <PREFIX>_ALTITUDE, <PREFIX>_DEPTH                            ");
  blk("  UAV_HEALTH_AVAILABLE       // 1 after MAVSDK health telemetry is received          ");
  blk("  UAV_IS_ARMED               // Current armed state reported by the flight controller ");
  blk("  UAV_HEALTH_GYRO            // 1 when healthy                                        ");
  blk("  UAV_HEALTH_ACCEL           // 1 when healthy                                        ");
  blk("  UAV_HEALTH_MAG             // 1 when healthy                                        ");
  blk("  UAV_HEALTH_LOCAL_POSITION  // MAVSDK health flag; not proof of GPS fix              ");
  blk("  UAV_HEALTH_GLOBAL_POSITION // MAVSDK health flag; check UAV_GPS_* too                ");
  blk("  UAV_HEALTH_HOME_POSITION   // 1 when healthy                                        ");
  blk("  UAV_IS_ARMABLE             // pre-arm status reported by MAVSDK                    ");
  blk("  UAV_HEALTH_ALL_OK          // MAVSDK aggregate health                               ");
  blk("  UAV_HEALTH_AGE             // Seconds since the last detailed health sample         ");
  blk("  UAV_ARM_POLICY_READY       // 1 when bridge policy permits a new ARM submission      ");
  blk("  UAV_ARM_POLICY_REASON      // Stable reason token, e.g. READY or HEALTH_STALE        ");
  blk("  UAV_DISARM_POLICY_READY    // 1 when bridge policy permits a new DISARM submission   ");
  blk("  UAV_DISARM_POLICY_REASON   // Stable reason token, e.g. READY or NOT_ON_GROUND       ");
  blk("  UAV_LAND_POLICY_READY      // 1 when mode policy permits a new LAND submission       ");
  blk("  UAV_LAND_POLICY_REASON     // Stable reason token; all unlisted modes are denied      ");
  blk("  UAV_COMMAND_RESULT         // Lifecycle; confirmed mode changes add CONFIRMED/TIMED_OUT ");
  blk("  UAV_GPS_AVAILABLE          // 1 after GPS telemetry is received; fix may be invalid ");
  blk("  UAV_GPS_FIX_TYPE           // 0=No GPS,1=No Fix,2=2D,3=3D,...6=RTK fixed          ");
  blk("  UAV_GPS_SATELLITES         // Satellites used/visible                               ");
  blk("  UAV_GPS_HDOP               // Horizontal dilution of precision                     ");
  blk("  UAV_GPS_VDOP               // Vertical dilution of precision                       ");
  blk("  UAV_GPS_AGE                // Seconds since the last GPS sample                     ");
  blk("  UAV_LANDED_STATE_AVAILABLE // 1 after flight-controller landed state is received    ");
  blk("  UAV_LANDED_STATE           // ON_GROUND, IN_AIR, TAKING_OFF, LANDING, or UNKNOWN    ");
  blk("  UAV_LANDED_STATE_AGE       // Seconds since the last landed-state sample             ");
  blk("  AUTOPILOT_MODE             // pArduBridge helm state, not raw FC mode ");
  blk("  MOOS_MANUAL_OVERRIDE, RETURN_UPDATE, TOWAYPT_UPDATE          ");
  blk("  SURVEY_UPDATE, CONST_ALTITUDE_UPDATE, VIEW_*                  ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  //showReleaseInfo("pArduBridge", "gpl");
  exit(0);
}
