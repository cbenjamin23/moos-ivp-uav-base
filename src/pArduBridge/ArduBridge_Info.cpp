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
  blk("  The pArduBridge application is used for interfacing with              ");
  blk(" Ardupilot. It will send necessary commands from the MOOS community                                           ");
  blk(" needed to controll the vehicle autonomously.                                                                ");
  blk("                                                                ");
  blk("                                                                ");
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
  blk("  AppTick   = 4                                                 ");
  blk("  CommsTick = 4                                                 ");
  blk("                                                                ");
  blk("  ArduPilotURL/ ardupiloturl | url   = 0.0.0.0:1455                   ");    
  blk("  (or ArduPilotURL   = ttySAC0:115200 )                             ");    
  blk("  url_protocol = serial, tcp, udp                                ");    
  blk("  vehicle_type = plane  // plane or copter                       ");
  blk("  takeoff_altitude = 10  // Copter default if not configured     ");
  blk("  precision_loiter_enter_loiter = true                           ");
  blk("  prefix   = NAV                                               ");    
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
  blk("  NODE_MESSAGE = src_node=alpha,dest_node=bravo,var_name=FOO,   ");
  blk("                 string_val=BAR                                 ");
  blk("                                                                ");
  blk("  FLY_WAYPOINT                                                              ");
  blk("  DO_TAKEOFF                                                              ");
  blk("  CHANGE_SPEED                                                              ");
  blk("  ARM_UAV = true  // Arm; false disarms                                ");
  blk("  RESET_SPEED_MIN                                                              ");
  blk("  RETURN_TO_LAUNCH = true  // Request MOOS return routing or ArduPilot RTL      ");
  blk("  AUTOLAND = true  // Request LAND through the conservative mode policy  ");
  blk("  LOITER                                                              ");
  blk("  ARDU_COMMAND=PRECISION_LOITER // Copter only: enable aux function 39          ");
  blk("  ARDU_COMMAND=PRECISION_LOITER_OFF // Disable Precision Loiter                 ");
  blk("  ARDU_COMMAND=LOITER_FC // Native Copter hold or Plane orbit at current point  ");
  blk("  NEXT_WAYPOINT                                                              ");
  blk("                                                                ");
  blk("  DESIRED_HEADING                                                              ");
  blk("    Plane: desired course over ground. Copter: absolute yaw heading.            ");
  blk("  DESIRED_SPEED                                                              ");
  blk("    Plane: airspeed by default. Copter: ground speed.                          ");
  blk("  DESIRED_ALTITUDE                                                           ");
  blk("                                                                ");
  blk("  MOOS_MANUAL_OVERIDE                                                              ");
  blk("                                                                ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  Publications are determined by the node message content.      ");
  blk("                                                                ");
  blk("  UAV_LAT                                                              ");
  blk("  UAV_LONG                                                              ");
  blk("  UAV_X                                                              ");
  blk("  UAV_Y                                                              ");
  blk("  UAV_HEADING                                                              ");
  blk("  UAV_SPEED_OVER_GROUND                                                              ");
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
  blk("  UAV_GPS_FIX_TYPE           // MAVSDK fix type enum: 0=None through 6=RTK fixed      ");
  blk("  UAV_GPS_SATELLITES         // Satellites used/visible                               ");
  blk("  UAV_GPS_HDOP               // Horizontal dilution of precision                     ");
  blk("  UAV_GPS_VDOP               // Vertical dilution of precision                       ");
  blk("  UAV_GPS_AGE                // Seconds since the last GPS sample                     ");
  blk("  UAV_LANDED_STATE_AVAILABLE // 1 after flight-controller landed state is received    ");
  blk("  UAV_LANDED_STATE           // ON_GROUND, IN_AIR, TAKING_OFF, LANDING, or UNKNOWN    ");
  blk("  UAV_LANDED_STATE_AGE       // Seconds since the last landed-state sample             ");
  blk("  DEPLOY                                                              ");
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
