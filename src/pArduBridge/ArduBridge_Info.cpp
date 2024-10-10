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
  blk("  ARM_UAV                                                              ");
  blk("  RESET_SPEED_MIN                                                              ");
  blk("  RETURN_TO_LAUNCH                                                              ");
  blk("  LOITER                                                              ");
  blk("  NEXT_WAYPOINT                                                              ");
  blk("                                                                ");
  blk("  DESIRED_HEADING                                                              ");
  blk("  DESIRED_SPEED                                                              ");
  blk("  DESIRED_DEPTH                                                              ");
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
  blk("  DEPLOY                                                              ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("pArduBridge", "gpl");
  exit(0);
}

