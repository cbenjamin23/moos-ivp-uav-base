/*****************************************************************/
/*    NAME: Steve Nomeny                                         */
/*    ORGN: NTNU, Trondheim                                       */
/*    FILE: MissionOperator_Info.cpp                                */
/*    DATE: May 2025                                            */
/*****************************************************************/

#include <cstdlib>
#include <iostream>
#include "MissionOperator_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  The pMissionOperator application is a module for automatically");
  blk("  running sequences of missions with different planning         ");
  blk("  algorithms. It monitors mission status and handles mission    ");
  blk("  completion or timeouts, automatically resetting and starting  ");
  blk("  new missions according to the configuration.                  ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: pMissionOperator file.moos [OPTIONS]                     ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias", "=<ProcessName>                                      ");
  blk("      Launch pMissionOperator with the given process name rather");
  blk("      than pMissionOperator.                                    ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of pMissionOperator.          ");
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
  blu("pMissionOperator Example MOOS Configuration                     ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = pMissionOperator                                ");
  blk("{                                                               ");
  blk("  AppTick   = 4                                                 ");
  blk("  CommsTick = 4                                                 ");
  blk("                                                                ");
  blk("  op_region = -1008.5,-586.1:247.2,451.5 // Operating region  ");
  blk("  mission_operator_enable = false // Enable mission operator     "); 
  blk("  // required                                                   ");
  blk("  is_running_moos_pid = true                                    ");
  blk("                                                                ");
  blk("  // Mission operation parameters                               ");
  blk("  mission_duration = 600     // Maximum mission time in seconds ");
  blk("  reset_delay = 2            // Delay between mission completion and reset ");
  blk("                                                                ");
  blk("  // Algorithm missions                                         ");
  blk("  voronoi_missions = 10      // Run 10 missions with VORONOI_SEARCH ");
  blk("  tmstc_missions = 5         // Run 5 missions with TMSTC_STAR  ");
  blk("                                                                ");
  blk("  // Initial planner mode (optional, will use first algorithm with missions if not specified) ");
  blk("  planner_mode = VORONOI_SEARCH                                 ");
  blk("}                                                               ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showInterfaceAndExit

void showInterfaceAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("pMissionOperator INTERFACE                                      ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  MISSION_COMPLETE    = true                                    ");
  blk("  CHANGE_PLANNER_MODEX                                          ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  CHANGE_PLANNER_MODEX = VORONOI_SEARCH                         ");
  blk("  CHANGE_PLANNER_MODE_ALL = VORONOI_SEARCH                      ");
  blk("  XENABLE_MISSION = true                                        ");
  blk("  XDISABLE_RESET_MISSION = true                                 ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("pMissionOperator", "gpl");
  exit(0);
}
