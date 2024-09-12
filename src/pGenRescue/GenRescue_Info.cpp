/****************************************************************/
/*   NAME: Steve Nomeny and Filip Stromstad                                             */
/*   ORGN: MIT, Cambridge MA                                    */
/*   FILE: GenRescue_Info.cpp                               */
/*   DATE: December 29th, 1963                                  */
/****************************************************************/

#include <cstdlib>
#include <iostream>
#include "GenRescue_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  The pGenRescue application is used for generating a path based   ");
  blk("  on a given input file. It is part of the MOOS-IvP              ");
  blk("  autonomous vehicle software suite developed at MIT.           ");
  blk("                                                                ");
  blk("  The application takes a file.moos as input and provides        ");
  blk("  options for customizing the path generation process.           ");
  blk("                                                                ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: pGenRescue file.moos [OPTIONS]                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch pGenRescue with the given process name         ");
  blk("      rather than pGenRescue.                           ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of pGenRescue.        ");
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
  blu("pGenRescue Example MOOS Configuration                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = pGenRescue                              ");
  blk("{                                                               ");
  blk("  AppTick   = 4                                                 ");
  blk("  CommsTick = 4                                                 ");
  blk("                                                                ");
  blk("  visit_radius = 3                                              ");
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
  blu("pGenRescue INTERFACE                                    ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  - NAV_X: 0                                                     ");
  blk("  - NAV_Y: 0                                                     ");
  blk("  - VISIT_POINT: 0                                                ");
  blk("  - MODE: 0                                                       ");
  blk("  - GENPATH_REGENERATE: 0                                         ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  - TRANSIT_UPDATES: update_str                                 ");
  blk("  - TRANSIT: true                                              ");
  blk("  - DEPLOY: true                                               ");
  blk("  - MOOS_MANUAL_OVERRIDE: false                                ");
  blk("  - VEHICLE_READY: true                                        ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("pGenRescue", "gpl");
  exit(0);
}

