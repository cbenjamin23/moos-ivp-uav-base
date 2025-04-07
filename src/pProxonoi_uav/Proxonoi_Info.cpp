/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: Proxonoi_Info.cpp                                    */
/*    DATE: December 25th 2019                                   */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#include <cstdlib>
#include <iostream>
#include "Proxonoi_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  The pProxonoi app will calculate and publish a proximity      ");
  blk("Voronoi polygon around ownship, given (a) an operation region,  ");
  blk("and (b) the location of other known contacts in the opregion.   ");
  blk("The polygons of all vehicles, if similarly calculated, would    ");
  blk("constitute a voronoi decomposition of the opregion. Original    ");
  blk("motivation for this app is to support a Voronoi helm behavior   ");
  blk("to maneuver in a way to balance a local Voronoi distribution    ");
  blk("between nearby vehicles.                                        ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: pProxonoi file.moos [OPTIONS]                            ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch pProxonoi with the given process name              ");
  blk("      rather than pProxonoi.                                    ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of pProxonoi.                 ");
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
  blu("pProxonoi Example MOOS Configuration                            ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = pProxonoi                                       ");
  blk("{                                                               ");
  blk("  AppTick   = 4                                                 ");
  blk("  CommsTick = 4                                                 ");
  blk("                                                                ");
  blk("  region = -20,0 : -20,-190 : 170,-190 : 190,-90 : 170,0        ");
  blk("                                                                ");
  blk("  post_region  = false  // Default is false                     ");
  blk("  post_poly    = false  // Default is false                     ");
  blk("  reject_range = 100    // Default is 10000                     ");
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
  blu("pProxonoi INTERFACE                                    ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("     NODE_REPORT = NAME=alpha,TYPE=UUV,TIME=1252348077.5,       ");
  blk("                   X=51.71,Y=-35.50, LAT=43.824981,             ");
  blk("                   LON=-70.329755,SPD=2.0,HDG=118.8,            ");
  blk("                   YAW=118.8,DEPTH=4.6,LENGTH=3.8               ");
  blk("                                                                ");
  blk("  PROX_POLY_VIEW = true/false/toggle                            ");
  blk("  PROX_CLEAR     = true  (value is irrelevant)                  ");
  blk("  NAX_X          = 10                                           ");
  blk("  NAX_Y          = 5                                            ");
  blk("                                                                ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  Publications are determined by the node message content.      ");
  blk("                                                                ");
  blk("  VORONOI_POLY = pts={-20,0:-20,-48:73,-94:75,-94:101,0},       ");
  blk("                 label=vpoly_abe                                ");
  blk("  VIEW_POLYGON = pts={-20,0:-20,-48:73,-94:75,-94:101,0},       ");
  blk("                 label=vpoly_abe                                ");
  blk("  PROXONOI_PID = 718                                            ");

  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("pProxonoi", "gpl");
  exit(0);
}






