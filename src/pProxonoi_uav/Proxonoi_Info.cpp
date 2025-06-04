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
  blk("constitute a voronoi decomposition of the opregion.             ");
  blk("                                                                ");
  blk("  It can operate in multiple planning modes, including          ");
  blk("VORONOI_SEARCH and GRID_SEARCH, calculating optimal setpoints   ");
  blk("for UAV navigation through undiscovered areas. The application  ");
  blk("supports various setpoint methods (gridsearch/centroid/center)  ");
  blk("and integrates with grid-based search visualization.            ");
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
  mag("  --alias", "=<ProcessName>                                      ");
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
  blk("  AppTick   = 4               // Frequency of main app loop     ");
  blk("  CommsTick = 4               // Frequency of MOOS comms        ");
  blk("                                                                ");
  blk("  // Ownship vehicle name.                                      ");
  blk("  // Default: Automatically set from MOOS community (e.g. gilda)");
  blk("  ownship = gilda                                               ");
  blk("                                                                ");
  blk("  // Operational area polygon definition.                       ");
  blk("  region = pts={-20,0 : -20,-190 : 170,-190 : 190,-90 : 170,0}  ");
  blk("                                                                ");
  blk("  // Contacts outside this range are initially ignored.         ");
  blk("  reject_range = 10000        // Default: 10000 m               ");
  blk("                                                                ");
  blk("  // How to interpret contact X,Y vs Lat/Lon coordinates.       ");
  blk("  // Options: \"verbatim\", \"lazy_lat_lon\".                     ");
  blk("  contact_local_coords = verbatim // Default: verbatim           ");
  blk("                                                                ");
  blk("  // If true, use MOOSGeodesy for Lat/Lon to local X/Y.         ");
  blk("  use_geodesy = false         // Default: false                 ");
  blk("                                                                ");
  blk("  // If true, publish Voronoi region via 'region_update_var'.   ");
  blk("  post_region = false         // Default: false                 ");
  blk("                                                                ");
  blk("  // If true, publish Voronoi region as VIEW_POLYGON.           ");
  blk("  post_poly = false           // Default: false                 ");
  blk("                                                                ");
  blk("  // MOOS var for publishing region if post_region=true.        ");
  blk("  region_update_var = PROX_UP_REGION // Default: PROX_UP_REGION ");
  blk("                                                                ");
  blk("  // MOOS var for subscribing to dynamic ignore list.           ");
  blk("  ignore_list_update_var = PROX_SET_IGNORE_LIST // Default: PROX_SET_IGNORE_LIST");
  blk("                                                                ");
  blk("  // Color for VIEW_POLYGON visualization if post_poly=true.    ");
  blk("  vcolor = white              // Default: white                 ");
  blk("                                                                ");
  blk("  // Method for navigation setpoint calculation.                ");
  blk("  // Options: \"center\", \"centroid\", \"gridsearch\".           ");
  blk("  setpt_method = center         // Default: center              ");
  blk("                                                                ");
  blk("  // Time in seconds after which a contact is stale.            ");
  blk("  node_record_stale_treshold = 10 // Default: 10 s              ");
  blk("                                                                ");
  blk("  // Primary planning mode for behavior.                        ");
  blk("  // Options: \"VORONOI_SEARCH\", \"GRID_SEARCH\".                ");
  blk("  planner_mode = VORONOI_SEARCH // Default: VORONOI_SEARCH     ");
  blk("                                                                ");
  blk("  // Comma-separated list of vehicle names to always ignore.    ");
  blk("  always_reject = vehicle_x,vehicle_y // Default: (empty)      ");
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
  blu("pProxonoi MOOS Interface                                        ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  NODE_REPORT = NAME=alpha,TYPE=UUV,TIME=1252348077.5,         ");
  blk("                X=51.71,Y=-35.50,LAT=43.824981,LON=-70.329755, ");
  blk("                SPD=2.0,HDG=118.8,YAW=118.8,DEPTH=4.6,LENGTH=3.8");
  blk("    // Reports from other vehicles or contacts.                 ");
  blk("                                                                ");
  blk("  NAV_X = 123.45                                                ");
  blk("    // Ownship X position in local grid.                        ");
  blk("                                                                ");
  blk("  NAV_Y = -67.89                                                ");
  blk("    // Ownship Y position in local grid.                        ");
  blk("                                                                ");
  blk("  NAV_HEADING = 180.0                                           ");
  blk("    // Ownship heading in degrees.                              ");
  blk("                                                                ");
  blk("  PROX_POLY_VIEW = true                                         ");
  blk("    // Toggle display of proximity polygon (true/false/toggle). ");
  blk("                                                                ");
  blk("  PROX_CLEAR = true                                             ");
  blk("    // Clears internal state, contacts (value is irrelevant).   ");
  blk("                                                                ");
  blk("  PROX_SET_IGNORE_LIST = charlie,delta                          ");
  blk("    // Comma-separated list of contact names to ignore.         ");
  blk("    // Variable name configured by 'ignore_list_update_var'.    ");
  blk("                                                                ");
  blk("  PROX_UP_REGION = pts={0,0:10,0:10,10:0,10},label=my_region    ");
  blk("    // Optionally subscribe to an external operational region.  ");
  blk("    // Variable name configured by 'region_update_var'.         ");
  blk("                                                                ");
  blk("  PROX_SETPT_METHOD = gridsearch                                ");
  blk("    // Setpoint calculation method (gridsearch/centroid/center).");
  blk("                                                                ");
  blk("  VIEW_GRID = format=dense,label=sgrid,cell_size=10,            ");
  blk("              pts={0,0:0,100:100,100:100,0}                     ");
  blk("    // Defines a convex grid for planner_mode=GRID_SEARCH.      ");
  blk("                                                                ");
  blk("  VIEW_GRID_DELTA = cell=0,1,discovered;cell=2,3,explored       ");
  blk("    // Updates states of individual cells in the VIEW_GRID.     ");
  blk("                                                                ");
  blk("  CHANGE_PLANNER_MODE = GRID_SEARCH                             ");
  blk("    // Dynamically changes planner mode (VORONOI_SEARCH/etc.).  ");
  blk("                                                                ");
  blk("  PROX_SET_VISUALIZATION = true                                 ");
  blk("    // Toggles detailed visualization artifacts (true/false).   ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  PROXONOI_PID = 12345                                          ");
  blk("    // Process ID of the pProxonoi application.                 ");
  blk("                                                                ");
  blk("  PROXONOI_REGION = pts={0,0:10,0:10,10:0,10},label=vpoly_alpha ");
  blk("    // Publishes the current operational region.                ");
  blk("                                                                ");
  blk("  PROXONOI_POLY = pts={0,0:10,0:10,10:0,10},label=vpoly_alpha   ");
  blk("    // Publishes the current Voronoi polygon.                   ");
  blk("                                                                ");
  blk("  PROX_UP_REGION = pts={0,0:10,0:10,10:0,10},label=vpoly_alpha  ");
  blk("    // Published Voronoi polygon if 'post_region' is true.      ");
  blk("    // Variable name configured by 'region_update_var'.         ");
  blk("                                                                ");
  blk("  VIEW_POLYGON = pts={0,0:10,0:10,10:0,10},label=vpoly_alpha,   ");
  blk("                 edge_color=white,vertex_size=0,edge_size=1     ");
  blk("    // Visualization polygon if 'post_poly' is true.            ");
  blk("                                                                ");
  blk("  NODE_MESSAGE_LOCAL = src=skywalker,dest=all,var=PROX_POLY_AREA,");
  blk("                       val=1234.56,time=1599670562.4,color=off  ");
  blk("    // Area of the calculated Voronoi polygon.                  ");
  blk("                                                                ");
  blk("  VIEW_POINT = x=5.0,y=5.0,active=true,label=g_ownship,         ");
  blk("              vertex_color=white,vertex_size=10                 ");
  blk("    // Visualization for calculated setpoints and search points.");
  blk("                                                                ");
  blk("  PROX_SEARCHCENTER = x=5.123,y=6.789,label=searchCenter        ");
  blk("    // Calculated search center point in grid search mode.      ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  // showReleaseInfo("pProxonoi", "gpl");
  exit(0);
}
