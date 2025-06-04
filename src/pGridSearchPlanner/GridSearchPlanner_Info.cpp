/*****************************************************************/
/*    NAME: Steve Nomeny                                         */
/*    ORGN: NTNU, Trondheim                                       */
/*    FILE: GridSearchPlanner_Info.cpp                                */
/*    DATE: Feb 2025                                            */
/*****************************************************************/

#include <cstdlib>
#include <iostream>
#include "GridSearchPlanner_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  The pGridSearchPlanner application  is a module for storing a ");
  blk("  history of vehicle positions in a 2D grid defined over a      ");
  blk("  region of operation and generating a search patter according  ");
  blk("  to the TMSTC* algorithm.                                      ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: pGridSearchPlanner file.moos [OPTIONS]                          ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias", "=<ProcessName>                                      ");
  blk("      Launch pGridSearchPlanner with the given process name rather     ");
  blk("      than pGridSearchPlanner.                                          ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of pGridSearchPlanner.               ");
  blk("                                                                ");
  blk("Note: If argv[2] does not otherwise match a known option,       ");
  blk("      then it will be interpreted as a run alias. This is       ");
  blk("      to support pAntler launching conventions.                 ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showInterfaceAndExit

void showInterfaceAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("pGridSearchPlanner INTERFACE                                           ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:");
  blk("------------------------------------                            ");
  blk("  NODE_REPORT = NAME=alpha,TYPE=UAV,TIME=1678886400.0,X=0,Y=0,  ");
  blk("                SPD=0,HDG=0,LAT=0,LON=0,DEP=0,LENGTH=0,         ");
  blk("                MODE=MODE@ACTIVE:LOITERING                      ");
  blk("    // Vehicle position and state report.                      ");
  blk("                                                                ");
  blk("  NODE_REPORT_LOCAL = (Same format as NODE_REPORT)              ");
  blk("    // Local vehicle position and state report.                ");
  blk("                                                                ");
  blk("  IGNORED_REGION_ALERT = reg:: name=foo,pts={0,0:10,0:10,10:0,10}");
  blk("    // Alert to register a new ignored region.                 ");
  blk("  IGNORED_REGION_ALERT = unreg:: foo                            ");
  blk("    // Alert to unregister an existing ignored region by name. ");
  blk("                                                                ");
  blk("  DO_PLAN_PATHS = true                                          ");
  blk("    // Command to trigger path planning (boolean as string).   ");
  blk("                                                                ");
  blk("  GSP_VISUALIZE_PLANNER_GRIDS = true                            ");
  blk("    // Toggle visualization of TMSTC* internal grids (bool).   ");
  blk("                                                                ");
  blk("  GSP_VISUALIZE_PLANNER_PATHS = true                            ");
  blk("    // Toggle visualization of calculated paths (boolean).     ");
  blk("                                                                ");
  blk("  GSP_MAP_PRINT = 1                                             ");
  blk("    // Set TMSTC* map print version for console (0-3).         ");
  blk("                                                                ");
  blk("  GSP_START_POINT_CLOSEST = true                                ");
  blk("    // Toggle if path should start at waypoint closest to UAV. ");
  blk("                                                                ");
  blk("  XENABLE_MISSION = true                                        ");
  blk("    // Enable/disable mission execution (paths sent to UAVs).  ");
  blk("                                                                ");
  blk("  VIEW_GRID = pts={0,0:...},cell_size=10,cell_vars=x:0,...      ");
  blk("    // Full grid state from pGridSearchViz or similar.         ");
  blk("                                                                ");
  blk("  VIEW_GRID_DELTA = (string)                                    ");
  blk("    // Delta update for the grid from pGridSearchViz.          ");
  blk("                                                                ");
  blk("  CHANGE_PLANNER_MODEX = TMSTC_STAR                             ");
  blk("    // Request to change the active planner mode.              ");
  blk("    // (e.g., \"TMSTC_STAR\", \"VORONOI_SEARCH\")              ");
  blk("                                                                ");
  blk("  XREQUEST_PLANNER_MODE = true                                  ");
  blk("    // Request for the planner to publish its current mode.    ");
  blk("                                                                ");
  blk("PUBLICATIONS:");
  blk("------------------------------------                            ");
  blk("  APPCAST = (AppCastingMOOSApp standard output)                 ");
  blk("    // Standard application casting output.                    ");
  blk("                                                                ");
  blk("  SURVEY_UPDATE_DRONENAME = points=x1,y1:x2,y2:...              ");
  blk("    // Calculated path for a specific drone. DRONENAME is UAV  ");
  blk("    // name, e.g., SURVEY_UPDATE_SKYWALKER. Base name is       ");
  blk("    // configurable via path_publish_variable.                 ");
  blk("                                                                ");
  blk("  DO_SURVEY_DRONENAME = true                                    ");
  blk("    // Command for MOOS PID sim: start survey (if mission_enabled).");
  blk("  DEPLOY_DRONENAME = false                                      ");
  blk("    // Command for MOOS PID sim: ensure not deploying.         ");
  blk("  LOITER_DRONENAME = false                                      ");
  blk("    // Command for MOOS PID sim: ensure not loitering.         ");
  blk("  RETURN_DRONENAME = false                                      ");
  blk("    // Command for MOOS PID sim: ensure not returning.         ");
  blk("  MOOS_MANUAL_OVERRIDE_DRONENAME = false                        ");
  blk("    // Command for MOOS PID sim: ensure not in manual override.");
  blk("                                                                ");
  blk("  HELM_STATUS_DRONENAME = ON                                    ");
  blk("  GCS_COMMAND_DRONENAME = SURVEY                                ");
  blk("    // Command for ArduPilot/GCS: execute survey.              ");
  blk("                                                                ");
  blk("  // Note: The DRONENAME specific vars above are conditional   ");
  blk("  // on is_running_moos_pid and mission_enabled state when     ");
  blk("  // paths are notified.                                       ");
  blk("                                                                ");
  blk("  DO_SURVEY_ALL = false                                         ");
  blk("  DEPLOY_ALL = true                                             ");
  blk("  LOITER_ALL = false                                            ");
  blk("  RETURN_ALL = false                                            ");
  blk("  MOOS_MANUAL_OVERRIDE_ALL = false                              ");
  blk("    // MOOS PID sim: Command all UAVs for Voronoi search.      ");
  blk("                                                                ");
  blk("  HELM_STATUS_ALL = ON                                          ");
  blk("  GCS_COMMAND_ALL = DO_VORONOI                                  ");
  blk("    // ArduPilot/GCS: Command all UAVs for Voronoi search.     ");
  blk("                                                                ");
  blk("  // Note: The _ALL vars above are conditional on              ");
  blk("  // is_running_moos_pid and mission_enabled state when        ");
  blk("  // Voronoi search is notified.                               ");
  blk("                                                                ");
  blk("  VIEW_SEGLIST = label=skywalker_path,edge_color=orange,        ");
  blk("                 points=x1,y1:x2,y2,edge_size=10,active=true  ");
  blk("    // Visual seglist for calculated paths.                    ");
  blk("                                                                ");
  blk("  VIEW_MARKER = type=circle,x=X,y=Y,label=skywalker_start,     ");
  blk("                color=green,edge_color=orange,width=10        ");
  blk("    // Visual marker for path start and end points.            ");
  blk("                                                                ");
  blk("  VIEW_CIRCLE = x=X,y=Y,radius=R,label=Sr_0,edge_color=yellow, ");
  blk("                fill=off,active=true                          ");
  blk("    // Visual circles for TMSTC* grid cells (region/spanning). ");
  blk("                                                                ");
  blk("  XGSP_GRID_EMPTY = true                                        ");
  blk("    // Published if the VIEW_GRID received is considered empty.");
  blk("                                                                ");
  blk("  GSP_CURRENT_PLANNER_MODE = TMSTC_STAR                         ");
  blk("    // Published in response to XREQUEST_PLANNER_MODE or after ");
  blk("    // a CHANGE_PLANNER_MODEX command is processed.            ");
  blk("                                                                ");
  exit(0);
}
//----------------------------------------------------------------
// Procedure: showExampleConfigAndExit

void showExampleConfigAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("pGridSearchPlanner Example MOOS Configuration                          ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = pGridSearchPlanner                                     ");
  blk("{                                                               ");
  blk("  AppTick   = 4          // Interval for Iterate loop (seconds)  ");
  blk("  CommsTick = 4          // Interval for new mail check (seconds)");
  blk("                                                                ");
  blk("  // Core Parameters:                                           ");
  blk("  search_region = pts={-50,-40:-10,0:180,0:180,-150:-50,-150}   ");
  blk("    // Mandatory: The polygonal search region for planning.    ");
  blk("                                                                ");
  blk("  sensor_radius = 10           // Default: 10 (meters)          ");
  blk("    // Sensor detection radius used for coverage calculations. ");
  blk("                                                                ");
  blk("  region_grid_size_ratio = 0.5 // Default: 0.5                  ");
  blk("    // Ratio of sensor_radius to determine TMSTC* grid cell    ");
  blk("    // size (coveragecellradius = sensor_radius * ratio).      ");
  blk("                                                                ");
  blk("  path_publish_variable = SURVEY_UPDATE // Default: SURVEY_UPDATE");
  blk("    // Base MOOS variable name for publishing calculated paths.");
  blk("    // Full variable will be <path_publish_variable>_VNAME.    ");
  blk("                                                                ");
  blk("  planner_mode = VORONOI_SEARCH  // Default: VORONOI_SEARCH     ");
  blk("    // Initial planner mode. Options: TMSTC_STAR, VORONOI_SEARCH.");
  blk("                                                                ");
  blk("  // TMSTC* Specific Parameters:                                ");
  blk("  tmstc_star_config_vmax = 18    // Default: 18 (m/s)           ");
  blk("    // Maximum vehicle velocity for TMSTC* path calculation.   ");
  blk("                                                                ");
  blk("  tmstc_star_config_phi_max_rad = 0.785 // Default: 0.785 (45deg)");
  blk("    // Maximum vehicle bank angle (radians) for TMSTC*.        ");
  blk("                                                                ");
  blk("  tmstc_star_point_filtering = false // Default: false          ");
  blk("    // If true, TMSTC* filters out grid cells already visited  ");
  blk("    // (based on VIEW_GRID).                                   ");
  blk("                                                                ");
  blk("  // Behavior & Visualization:                                  ");
  blk("  start_point_closest = false    // Default: false              ");
  blk("    // If true, TMSTC* paths start from the grid point closest ");
  blk("    // to the vehicle's current position. Otherwise, vehicle   ");
  blk("    // closest to path start point gets the path.              ");
  blk("                                                                ");
  blk("  visualize_planner_grids = false // Default: false             ");
  blk("    // If true, posts TMSTC* internal grids for visualization. ");
  blk("                                                                ");
  blk("  visualize_planner_paths = false // Default: false             ");
  blk("    // If true, posts calculated paths for visualization.      ");
  blk("                                                                ");
  blk("  map_print_version = 0        // Default: 0 (0-3)              ");
  blk("    // TMSTC* map print style to console during planning.      ");
  blk("    // 0:off, 1:init, 2:cover, 3:direction.                    ");
  blk("                                                                ");
  blk("  // Simulation / Control Specific:                            ");
  blk("  is_running_moos_pid = false  // Default: false                ");
  blk("    // Set to true if vehicles are controlled by MOOS PID sim, ");
  blk("    // affects which MOOS variables are used for commands.     ");
  blk("}                                                               ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  // showReleaseInfo("pGridSearchPlanner", "gpl");
  exit(0);
}
