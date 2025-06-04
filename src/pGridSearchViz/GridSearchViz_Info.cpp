/*****************************************************************/
/*    NAME: Steve Nomeny                                         */
/*    ORGN: NTNU, Trondheim                                       */
/*    FILE: GridSearchViz_Info.cpp                                */
/*    DATE: Feb 2025                                            */
/*****************************************************************/

#include <cstdlib>
#include <iostream>
#include "GridSearchViz_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  The pGridSearchViz application visualizes drone/vehicle       ");
  blk("  search coverage in a 2D grid. It tracks covered areas as      ");
  blk("  vehicles move, calculates coverage statistics, and supports   ");
  blk("  visualization of sensor areas. The application can manage     ");
  blk("  ignored regions and track mission coverage metrics over time. ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: pGridSearchViz file.moos [OPTIONS]                          ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias", "=<ProcessName>                                      ");
  blk("      Launch pGridSearchViz with the given process name rather     ");
  blk("      than pGridSearchViz.                                          ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of pGridSearchViz.               ");
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
  blu("pGridSearchViz Example MOOS Configuration                          ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = pGridSearchViz                                     ");
  blk("{                                                               ");
  blk("  AppTick   = 4                                                 ");
  blk("  CommsTick = 4                                                 ");
  blk("                                                                ");
  blk("  // Basic App Behavior                                         ");
  blk("  report_deltas = true         // default: true. Enables delta  ");
  blk("                               // updates for grid visualization.");
  blk("                               // Post full grid if false.      ");
  blk("  grid_var_name = VIEW_GRID    // default: \"VIEW_GRID\". MOOS   ");
  blk("                               // variable for publishing the   ");
  blk("                               // grid data.                    ");
  blk("  grid_label    = gsv          // default: \"gsv\". Label used   ");
  blk("                               // in grid data strings (e.g.,  ");
  blk("                               // for VIEW_GRID_DELTA).         ");
  blk("                                                                ");
  blk("  // Vehicle Tracking Filters (applied to NODE_REPORT and       ");
  blk("  // NODE_REPORT_LOCAL)                                         ");
  blk("  // These are examples;                                        ");
  blk("  match_name    = abe          // Example: If set, only         ");
  blk("                               // NODE_REPORT from 'abe' is     ");
  blk("                               // processed.                    ");
  blk("  ignore_name   = ben          // Example: If set, NODE_REPORT  ");
  blk("                               // from 'ben' is ignored.        ");
  blk("                                                                ");
  blk("  // Sensor Visualization Configuration                         ");
  blk("  sensor_radius = 10           // default: 10. Max sensor       ");
  blk("                               // coverage radius in meters.    ");
  blk("  sensor_altitude_max = 25     // default: 25. Altitude (m) at  ");
  blk("                               // which sensor_radius is        ");
  blk("                               // achieved if not fixed. If     ");
  blk("                               // drone is higher,              ");
  blk("                               // sensor_radius_max is used.    ");
  blk("  sensor_radius_fixed = true   // default: true. If true,       ");
  blk("                               // sensor_radius is always used. ");
  blk("                               // If false, radius scales with  ");
  blk("                               // altitude up to                ");
  blk("                               // sensor_altitude_max.          ");
  blk("  sensor_color = black         // default: \"black\". Color for  ");
  blk("                               // sensor area visualization     ");
  blk("                               // (e.g., \"red\", \"blue\").     ");
  blk("  sensor_transparency = 0.2    // default: 0.2. Transparency of ");
  blk("                               // sensor area (0.0=invisible,   ");
  blk("                               // 1.0=opaque).                  ");
  blk("  visualize_sensor_area = true // default: true. Set to false   ");
  blk("                               // to disable publishing         ");
  blk("                               // VIEW_CIRCLE.                  ");
  blk("                                                                ");
  blk("  // Grid Cell Decay Configuration                              ");
  blk("  grid_cell_decay_time = 0     // default: 0. Time (seconds) for");
  blk("                               // covered cell values to decay. ");
  blk("                               // 0 means no decay.             ");
  blk("                                                                ");
  blk("  // Simulation Environment                                     ");
  blk("  is_running_moos_pid = false  // default: false. Set to true if");
  blk("                               // running MOOS PID simulation.  ");
  blk("                                                                ");
  blk("  // Grid Definition (example of a convex grid)                 ");
  blk("  grid_config = pts={-50,-40: -10,0: 180,0: 180,-150: -50,-150} ");
  blk("                // Polygon defining grid boundaries.            ");
  blk("  grid_config = cell_size=5                                     ");
  blk("                // Size of each grid cell in meters.            ");
  blk("  grid_config = cell_vars=x:0:y:0:z:0                           ");
  blk("                // Variables stored per cell, with initial      ");
  blk("                // values. Only first var used for coverage.    ");
  blk("  grid_config = cell_min=x:0                                    ");
  blk("                // Minimum value for cell variable 'x'.         ");
  blk("  grid_config = cell_max=x:10                                   ");
  blk("                // Maximum value for cell variable 'x'.         ");
  blk("}                                                               ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showInterfaceAndExit

void showInterfaceAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("pGridSearchViz MOOS Interface                                    ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  NODE_REPORT = (string) // Vehicle position reports.           ");
  blk("                      // e.g., NAME=alpha,X=10,Y=20,ALT=30,     ");
  blk("                      //       TIME=123.45                      ");
  blk("  NODE_REPORT_LOCAL = (string) // Local vehicle position reports.");
  blk("                      // e.g., NAME=bravo,X=15,Y=25,ALT=35,    ");
  blk("                      //       TIME=123.50                      ");
  blk("  GSV_RESET_GRID = (trigger) // Any message on this var resets  ");
  blk("                      // grid coverage data.                    ");
  blk("  GSV_VISUALIZE_SENSOR_AREA = (string) // Expects \"true\" or     ");
  blk("                      // \"false\" (case-insensitive) to      ");
  blk("                      // toggle sensor viz.                     ");
  blk("  IGNORED_REGION_ALERT = (string) // Register:                  ");
  blk("                      // \"label=r1,type=poly,                 ");
  blk("                      //  pts={0,0:10,0:10,10:0,10}\"          ");
  blk("                      // Unregister: \"unreg::r1\"            ");
  blk("  XENABLE_MISSION = (string) // Expects \"true\" (case-         ");
  blk("                      // insensitive) to enable mission and     ");
  blk("                      // start coverage tracking.               ");
  blk("  XDISABLE_RESET_MISSION = (trigger) // Any message on this var ");
  blk("                      // disables mission, resets start time    ");
  blk("                      // and grid.                              ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  VIEW_GRID = (string) // Full grid specification. e.g.,        ");
  blk("                      // \"label=gsv,pts={0,0:...},            ");
  blk("                      //  cell_size=5,...\"                   ");
  blk("  VIEW_GRID_DELTA = (string) // Grid update string for changed  ");
  blk("                      // cells. e.g., \"label=gsv,             ");
  blk("                      //  cellreplace=0:1,10:1,...\"          ");
  blk("  VIEW_CIRCLE = (string) // Sensor area visualization. e.g.,    ");
  blk("                      // \"x=10,y=20,radius=5,                 ");
  blk("                      //  label=v1_sensor,...\"               ");
  blk("  GSV_COVERAGE_PERCENTAGE = 0.0 // Current grid coverage       ");
  blk("                      // percentage (0.0-100.0). e.g., 75.2   ");
  blk("  MISSION_START_TIME = 0.0 // MOOSTime when mission began.      ");
  blk("                      // 0 if not started/reset. e.g.,        ");
  blk("                      // 1678886400.5                         ");
  blk("  XENABLE_MISSION = (string) // Published as \"false\" when     ");
  blk("                      // mission is programmatically disabled.  ");
  blk("                      // e.g., \"false\"                      ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("pGridSearchViz", "gpl");
  exit(0);
}
