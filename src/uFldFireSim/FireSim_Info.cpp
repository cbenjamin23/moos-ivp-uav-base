// filepath: /home/steve/moos-ivp-uav/src/uFldFireSim/FireSim_Info.cpp
/*****************************************************************/
/*    NAME: Michael Benjamin, modified by Steve Nomeny           */
/*    FILE: FireSim_Info.cpp                                     */
/*    DATE: Feb 2025 (Update with current month/year)            */
/*                                                               */
/*****************************************************************/

#include <cstdlib>
#include <iostream>
#include "FireSim_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  The uFldFireSim application serves as the central simulator   ");
  blk("  and arbiter for autonomous fire detection and response        ");
  blk("  missions. It maintains ground truth for fire locations, their ");
  blk("  status (e.g., undiscovered, detected), and the       ");
  blk("  status of designated 'ignored regions'.                       ");
  blk("                                                                ");
  blk("  Key functionalities include:                                  ");
  blk("  - Reading initial fire/ignored region configs from files.     ");
  blk("  - Dynamic modification of fires/regions via MOOS messages.    ");
  blk("  - Simulating discovery by UAVs based on proximity, sensor     ");
  blk("    characteristics (range, P_detect, altitude), and chance.    ");
  blk("  - Managing mission state: start time, duration, end criteria. ");
  blk("  - Tracking vehicle performance, determining leader.           ");
  blk("  - Calculating and reporting a mission score.                  ");
  blk("  - Dynamically spawning fires/regions based on mission time.   ");
  blk("  - Providing rich visualization via MOOS VIEW_* messages.      ");
  blk("  - Interfacing with a planner mode for scoring purposes.       ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                          ");
  blu("==========================================================");
  blu("Usage: uFldFireSim file.moos [OPTIONS]                    ");
  blu("==========================================================");
  blk("                                                          ");
  showSynopsis();
  blk("                                                          ");
  blk("Options:                                                  ");
  mag("  --alias", "=<ProcessName>                                ");
  blk("      Launch uFldFireSim with the given process name      ");
  blk("      rather than uFldFireSim.                            ");
  mag("  --example, -e                                           ");
  blk("      Display example MOOS configuration block.           ");
  mag("  --help, -h                                              ");
  blk("      Display this help message.                          ");
  mag("  --interface, -i                                         ");
  blk("      Display MOOS publications and subscriptions.        ");
  mag("  --version,-v                                            ");
  blk("      Display release version of uFldFireSim.             ");
  blk("                                                          ");
  blk("Note: If argv[2] does not otherwise match a known option, ");
  blk("      then it will be interpreted as a run alias. This is ");
  blk("      to support pAntler launching conventions.           ");
  blk("                                                          ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showExampleConfigAndExit

void showExampleConfigAndExit()
{
  blu("=============================================================== ");
  blu("uFldFireSim Example MOOS Configuration                          ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = uFldFireSim                                     ");
  blk("{                                                               ");
  blk("  AppTick   = 4          // Main app loop frequency Hz         ");
  blk("  CommsTick = 4          // MOOS comms frequency Hz            ");
  blk("                                                                ");
  blk("  // AppCastingMOOSApp settings                                 ");
  blk("  term_report_interval = 0.4 // Default: 0.4s                  ");
  blk("  max_appcast_events   = 8   // Default: 8                     ");
  blk("  max_appcast_run_warnings = 10 // Default: 10                 ");
  blk("                                                                ");
  blk("  // Fire and Region Configuration                              ");
  blk("  fire_config = generate=true                                   ");
  blk("  fire_config = file=fires_field.txt                            ");
  blk("  fire_config = count=40                                        ");
  blk("  fire_config = sep_min=10                                      ");
  blk("  fire_config = region={50,-40: -10,0: 180,0: 180,...}        ");
  blk("  fire_config = spawn_count=0                                   ");
  blk("  fire_config = spawn_interval=300:500                          ");
  blk("  fire_config = save_path=\"missions/UAV_Fly/gen_files/\"        ");
  blk("                                                                ");
  blk("  ignoredRegion_config = generate=true                          ");
  blk("  ignoredRegion_config = file=ignoredRegions.txt                ");
  blk("  ignoredRegion_config = count=1                                ");
  blk("  ignoredRegion_config = sep_min=150                            ");
  blk("  ignoredRegion_config = region={50,-40: -10,0: 180,0: 180,...} ");
  blk("  ignoredRegion_config = spawn_count=0                          ");
  blk("  ignoredRegion_config = spawn_interval=300:500                 ");
  blk("  ignoredRegion_config = save_path=\"missions/UAV_Fly/gen_files/\"");
  blk("                                                                ");
  blk("  // Mission Settings                                           ");
  blk("  mission_duration = 600   // Seconds. Default: 600            ");
  blk("  impute_time      = false // Score undiscovered fires at       ");
  blk("                           // deadline. Default: false         ");
  blk("  mission_score_save_path = missions/scores/ // Save scores here");
  blk("                                                                ");
  blk("  // Sensor Simulation Settings                                 ");
  blk("  show_detect_rng  = true  // Visualize sensor range. Def: true");
  blk("  detect_rng_min   = 25    // Min sensor range (m). Def: 25    ");
  blk("  detect_rng_max   = 40    // Max sensor range (m). Def: 40    ");
  blk("  detect_rng_pd    = 0.5   // P(detection) at min_rng. Def: 0.5");
  blk("  detect_alt_max   = 25    // Max alt for full range (m). Def:25");
  blk("  detect_rng_fixed = true  // No altitude scaling. Def: true   ");
  blk("  scout_rng_transparency = 0.1 // Vis transparency. Def: 0.1   ");
  blk("                                                                ");
  blk("  // Output Visuals and Flags                                   ");
  blk("  fire_color       = red     // For discovered fires. Def: red ");
  blk("  // fire_color    = vehicle // Or use vehicle's color         ");
  blk("  leader_flag      = NEW_LEADER=$[LEADER]                     ");
  blk("  finish_flag      = MISSION_COMPLETE=true                    ");
  blk("  finish_flag      = RETURN_ALL=true                          ");
  blk("}                                                               ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showInterfaceAndExit

void showInterfaceAndExit()
{
  blu("=============================================================== ");
  blu("uFldFireSim MOOS Interface                                      ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  NODE_REPORT = NAME=alpha,TYPE=UAV,TIME=1678886400.0,X=50,... ");
  blk("    // Receives standard node reports from vehicles.           ");
  blk("                                                                ");
  blk("  NODE_REPORT_LOCAL = NAME=bravo,TYPE=UAV,TIME=1678886400.0,...");
  blk("    // Receives node reports from local (shoreside) vehicles.  ");
  blk("                                                                ");
  blk("  XFIRE_ALERT = type=unreg, x=92, y=31                          ");
  blk("    // External command to add/modify a fire (e.g., from GCS). ");
  blk("                                                                ");
  blk("  XDISCOVERED_FIRE = x=-16.5, y=4.3                             ");
  blk("    // External command to declare a fire as discovered.       ");
  blk("                                                                ");
  blk("  SCOUT_REQUEST = vname=ben                                     ");
  blk("    // Vehicle 'ben' requests to scout for fire/ignored region. ");
  blk("                                                                ");
  blk("  MISSION_START_TIME = 1678886400.0 (double, UTC seconds)     ");
  blk("    // Sets the official start time for the mission.           ");
  blk("                                                                ");
  blk("  GSV_VISUALIZE_SENSOR_AREA = true (string: \"true\"/\"false\")   ");
  blk("    // Toggles sensor area visualization (false hides them).    ");
  blk("                                                                ");
  blk("  GSV_COVERAGE_PERCENTAGE = 75.5 (double)                       ");
  blk("    // Input for mission scoring; current map coverage %.      ");
  blk("                                                                ");
  blk("  IGNORED_REGION = reg::x=10,y=20                               ");
  blk("  IGNORED_REGION = unreg::x=10,y=20                             ");
  blk("    // Command to register or unregister an ignored region.    ");
  blk("                                                                ");
  blk("  CHANGE_PLANNER_MODEX = VORONOI_SEARCH (string)                ");
  blk("    // Sets the planner mode, used for scoring.                ");
  blk("                                                                ");
  blk("  XDISABLE_RESET_MISSION = (no value, presence triggers)        ");
  blk("    // Resets mission: un-discovers fires/regions, clears state.");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  DISCOVERED_FIRE = id=f1, finder=cal                           ");
  blk("    // Announces a fire 'f1' was discovered by vehicle 'cal'.  ");
  blk("                                                                ");
  blk("  UFFS_LEADER = ben (string, or \"tie\")                          ");
  blk("    // Current leader in fire discoveries.                     ");
  blk("                                                                ");
  // blk("  UFFS_WINNER = abe (string, or \"pending\")                      ");
  // blk("    // Declared winner of the mission, or 'pending'.           ");
  // blk("                                                                ");
  blk("  UFFS_FINISHED = true (string: \"true\"/\"false\")               ");
  blk("    // Indicates if the mission has finished.                  ");
  blk("                                                                ");
  blk("  MISSION_FINISHED_TIME = 1678887000.0 (double, UTC seconds)  ");
  blk("    // Timestamp when the mission concluded.                   ");
  blk("                                                                ");
  blk("  VIEW_MARKER = x=10,y=20,label=f1,type=diamond,color=red,... ");
  blk("    // Visual marker for fires and ignored region centers.     ");
  blk("                                                                ");
  blk("  VIEW_CIRCLE = x=50,y=50,rad=40,label=sensor_max_abe,...     ");
  blk("    // Visual circle representing sensor ranges.               ");
  blk("                                                                ");
  blk("  VIEW_POLYGON = pts={0,0:100,0:100,100:0,100},label=sregion,...");
  blk("    // Visual polygon for search area or ignored regions.      ");
  blk("                                                                ");
  blk("  SEARCH_REGION = pts={0,0:100,0:100,100:0,100}                 ");
  blk("    // Defines the polygonal search area boundaries.           ");
  blk("                                                                ");
  blk("  VIEW_RANGE_PULSE = x=10,y=20,rad=80,time=1678886500.0,...    ");
  blk("    // Visual pulse effect on fire/region discovery or spawn.  ");
  blk("                                                                ");
  blk("  PLOGGER_CMD = COPY_FILE_REQUEST=fires.txt                     ");
  blk("    // Command for pLogger, e.g., to save config/score files.  ");
  blk("                                                                ");
  blk("  IGNORED_REGION_ALERT = reg::name=r1,points={...}              ");
  blk("  IGNORED_REGION_ALERT = unreg::name=r1                         ");
  blk("    // Alerts about registration/unregistration of ignored regions.");
  blk("                                                                ");
  blk("  XREQUEST_PLANNER_MODE = true (string: \"true\")                 ");
  blk("    // On connect, requests current planner mode from others.  ");
  blk("                                                                ");
  blk("  FS_DEBUG = Invalid incoming node report (string)              ");
  blk("    // Debug messages published by the application.            ");
  blk("                                                                ");
  blk("  // Mission Score Publications:                                ");
  blk("  MISSION_SCORE = 85.5 (double)                               ");
  blk("  TOTAL_FIRES_IN_MISSION = 10 (uint)                          ");
  blk("  TOTAL_FIRES_DISCOVERED_IN_MISSION = 8 (uint)                ");
  blk("  MISSION_ALGORITHM_NAME = VORONOI_CVT (string)               ");
  blk("  MISSION_TOTAL_TIME_S = 598.2 (double)                       ");
  blk("  MISSION_DEADLINE_S = 600.0 (double)                         ");
  blk("  MISSION_IMPUTED_TIME_FLAG = false (bool as string)          ");
  blk("  MISSION_IGNORED_REGIONS_COUNT = 2 (uint)                    ");
  blk("  MISSION_SPAWNED_IGNORED_REGIONS_COUNT = 1 (uint)            ");
  blk("  MISSION_DRONE_COUNT = 3 (uint)                              ");
  blk("                                                                ");
  // blk("  // FIRE_ALERT_VNAME (e.g. FIRE_ALERT_ABE) = x=23,y=148,id=21 ");
  // blk("  //   (Potentially published if internal broadcast code is    ");
  // blk("  //    uncommented; broadcasts fire locations to vehicles.)   ");
  blk("                                                                ");
  blk("PUBLICATIONS (user-configurable flags):                       ");
  blk("------------------------------------                            ");
  blk("  leader_flag = (e.g., NEW_LEADER=$[LEADER])                   ");
  blk("    // MOOS variable posted when the leader changes.           ");
  blk("    // $[LEADER] is replaced with the leader's name or 'tie'.  ");
  blk("                                                                ");
  // blk("  winner_flag = (e.g., NEW_WINNER=$[WINNER])                   ");
  // blk("    // MOOS variable posted when a winner is established.      ");
  // blk("    // $[WINNER] is replaced with the winner's name or 'pending'.");
  // blk("                                                                ");
  blk("  finish_flag = (e.g., MISSION_COMPLETE=true)                 ");
  blk("    // MOOS variable(s) posted when the mission is finished.   ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: sOOowReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  // showReleaseInfo("uFldFireSim", "gpl");
  exit(0);
}