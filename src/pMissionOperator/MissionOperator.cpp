/*****************************************************************/
/*    NAME: Steve Nomeny                                         */
/*    ORGN: NTNU, Trondheim                       */
/*    FILE: MissionOperator.cpp                                       */
/*    DATE: May 2025                                            */
/*****************************************************************/

#include <iterator>
#include "MissionOperator.h"
#include "MBUtils.h"
#include "ACTable.h"

#include "XYFormatUtilsPoly.h"

#include "common.h"
#include "Logger.h"

//---------------------------------------------------------
// Constructor()

MissionOperator::MissionOperator()
{
  // Initialize mission operator variables
  m_missionEnabled = false;
  m_isRunningMoosPid = false;
  m_mission_in_progress = false;
  m_mission_start_time = 0;
  m_reset_start_time = std::chrono::steady_clock::now();
  m_waiting_for_reset = false;
  m_total_missions_completed = 0;
  m_total_missions_target = 0;
  m_current_algorithm_index = 0;
  m_all_missions_complete = false;
  m_mission_duration = 600; // Default 10 minutes in MOOSTime
  m_reset_delay = 2;        // Default 2 seconds delay
  m_missionOperatorEnabled = false;
  m_isGSPlannerGridEmpty = true; // Assume the grid of GridSearchPlanner is empty at startup

  // Default planner mode
  m_planner_mode = Planner::PlannerMode::VORONOI_SEARCH;
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool MissionOperator::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  bool handled = false;
  for (p = NewMail.begin(); p != NewMail.end(); p++)
  {
    CMOOSMsg &msg = *p;

    std::string key = msg.GetKey();
    std::string sval = msg.GetString();
    double dval = msg.GetDouble();
    // double mtime = msg.GetTime();
    // bool   mdbl  = msg.IsDouble();
    // bool   mstr  = msg.IsString();
    // string msrc  = msg.GetSource();
    // std::string community = msg.GetCommunity();

    if (msg.GetSource() == m_sAppName)
      continue;


    if (key == "XREQUEST_PLANNER_MODE")
    {
      Notify("CHANGE_PLANNER_MODEX", Planner::modeToString(m_planner_mode));
      handled = true;
    }

    else if (key == "MISSION_COMPLETE")
      handled = handleMailMissionComplete(sval);

    else if (key == "XMISSION_OPERATOR_ENABLE")
      handled = setBooleanOnString(m_missionOperatorEnabled, sval);

    else if (key == "NODE_REPORT")
      handled = handleMailNodeRecord(sval);

    else if (key == "XGSP_GRID_EMPTY")
      handled = setBooleanOnString(m_isGSPlannerGridEmpty, sval);


    if (!handled)
    {
      reportRunWarning("Unhandled mail: " + key);
    }
  }

  return (true);
}

//---------------------------------------------------------
// Procedure: handleMailMissionComplete()

bool MissionOperator::handleMailMissionComplete(std::string sval)
{
  Logger::info("Received mission complete message: " + sval);
  bool mission_complete = false;
  if (!setBooleanOnString(mission_complete, sval) || !mission_complete)
    return false;

  if (m_mission_in_progress)
  {
    m_mission_in_progress = false;

    // Record completion of this mission
    m_missions_completed[m_planner_mode]++;
    m_total_missions_completed++;

    // Start reset procedure
    m_waiting_for_reset = true;
    m_reset_start_time = std::chrono::steady_clock::now();

    Logger::info("Mission complete received. Starting reset delay of " +
                 doubleToStringX(m_reset_delay, 1) + " seconds.");

    return true;
  }

  if(m_missionOperatorEnabled)
  {
    Logger::info("Mission l received but no mission in progress.");
    return true;
  }

  return false;
}

//---------------------------------------------------------
// Procedure: handleMailNodeRecord()

bool MissionOperator::handleMailNodeRecord(std::string sval)
{
  NodeRecord node_record = string2NodeRecord(sval);

  if (!node_record.valid())
  {
    Logger::error("Invalid node record received: " + sval);
    return false;
  }

  std::string name = node_record.getName();
  m_map_drone_records[name] = node_record;

  return true;
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool MissionOperator::OnConnectToServer()
{
  registerVariables();
  return (true);
}

//---------------------------------------------------------
// Procedure: Iterate()

bool MissionOperator::Iterate()
{
  AppCastingMOOSApp::Iterate();

  // Check if all missions are complete or if the mission operator is disabled
  if (m_all_missions_complete || !m_missionOperatorEnabled)
  {
    // Nothing more to do
    AppCastingMOOSApp::PostReport();
    return true;
  }

  double current_time = MOOSTime();

  // Check if we need to reset the mission after delay
  if (m_waiting_for_reset)
  {

    bool validVehiclePositions = true;
    if (shouldEnsureAllVehiclesInOpRegion())
      validVehiclePositions = allVehiclesInOpRegion();

    bool timeout = (std::chrono::steady_clock::now() - m_reset_start_time) >= std::chrono::seconds(static_cast<int>(m_reset_delay));
    if (timeout && validVehiclePositions)
    {
      m_waiting_for_reset = false;
      resetMission();

      // Check if we need to switch algorithms or if we're done with all missions
      if (!algorithmHasRemainingMissions(m_planner_mode))
        switchAlgorithm();
      else // Start a new mission with the same algorithm
        startNewMission();
    }
  }
  // Check if a mission is in progress and if it has timed out
  else if (m_mission_in_progress)
  {
    constexpr double DELAY = 15; // seconds
    if ((current_time - m_mission_start_time) >= m_mission_duration + DELAY)
    {
      Logger::warning("Mission timed out after " +
                      doubleToStringX(m_mission_duration, 1) + " seconds");
      // reportRunWarning("Mission timed out");

      // Handle timeout the same as mission complete
      m_mission_in_progress = false;
      m_missions_completed[m_planner_mode]++;
      m_total_missions_completed++;

      // Start reset procedure
      m_waiting_for_reset = true;
      m_reset_start_time = std::chrono::steady_clock::now();
    }
  }
  // If no mission is in progress and we're not waiting for reset, start a new mission
  else if (!m_mission_in_progress && !m_waiting_for_reset && !m_all_missions_complete)
  {
    startNewMission();
  }

  AppCastingMOOSApp::PostReport();
  return (true);
}

//---------------------------------------------------------
// Procedure: startNewMission()

void MissionOperator::startNewMission()
{

  if (std::chrono::steady_clock::now() < m_reset_start_time + std::chrono::seconds(static_cast<int>(RESET_TO_MISSIONSTART_TIME)))
    return;

  if (!m_isGSPlannerGridEmpty)
  {
    Logger::info("GridSearchPlanner grid is not empty, not starting new mission");
    return;
  }

  // Set the current planner mode if it's not already set
  Notify("CHANGE_PLANNER_MODEX", Planner::modeToString(m_planner_mode));
  Notify("CHANGE_PLANNER_MODE_ALL", Planner::modeToString(m_planner_mode));

  // Enable the mission
  m_missionEnabled = true;
  Notify("XENABLE_MISSION", "true");

  // Record the start time
  m_mission_start_time = MOOSTime();
  m_mission_in_progress = true;

  m_isGSPlannerGridEmpty = false; // The grid of GridSearchPlanner is not empty at mission start

  Logger::info("Starting new mission with algorithm: " +
               Planner::modeToString(m_planner_mode) +
               " (" + intToString(m_missions_completed[m_planner_mode] + 1) +
               "/" + intToString(m_missions_per_algorithm[m_planner_mode]) + ")");
}

//---------------------------------------------------------
// Procedure: resetMission()

void MissionOperator::resetMission()
{
  // Reset the mission state
  m_missionEnabled = false;
  // Notify("XENABLE_MISSION", "false");
  Notify("XDISABLE_RESET_MISSION", "true");

  Logger::info("Mission reset completed");

  m_reset_start_time = std::chrono::steady_clock::now();
}

//---------------------------------------------------------
// Procedure: switchAlgorithm()

void MissionOperator::switchAlgorithm()
{
  // Check if we have more algorithms to try
  if (m_current_algorithm_index < m_algorithm_sequence.size() - 1)
  {
    // Move to the next algorithm
    m_current_algorithm_index++;
    m_planner_mode = m_algorithm_sequence[m_current_algorithm_index];

    Logger::info("Switching to algorithm: " + Planner::modeToString(m_planner_mode));
  }
  else
  {
    // All algorithms have completed their missions
    m_all_missions_complete = true;

    std::string msg = "All " + intToString(m_total_missions_target) +
                      " missions completed across all algorithms";
    reportEvent(msg);
    Logger::info(msg);
  }
}

//---------------------------------------------------------
// Procedure: algorithmHasRemainingMissions()

bool MissionOperator::algorithmHasRemainingMissions(Planner::PlannerMode mode)
{
  return m_missions_completed[mode] < m_missions_per_algorithm[mode];
}

bool MissionOperator::shouldEnsureAllVehiclesInOpRegion()
{
  return (m_planner_mode == Planner::PlannerMode::TMSTC_STAR) ||
         (m_planner_mode == Planner::PlannerMode::VORONOI_SEARCH && !algorithmHasRemainingMissions(m_planner_mode));
}

//---------------------------------------------------------
// Procedure: OnStartUp()

bool MissionOperator::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  CMOOSApp::OnStartUp();

  std::list<std::string> sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if (m_MissionReader.GetConfiguration(GetAppName(), sParams))
  {

    std::list<std::string>::reverse_iterator p;
    for (p = sParams.rbegin(); p != sParams.rend(); p++)
    {
      std::string orig = *p;
      std::string line = *p;
      std::string param = tolower(biteStringX(line, '='));
      std::string value = line;

      // Logger::info("OnStartUp: " + orig);

      bool handled = false;
      if (param == "is_running_moos_pid")
        handled = setBooleanOnString(m_isRunningMoosPid, value);
      // Mission operator parameters
      else if (param == "mission_duration")
      {
        handled = setDoubleOnString(m_mission_duration, value);
        if (handled)
          Logger::info("Mission duration set to " + doubleToStringX(m_mission_duration, 1) + " seconds");
      }
      else if (param == "reset_delay")
      {
        handled = setDoubleOnString(m_reset_delay, value);
        if (handled)
          Logger::info("Reset delay set to " + doubleToStringX(m_reset_delay, 1) + " seconds");
      }
      else if ((param == "voronoi_search_missions") ||
               (param == "voronoi_missions"))
      {
        int missions = 0;
        handled = setIntOnString(missions, value);
        if (handled && missions > 0)
        {
          m_missions_per_algorithm[Planner::PlannerMode::VORONOI_SEARCH] = missions;
          m_total_missions_target += missions;

          // Add to algorithm sequence if it's not already there
          if (std::find(m_algorithm_sequence.begin(), m_algorithm_sequence.end(),
                        Planner::PlannerMode::VORONOI_SEARCH) == m_algorithm_sequence.end())
          {
            m_algorithm_sequence.push_back(Planner::PlannerMode::VORONOI_SEARCH);
          }

          Logger::info("Added " + intToString(missions) + " VORONOI_SEARCH missions");
        }
      }
      else if ((param == "tmstc_star_missions") ||
               (param == "tmstc_missions"))
      {
        int missions = 0;
        handled = setIntOnString(missions, value);
        if (handled && missions > 0)
        {
          m_missions_per_algorithm[Planner::PlannerMode::TMSTC_STAR] = missions;
          m_total_missions_target += missions;

          // Add to algorithm sequence if it's not already there
          if (std::find(m_algorithm_sequence.begin(), m_algorithm_sequence.end(),
                        Planner::PlannerMode::TMSTC_STAR) == m_algorithm_sequence.end())
          {
            m_algorithm_sequence.push_back(Planner::PlannerMode::TMSTC_STAR);
          }

          Logger::info("Added " + intToString(missions) + " TMSTC_STAR missions");
        }
      }
      else if (param == "planner_mode")
      {
        std::string mode_str = toupper(value);

        try
        {
          m_planner_mode = Planner::stringToMode(mode_str);
          handled = true;
        }
        catch (std::exception &e)
        {
          std::string msg = "Failed to set planner mode. Exception: " + std::string(e.what());
          m_generate_warnings.push_back(msg);
          Logger::error("OnStartUp:" + msg);
          reportRunWarning(msg);
          handled = false;
        }
      }
      else if (param == "op_region")
      {
        m_op_region = string2Poly(value);
        handled = true;
      }
      else if (param == "mission_operator_enable")
      {
        handled = setBooleanOnString(m_missionOperatorEnabled, value);
        if (handled)
          Logger::info("Mission operator enabled: " + boolToString(m_missionOperatorEnabled));
      }

      if (!handled)
        reportUnhandledConfigWarning(orig);
    }
  }

  if (!m_op_region.is_convex())
  {
    reportConfigWarning("Operational region is not convex");
    Logger::error("Operational region is not convex");
  }

  // Initialize algorithm tracking if none were specified
  if (m_algorithm_sequence.empty())
  {
    // Default to current planner mode with 1 mission
    m_algorithm_sequence.push_back(m_planner_mode);
    m_missions_per_algorithm[m_planner_mode] = 1;
    m_total_missions_target = 1;

    reportConfigWarning("No missions specified for any algorithm. Defaulting to 1 mission with " +
                        Planner::modeToString(m_planner_mode));
  }

  // Initialize the completed missions counters
  for (const auto &pair : m_missions_per_algorithm)
  {
    m_missions_completed[pair.first] = 0;
  }

  // Set initial algorithm
  m_current_algorithm_index = 0;
  m_planner_mode = m_algorithm_sequence[m_current_algorithm_index];

  Notify("CHANGE_PLANNER_MODEX", Planner::modeToString(m_planner_mode));

  for(const auto& algo : m_algorithm_sequence) {
    Logger::info("Algorithm que " + Planner::modeToString(algo));
  }

  registerVariables();

  return (true);
}

//------------------------------------------------------------
// Procedure: registerVariables()

void MissionOperator::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();

  // Register("CHANGE_PLANNER_MODEX", 0);
  Register("XREQUEST_PLANNER_MODE", 0);

  // Register for mission completion notification
  Register("MISSION_COMPLETE", 0);

  Register("XMISSION_OPERATOR_ENABLE", 0);

  Register("NODE_REPORT", 0);

  Register("XGSP_GRID_EMPTY", 0);
}

void MissionOperator::clearAllGenerateWarnings()
{
  for (const auto &warning : m_generate_warnings)
    retractRunWarning(warning);

  m_generate_warnings.clear();
}

bool MissionOperator::allVehiclesInOpRegion()
{
  // Check if all vehicles are in the operational region
  for (const auto &pair : m_map_drone_records)
  {
    const NodeRecord &node_record = pair.second;
    auto node_x = node_record.getX();
    auto node_y = node_record.getY();

    if (!m_op_region.contains(node_x, node_y))
      return false;
  }
  return true;
}

//------------------------------------------------------------
// Procedure: buildReport()

bool MissionOperator::buildReport()
{

  m_msgs << "Mission Operator Status" << std::endl;
  m_msgs << "---------------------------------" << std::endl;
  m_msgs << "  Mission operator enabled: " << boolToString(m_missionOperatorEnabled) << std::endl;
  m_msgs << "           Mission enabled: " << boolToString(m_missionEnabled) << std::endl;
  m_msgs << "       Mission in progress: " << boolToString(m_mission_in_progress) << std::endl;
  m_msgs << "         Waiting for reset: " << boolToString(m_waiting_for_reset) << std::endl;
  m_msgs << "     All missions complete: " << boolToString(m_all_missions_complete) << std::endl;
  m_msgs << "     Reset delay (seconds): " << doubleToStringX(m_reset_delay, 1) << std::endl;
  m_msgs << "     Running SIMULATOR PID: " << boolToString(m_isRunningMoosPid) << std::endl;
  m_msgs << " GridSearchPlanner grid empty: " << boolToString(m_isGSPlannerGridEmpty) << std::endl;
  m_msgs << std::endl;
  if (m_mission_in_progress)
  {
    double elapsed =  m_curr_time - m_mission_start_time;
    m_msgs << "     Mission time elapsed: " << doubleToStringX(elapsed, 1) << " seconds" << std::endl;
    m_msgs << "       Mission timeout at: " << doubleToStringX(m_mission_duration, 1) << " seconds" << std::endl;
    m_msgs << "           Time remaining: " << doubleToStringX(m_mission_duration - elapsed, 1) << " seconds" << std::endl;
  }

  if (m_waiting_for_reset)
  {
    double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - m_reset_start_time).count();
    m_msgs << "       Reset time elapsed: " << doubleToStringX(elapsed, 1) << " seconds" << std::endl;
    m_msgs << "   Reset will complete at: " << doubleToStringX(m_reset_delay, 1) << " seconds" << std::endl;
    m_msgs << "     Reset time remaining: " << doubleToStringX(m_reset_delay - elapsed, 1) << " seconds" << std::endl;
  }

  m_msgs << std::endl;

  // Mission statistics
  m_msgs << "Mission Statistics" << std::endl;
  m_msgs << "---------------------------------" << std::endl;
  m_msgs << "      Current algorithm: " << Planner::modeToString(m_planner_mode) << std::endl;
  m_msgs << "     Completed missions: " << intToString(m_total_missions_completed) << "/" << intToString(m_total_missions_target) << std::endl;

  // Create a table for algorithm statistics
  ACTable actab(3);
  actab << "Algorithm" << "Completed" << "Total";
  actab.addHeaderLines();

  for (const auto &pair : m_missions_per_algorithm)
  {
    std::string algo_name = Planner::modeToString(pair.first);
    unsigned int completed = m_missions_completed[pair.first];
    unsigned int total = pair.second;
    actab << algo_name << intToString(completed) << intToString(total);
  }

  m_msgs << actab.getFormattedString() << std::endl;
  m_msgs << std::endl;

  return (true);
}
