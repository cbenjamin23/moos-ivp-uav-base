/*****************************************************************/
/*    NAME: Steve Nomeny                                         */
/*    ORGN: NTNU, Trondheim                       */
/*    FILE: MissionOperator.h                                       */
/*    DATE: May 2025                                            */
/*****************************************************************/

#pragma once

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

#include <map>

#include "NodeRecordUtils.h"
#include "XYPolygon.h"
#include "common.h"

#include <chrono>

class MissionOperator : public AppCastingMOOSApp
{
public:
  MissionOperator();
  virtual ~MissionOperator() {}

  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();

protected:
  bool buildReport();
  void registerVariables();
  bool handleMailMissionComplete(std::string str);
  bool handleMailNodeRecord(std::string str);

  void startNewMission();
  void resetMission();
  void switchAlgorithm();
  bool algorithmHasRemainingMissions(Planner::PlannerMode mode);

  bool shouldEnsureAllVehiclesInOpRegion();

  bool allVehiclesInOpRegion();

protected:
  void clearAllGenerateWarnings();

protected: // Config vars
  bool m_isRunningMoosPid;

  XYPolygon m_op_region;

  // Mission operator config parameters
  double m_mission_duration;
  double m_reset_delay;
  std::map<Planner::PlannerMode, unsigned int> m_missions_per_algorithm;

  bool m_missionEnabled;
  Planner::PlannerMode m_planner_mode;

  bool m_missionOperatorEnabled;

  const double RESET_TO_MISSIONSTART_TIME = 5; // seconds

protected: // State vars
  // Mission operator state variables
  bool m_mission_in_progress;
  double m_mission_start_time;

  std::chrono::time_point<std::chrono::steady_clock> m_reset_start_time;

  bool m_waiting_for_reset;
  std::map<Planner::PlannerMode, unsigned int> m_missions_completed;
  unsigned int m_total_missions_completed;
  unsigned int m_total_missions_target;
  std::vector<Planner::PlannerMode> m_algorithm_sequence;
  unsigned int m_current_algorithm_index;
  bool m_all_missions_complete;

  std::vector<std::string> m_generate_warnings;

  bool m_isGSPlannerGridEmpty;
  std::map<std::string, NodeRecord> m_map_drone_records;
};
