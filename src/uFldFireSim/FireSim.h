/*****************************************************************/
/*    NAME: Michael Benjamin, Modified by Steve Nomeny           */
/*    FILE: FireSim.h                                            */
/*    DATE: Feb 2022                                             */
/*                                                               */
/*****************************************************************/

#ifndef UFLD_FIRE_SENSOR_MOOSAPP_HEADER
#define UFLD_FIRE_SENSOR_MOOSAPP_HEADER

#include <map>
#include <string>
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "NodeRecord.h"
#include "VarDataPair.h"

#include "FireSet.h"
#include "FireMissionScorer.h"
#include "IgnoredRegionSet.h"
#include "common.h"

constexpr double FIREMARKER_WIDTH = 20;
constexpr double FIREMARKER_TRANSPARENCY_UNDISC = 0.3;
constexpr double FIREMARKER_TRANSPARENCY_DISC = 0.5;
constexpr double FIREMARKER_TRANSPARENCY_DISC_NOTABLE = 0.7;

constexpr double FIRE_PULSE_RANGE = 80; // moosDistance
constexpr double PULSE_DURATION = 6;

constexpr double IGNORED_REGION_PULSE_RANGE = 90; // moosDistance
constexpr double IGNORED_REGION_MARKER_TRANSPARENCY_UNDISC = 0.1;
constexpr double IGNORED_REGION_MARKER_TRANSPARENCY_DISC = 0.6;

class FireSim : public AppCastingMOOSApp
{
public:
  FireSim();
  virtual ~FireSim() {}

public: // Standard MOOSApp functions to overload
  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();

protected: // Standard AppCastingMOOSApp function to overload
  bool buildReport();
  void registerVariables();

protected: // Configuration utility
  bool handleConfigDetectRangeMin(std::string);
  bool handleConfigDetectRangeMax(std::string);

protected: // Incoming mail utility
  bool handleMailNodeReport(const std::string &);
  bool handleMailScoutRequest(std::string);
  bool handleConfigDetectRangePd(std::string);
  bool handleMailVisualizeSensorArea(std::string);
  bool handleMailIgnoredRegion(std::string);
  bool handleMailDisableResetMission(std::string &warning);
  void retractRunWarnings(std::vector<std::string>& warnings);
  bool handleMissionScoreSavePath(std::string path);



protected: // Outgoing mail utility
  void declareDiscoveredFire(std::string vname, std::string fname);
  void declareDiscoveredIgnoredRegion(std::string vname, std::string rname);
  void notifyUnregIgnoredRegions();

protected: // Utilities
  void registerRemoveIgnoredRegion(std::string pos_str, bool doRegister = true);

  void tryScouts();
  void tryScoutsVName(std::string vname);
  void tryScoutsVNameFire(std::string vname, std::string fname);
  void tryScoutsVNameIgnoredRegion(std::string vname, std::string ignoredRegion);

  void trySpawnFire();
  void trySpawnIgnoredRegion();

  void updateLeaderStatus();
  void updateWinnerStatus(bool finished = false);
  void updateFinishStatus();
  void calculateMissionScore(bool imputeTime = false);

  bool isMissionDeadlineReached() const { return (MOOSTime() >= (m_mission_start_utc + m_mission_duration_s)); }
  bool isMissionRunning() const { return (m_mission_start_utc) && !m_finished; }

  bool rollDiceFire(std::string vname, std::string fname);
  bool rollDiceIgnoredRegion(std::string vname, std::string rname);

  double altScaledRange(double range_limit, std::string vname) const;

  void postScoutRngPolys();
  void postRangePolys(std::string vname, bool active);
  void postFireMarkers();
  void postFireMarker(std::string fname);
  void postFirePulseMessage(Fire fire, double time, std::string discoverer = "");

  void postIgnoredRegions();
  void postIgnoredRegion(std::string rname);
  void postIgnoredRegionPulseMessage(IgnoredRegion ignoredRegion, double time, std::string discoverer = "");

  void postFlags(const std::vector<VarDataPair> &flags);
  void broadcastFires();

  void addNotable(std::string vname, std::string fname);
  bool isNotable(std::string fname);


protected: // State variables
  FireSet m_fireset;
  IgnoredRegionSet m_ignoredRegionset;

  double m_last_broadcast;

  // Key for each map below is the vehicle name.
  std::map<std::string, NodeRecord> m_map_node_records;
  std::map<std::string, std::string> m_map_node_vroles;
  std::map<std::string, double> m_map_node_last_discover_req;
  std::map<std::string, std::list<std::string>> m_map_notables;

  std::map<std::string, unsigned int> m_map_node_discoveries;

  std::map<std::string, double> m_map_node_last_scout_req;
  std::map<std::string, double> m_map_node_last_scout_try;
  std::map<std::string, double> m_map_node_last_discover_utc;
  std::map<std::string, unsigned int> m_map_node_scout_reqs;
  std::map<std::string, unsigned int> m_map_node_scout_tries;



  // Notables map key is vname to list of recent fires discovered
  // std::map<std::string, std::list<std::string>> m_map_notables;

  unsigned int m_total_discoverers;
  std::string m_vname_leader;
  std::string m_vname_winner;
  bool m_scouts_inplay;
  bool m_finished;

  double m_mission_start_utc;   // Time at which the mission starts
  double m_mission_duration_s;  // Duration of the mission
  double m_mission_endtime_utc; // Time at which the mission ends

  FireMissionScorer m_mission_scorer; // Mission scoring object
  bool m_imputeTime; // if true, fires not discovered by deadline are given a time of discovery equal to the deadline

  Planner::PlannerMode m_planner_mode;

  std::string m_mission_score_save_path;

protected: // Configuration variables
  std::vector<VarDataPair> m_winner_flags;
  std::vector<VarDataPair> m_leader_flags;
  std::vector<VarDataPair> m_finish_flags;

  double m_detect_rng_min;
  double m_detect_rng_max;
  double m_detect_rng_pd;
  double m_detect_alt_max;
  bool m_detect_rng_fixed;

  std::string m_fire_color;
  bool m_fire_color_from_vehicle;

  bool m_scout_rng_show;
  double m_scout_rng_transparency;
};

#endif
