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
#include "FireSet.h"
#include "VarDataPair.h"
#include "FireMissionScorer.h"

constexpr double FIREMARKER_WIDTH = 20;
constexpr double FIREMARKER_TRANSPARENCY_UNDISC = 0.3;
constexpr double FIREMARKER_TRANSPARENCY_DISC = 0.5;
constexpr double FIREMARKER_TRANSPARENCY_DISC_NOTABLE = 0.7;

constexpr double FIRE_PULSE_RANGE = 60; //moosDistance
constexpr double FIRE_PULSE_DURATION = 4; 


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
  bool handleConfigDetectRangePd(std::string);

protected: // Incoming mail utility
  bool handleMailNodeReport(const std::string &);
  // bool handleMailDiscoverRequest(std::string);
  bool handleMailScoutRequest(std::string);

protected: // Outgoing mail utility
  void declareDiscoveredFire(std::string vname, std::string fname);
  // void declareScoutedFire(std::string vname, std::string fname);

protected: // Utilities
  void tryScouts();
  void tryScoutsVName(std::string vname);
  void tryScoutsVNameFire(std::string vname, std::string fname);

  void trySpawnFire();

  void updateLeaderStatus();
  void updateWinnerStatus(bool finished = false);
  void updateFinishStatus();
  void calculateMissionScore(bool imputeTime = false);

  bool isMissionDeadlineReached() const {return (MOOSTime() >= (m_mission_start_utc + m_mission_duration_s)); }
  bool isMissionRunning() const { return (m_mission_start_utc) && !m_finished; }

  bool rollDice(std::string vname, std::string);

  double altScaledRange(double range_limit, std::string vname) const;

  void postScoutRngPolys();
  void postRangePolys(std::string vname, bool active);
  void postFireMarkers();
  void postFireMarker(std::string fname);
  void postPulseMessage(Fire fire, double time,  std::string discoverer="");

  void postFlags(const std::vector<VarDataPair> &flags);
  void broadcastFires();

  void addNotable(std::string vname, std::string fname);
  bool isNotable(std::string fname);

protected: // State variables
  FireSet m_fireset;

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
  // std::map<std::string, unsigned int> m_map_node_scouts;

  // Key for this map is the scout vname
  // std::map<std::string, std::string> m_map_node_tmate;

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
  bool m_imputeTime;

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
