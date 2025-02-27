/*****************************************************************/
/*    NAME: Michael Benjamin, Modified by Steve Nomeny                                     */
/*    ORGN: Dept of Mechanical Eng / CSAIL, MIT Cambridge MA     */
/*    FILE: FireSim.h                                            */
/*    DATE: Feb 18th, 2022                                       */
/*                                                               */
/* This file is part of MOOS-IvP                                 */
/*                                                               */
/* MOOS-IvP is free software: you can redistribute it and/or     */
/* modify it under the terms of the GNU General Public License   */
/* as published by the Free Software Foundation, either version  */
/* 3 of the License, or (at your option) any later version.      */
/*                                                               */
/* MOOS-IvP is distributed in the hope that it will be useful,   */
/* but WITHOUT ANY WARRANTY; without even the implied warranty   */
/* of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See  */
/* the GNU General Public License for more details.              */
/*                                                               */
/* You should have received a copy of the GNU General Public     */
/* License along with MOOS-IvP.  If not, see                     */
/* <http://www.gnu.org/licenses/>.                               */
/*****************************************************************/

#ifndef UFLD_FIRE_SENSOR_MOOSAPP_HEADER
#define UFLD_FIRE_SENSOR_MOOSAPP_HEADER

#include <map>
#include <string>
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "NodeRecord.h"
#include "FireSet.h"
#include "VarDataPair.h"

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
  bool handleMailDiscoverRequest(std::string);
  bool handleMailScoutRequest(std::string);

protected: // Outgoing mail utility
  void declareDiscoveredFire(std::string vname, std::string fname);
  void declareScoutedFire(std::string vname, std::string fname);

protected: // Utilities
  void tryDiscovers();
  void tryDiscoversVName(std::string vname);
  void tryDiscoversVNameFire(std::string vname, std::string fname);

  void tryScouts();
  void tryScoutsVName(std::string vname);
  void tryScoutsVNameFire(std::string vname, std::string fname);

  void updateLeaderStatus();
  void updateWinnerStatus(bool finished = false);
  void updateFinishStatus();

  // TODO: Remove dtype... not used
  bool rollDice(std::string vname, std::string, std::string dtype);

  void postDetectRngPolys();
  void postScoutRngPolys();
  void postRangePolys(std::string vname, std::string tag, bool active);
  void postFireMarkers();
  void postFireMarker(std::string fname);

  void postFlags(const std::vector<VarDataPair> &flags);
  void broadcastFires();

  void applyTMateColors();

  void addNotable(std::string vname, std::string fname);
  bool isNotable(std::string fname);

protected: // State variables
  FireSet m_fireset;

  double m_last_broadcast;

  // Key for each map below is the vehicle name.
  std::map<std::string, NodeRecord> m_map_node_records;
  std::map<std::string, std::string> m_map_node_vroles;

  std::map<std::string, double> m_map_node_last_discover_req;
  std::map<std::string, double> m_map_node_last_discover_try;
  std::map<std::string, double> m_map_node_last_discover_utc;
  std::map<std::string, unsigned int> m_map_node_discover_reqs;
  std::map<std::string, unsigned int> m_map_node_discover_tries;
  std::map<std::string, unsigned int> m_map_node_discovers;

  std::map<std::string, double> m_map_node_last_scout_req;
  std::map<std::string, double> m_map_node_last_scout_try;
  std::map<std::string, unsigned int> m_map_node_scout_reqs;
  std::map<std::string, unsigned int> m_map_node_scout_tries;
  std::map<std::string, unsigned int> m_map_node_scouts;

  // Key for this map is the scout vname
  std::map<std::string, std::string> m_map_node_tmate;

  // Notables map key is vname to list of recent fires discovered
  std::map<std::string, std::list<std::string>> m_map_notables;

  unsigned int m_total_discoverers;
  std::string m_vname_leader;
  std::string m_vname_winner;
  bool m_finished;
  bool m_scouts_inplay;

  unsigned int m_known_undiscovered;

protected: // Configuration variables
  std::vector<VarDataPair> m_winner_flags;
  std::vector<VarDataPair> m_leader_flags;
  std::vector<VarDataPair> m_finish_flags;

  double m_detect_rng_min;
  double m_detect_rng_max;
  double m_detect_rng_pd;

  std::string m_fire_color;

  bool m_finish_upon_win;

  bool m_detect_rng_show;
  double m_detect_rng_transparency;
};

#endif
