/*****************************************************************/
/*    NAME: Steve Nomeny                                         */
/*    ORGN: NTNU, Trondheim                       */
/*    FILE: GridSearchPlanner.h                                       */
/*    DATE: Feb 2025                                            */
/*****************************************************************/

#pragma once

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

#include "XYCircle.h"
#include "XYMarker.h"
#include "NodeRecord.h"

#include "IgnoredRegion.h"
#include "TMSTCGridConverter.h"
#include "TMSTCStar.h"

class GridSearchPlanner : public AppCastingMOOSApp
{
public:
  GridSearchPlanner();
  virtual ~GridSearchPlanner() {}

  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();

protected:
  bool buildReport();
  void registerVariables();
  bool handleMailNodeReport(std::string);
  bool handleMailIgnoredRegionAlert(std::string);

protected:
  void registerIgnoredRegion(std::string str);
  void unregisterIgnoredRegion(std::string name);

  void doPlanPaths();

  void distributePathsToVehicles(Mat paths);
  void notifyCalculatedPathsAndExecute(bool executePath = false);
  void postCalculatedPaths(bool visible = true);

  void convertGridToTMSTC();
  void postTMSTCGrids(bool visible = true);

  void updateTMSTCVehiclePositions();
  void updateTMSTCIgnoredRegions();

  void clearAllGenerateWarnings();

protected: // Config vars
  // Sensor data
  double m_sensor_radius;
  double m_region_grid_size_ratio;
  bool m_isRunningMoosPid;

  bool m_visualize_planner_grids;
  bool m_visualize_planner_paths;
  int m_map_print_version;

  bool m_start_point_closest;

  std::string m_path_publish_variable;

  bool m_missionEnabled;

protected: // State vars
  bool m_do_plan_paths;

  bool m_is_paths_calculated;

  // key is name of drone
  std::map<std::string, NodeRecord> m_map_drone_records;
  std::map<std::string, XYSegList> m_map_drone_paths;
  // key is name of ignored region
  std::map<std::string, XYPolygon> m_map_ignored_regions_poly;

  TMSTCGridConverter m_tmstc_grid_converter;
  std::unique_ptr<TMSTCStar> m_tmstc_star_ptr;

  std::vector<std::string> m_generate_warnings;
};
