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

#include "IgnoredRegion.h"
#include "TMSTCGridConverter.h"

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
  void handleMailNodeReport(std::string);
  void handleMailIgnoredRegionAlert(std::string);

protected:
  void registerIgnoredRegion(std::string str);
  void unregisterIgnoredRegion(std::string name);

  void doPlanPaths();

  void convertGridToTMSTC();
  void postTMSTCGrids(bool active = true);

  void updateTMSTCVehiclePositions();
  void updateTMSTCIgnoredRegions();

protected: // Config vars
  // Sensor data
  double m_sensor_radius;

  bool m_visualize_planner_grids;

protected: // State vars
  bool m_do_plan_paths;

  // key is name of drone
  std::map<std::string, XYPoint> m_map_drone_positions;
  // key is name of ignored region
  std::map<std::string, XYPolygon> m_map_ignored_regions_poly;

  TMSTCGridConverter m_tmstc_grid_converter;
};
