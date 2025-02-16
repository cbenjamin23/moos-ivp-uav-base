/*****************************************************************/
/*    NAME: Steve Nomeny                                         */
/*    ORGN: NTNU, Trondheim                       */
/*    FILE: GridSearchViz.h                                       */
/*    DATE: Feb 2025                                            */
/*****************************************************************/

#pragma once

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "XYConvexGrid.h"
#include "ExFilterSet.h"

class GridSearchViz : public AppCastingMOOSApp
{
public:
  GridSearchViz();
  virtual ~GridSearchViz() {}

  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();

protected:
  bool buildReport();
  void registerVariables();
  void handleMailNodeReport(std::string);

  void postGrid();
  void postGridUpdates();

protected: // Config vars
  bool m_report_deltas;
  std::string m_grid_label;
  std::string m_grid_var_name;

  ExFilterSet m_filter_set;

  // Sensor data
  double m_sensor_radius_max;

protected: // State vars
  XYConvexGrid m_grid;

  std::map<std::string, double> m_map_drone_sensor_radius;
  std::map<unsigned int, double> m_map_deltas;
};
