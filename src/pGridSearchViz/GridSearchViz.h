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

#include "XYCircle.h"

struct DroneRecord
{
  std::string name;
  double altitude;
  double sensor_radius;
  XYCircle sensorArea;

  DroneRecord() : name(""), altitude(0), sensor_radius(0), sensorArea() {}

  DroneRecord(std::string name, double altitude, double sensor_radius, XYCircle sensorA)
      : name(name), altitude(altitude), sensor_radius(sensor_radius), sensorArea(sensorA) {}
};

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

  void calculateCoverageStatistics();

  void gridSetCell(const int ix, const double val);
  // Increment the value of the first cell variable ("x") 0 by val
  void gridModifyCell(const int ix, const double val);

protected: // Config vars
  bool m_report_deltas;
  std::string m_grid_label;
  std::string m_grid_var_name;

  ExFilterSet m_filter_set;

  // Sensor data
  double m_sensor_radius_max;
  std::string m_sensor_color;

  // The min time seperation at which covered cells decay in value
  double m_grid_cell_decay_time; // 0 means no decay

protected: // State vars
  XYConvexGrid m_grid;

  std::map<std::string, DroneRecord> m_map_drone_records;
  std::map<unsigned int, double> m_map_deltas;

  std::map<std::string, double> m_map_coverage_statistics;
  double m_missionStartTime;
};
