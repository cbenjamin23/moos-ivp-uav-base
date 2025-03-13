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
#include "XYMarker.h"

#include "IgnoredRegion.h"
#include "TMSTCGridConverter.h"

struct DroneRecord
{
  std::string name;
  double altitude;
  double sensor_radius;

  DroneRecord() : name(""), altitude(0), sensor_radius(0) {}

  DroneRecord(std::string name, double altitude, double sensor_radius)
      : name(name), altitude(altitude), sensor_radius(sensor_radius) {}
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
  void handleMailIgnoredRegionAlert(std::string);

  void registerIgnoredRegion(std::string str);
  void unregisterIgnoredRegion(std::string name);
  XYPolygon parseStringIgnoredRegion(std::string str, std::string type) const;
  XYPolygon stringHexagon2Poly(std::string str) const;
  XYPolygon stringRectangle2Poly(std::string str) const;

  void postGrid();
  void postGridUpdates();

  void calculateCoverageStatistics();
  std::vector<int>::iterator ignoreCellIndex(std::vector<int>::iterator it, std::vector<int> &cell_indices);
  void registerCellIndeces(std::vector<int> &cell_indices);

  void gridSetCell(const int ix, const double val);
  // Increment the value of the first cell variable ("x") 0 by val
  void gridModifyCell(const int ix, const double val);


  void convertGridToTMSTC();
  void postTMSTCGrids(bool active=true);

protected: // Config vars
  bool m_report_deltas;
  std::string m_grid_label;
  std::string m_grid_var_name;
  bool m_visualize_sensor_area;

  ExFilterSet m_filter_set;

  // Sensor data
  std::string m_sensor_color;
  double m_sensor_transparency;
  double m_sensor_radius_max;
  double m_sensor_altitude_max;
  bool m_sensor_radius_fixed;

  // The min time seperation at which covered cells decay in value
  double m_grid_cell_decay_time; // 0 means no decay

protected: // State vars
  XYConvexGrid m_grid;

  std::map<std::string, DroneRecord> m_map_drone_records;
  std::map<unsigned int, double> m_map_deltas;

  std::map<std::string, double> m_map_coverage_statistics;
  double m_missionStartTime;


  std::vector<int> m_valid_cell_indices;
  // Ignored regions cells with names as keys
  std::map<std::string, std::vector<int>> m_map_ignored_cell_indices;

  std::map<std::string, XYPolygon> m_map_ignored_regions_poly;

  TMSTCGridConverter m_tmstc_grid_converter;

};
