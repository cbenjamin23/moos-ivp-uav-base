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

struct DroneRecord
{
  std::string name;
  double altitude;
  double sensor_radius;

  DroneRecord() : name(""), altitude(0), sensor_radius(0) {}

  DroneRecord(std::string name, double altitude, double sensor_radius)
      : name(name), altitude(altitude), sensor_radius(sensor_radius) {}
};

struct PolyRegion
{
  XYPolygon region;
  XYMarker marker;

  std::vector<int> ignored_cell_indices;
  PolyRegion(XYPolygon reg, XYMarker lab) : region(reg), marker(lab) {}
  PolyRegion() = default;
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
  void handleMailIgnoredRegion(std::string);

  void registerIgnoredRegion(std::string str);
  void unregisterIgnoredRegion(std::string str);
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

protected: // Config vars
  bool m_report_deltas;
  std::string m_grid_label;
  std::string m_grid_var_name;
  bool m_visualize_sensor_area; 

  ExFilterSet m_filter_set;

  // Sensor data
  std::string m_sensor_color;
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

  std::vector<PolyRegion> m_ignoredRegions;

  const std::vector<std::string> m_validRegionTypes = {
      "ellipse",  // "ellipse":  Format:  "format=ellipse, msg=val, x=val, y=val, major=val, minor=val, pts=val, degs=val, snap_value=val"
      "radial",   // "circle":   Format:  "format=circle, msg=val, x=val, y=val, radius=val, pts=val, snap=val"
      "oval",     // "oval":     Format:  "format=oval, msg=val, x=val, y=val, rad=val, len=val, draw_degs=val" // len > 2*rad
      "hexagon",  // "pylon":    Format:  "format=hexagon, msg=val, x=val, y=val rad=val, pts=val, snap_val=val
      "rectangle" //  "rectangle"  //"rectangle": Format:  "format=rectangle, msg=val, cx=val, cy=val, width=val, height=val, degs=val"
  };
  const double REGION_MARKER_WIDTH = 10;
};
