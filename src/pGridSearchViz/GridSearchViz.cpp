/*****************************************************************/
/*    NAME: Steve Nomeny                                         */
/*    ORGN: NTNU, Trondheim                       */
/*    FILE: GridSearchViz.cpp                                       */
/*    DATE: Feb 2025                                            */
/*****************************************************************/

#include <iterator>
#include "GridSearchViz.h"
#include "MBUtils.h"
#include "NodeRecord.h"
#include "NodeRecordUtils.h"
#include "XYFormatUtilsConvexGrid.h"
#include "XYGridUpdate.h"
#include "ACTable.h"
#include "XYFormatUtilsPoly.h"

#include "../lib_common/Logger.h"

#include <numeric>
#include <algorithm>

//---------------------------------------------------------
// Constructor()

GridSearchViz::GridSearchViz()
{
  m_report_deltas = true;
  m_grid_label = "gsv";
  m_grid_var_name = "VIEW_GRID";
  m_sensor_radius_max = 10;
  m_sensor_color = "black";
  m_missionStartTime = 0;
  m_map_coverage_statistics["coverage_%"] = 0;

  m_grid_cell_decay_time = 0; // 0 means no decay
  m_sensor_radius_fixed = true;
  m_sensor_altitude_max = 25;
  m_visualize_sensor_area = true; 
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool GridSearchViz::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for (p = NewMail.begin(); p != NewMail.end(); p++)
  {
    CMOOSMsg &msg = *p;

    std::string key = msg.GetKey();
    std::string sval = msg.GetString();
    // double dval  = msg.GetDouble();
    // double mtime = msg.GetTime();
    // bool   mdbl  = msg.IsDouble();
    // bool   mstr  = msg.IsString();
    // string msrc  = msg.GetSource();
    std::string community = msg.GetCommunity();

    bool ok_community = m_filter_set.filterCheckVName(community);
    if (!ok_community)
      continue;

    if ((key == "NODE_REPORT") || (key == "NODE_REPORT_LOCAL"))
      handleMailNodeReport(sval);
    else if (key == "GSV_RESET_GRID")
      m_grid.reset();
    else if (key == "IGNORED_REGION")
      handleMailIgnoredRegion(sval);
    else if (key == "GSV_VISUALIZE_SENSOR_AREA")
      setBooleanOnString(m_visualize_sensor_area, sval);
  }

  return (true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool GridSearchViz::OnConnectToServer()
{
  registerVariables();
  return (true);
}

//---------------------------------------------------------
// Procedure: Iterate()

bool GridSearchViz::Iterate()
{
  AppCastingMOOSApp::Iterate();

  calculateCoverageStatistics();

  if (m_report_deltas)
    postGridUpdates();
  else
    postGrid();

  AppCastingMOOSApp::PostReport();
  return (true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()

bool GridSearchViz::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  CMOOSApp::OnStartUp();

  std::string grid_config;

  std::list<std::string> sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if (m_MissionReader.GetConfiguration(GetAppName(), sParams))
  {

    std::list<std::string>::reverse_iterator p;
    for (p = sParams.rbegin(); p != sParams.rend(); p++)
    {
      std::string orig = *p;
      std::string line = *p;
      std::string param = tolower(biteStringX(line, '='));
      std::string value = line;

      // Logger::info("OnStartUp: " + orig);

      bool handled = false;
      if (param == "grid_config")
      {
        unsigned int len = grid_config.length();
        if ((len > 0) && (grid_config.at(len - 1) != ','))
          grid_config += ",";
        grid_config += value;
        handled = true;
      }
      else if (param == "report_deltas")
        handled = setBooleanOnString(m_report_deltas, value);
      else if (param == "ignore_name")
        handled = m_filter_set.addIgnoreName(value);
      else if (param == "match_name")
        handled = m_filter_set.addMatchName(value);
      else if (param == "grid_label")
        handled = setNonWhiteVarOnString(m_grid_label, value);
      else if (param == "grid_var_name")
        handled = setNonWhiteVarOnString(m_grid_var_name, toupper(value));
      else if (param == "sensor_radius")
        handled = setDoubleOnString(m_sensor_radius_max, value);
      else if (param == "sensor_color")
      {
        m_sensor_color = value;
        handled = true;
      }
      else if (param == "sensor_altitude_max")
        handled = setDoubleOnString(m_sensor_altitude_max, value);
      else if (param == "sensor_radius_fixed")
        handled = setBooleanOnString(m_sensor_radius_fixed, value);
      else if (param == "grid_cell_decay_time")
        handled = setDoubleOnString(m_grid_cell_decay_time, value);
      else if (param == "visualize_sensor_area")
        handled = setBooleanOnString(m_visualize_sensor_area, value);

      if (!handled)
        reportUnhandledConfigWarning(orig);
    }
  }

  m_grid = string2ConvexGrid(grid_config);

  if (m_grid.size() == 0)
    reportConfigWarning("Unsuccessful ConvexGrid construction.");

  m_grid.set_label("gsv");

  m_grid.set_transparency(0.2);

  // create a increasing vector to m_grid.size()
  m_valid_cell_indices.resize(m_grid.size());
  std::iota(m_valid_cell_indices.begin(), m_valid_cell_indices.end(), 0);

  postGrid();
  registerVariables();
  return (true);
}

//------------------------------------------------------------
// Procedure: registerVariables()

void GridSearchViz::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NODE_REPORT_LOCAL", 0);
  Register("NODE_REPORT", 0);
  Register("GSV_RESET_GRID", 0);

  Register("IGNORED_REGION", 0);
  Register("GSV_VISUALIZE_SENSOR_AREA", 0);
}

//------------------------------------------------------------
// Procedure: handleMailNodeReport()

void GridSearchViz::handleMailNodeReport(std::string str)
{
  NodeRecord record = string2NodeRecord(str);
  if (!record.valid())
    return;

  std::string name = record.getName();
  double posx = record.getX();
  double posy = record.getY();
  double altitude = record.getAltitude();

  // Logger::info("NodeReport altidute: " + doubleToStringX(altitude, 2) + ", name: " + name);

  double sensor_radius = (altitude > m_sensor_altitude_max || m_sensor_radius_fixed) ? m_sensor_radius_max : ((m_sensor_radius_max / m_sensor_altitude_max) * altitude);
  if (sensor_radius <= 0)
    return;

  sensor_radius = std::min(sensor_radius, m_sensor_radius_max);
  // Logger::info("--->Sensor radius: " + doubleToStringX(sensor_radius, 2));


  // if the drone is not in the map, add it
  if (m_map_drone_records.find(name) == m_map_drone_records.end())
    m_map_drone_records[name] = DroneRecord(name, altitude, sensor_radius);
  else
  {
    DroneRecord &drone = m_map_drone_records.at(name);
    drone.altitude = altitude;
    drone.sensor_radius = sensor_radius;
  }
  

  XYCircle sensorArea(posx, posy, sensor_radius);
  sensorArea.set_vertex_color(m_sensor_color);
  sensorArea.set_edge_color(m_sensor_color);
  // sensorArea.set_edge_size(1);
  sensorArea.set_label(name);
  sensorArea.set_label_color("off");
  // sensorArea.set_msg("sensor_area");

  sensorArea.set_color("fill", m_sensor_color);
  sensorArea.set_transparency(0.2);
  static bool registerMissionStartTime = false;

  for (auto ix : m_valid_cell_indices)
  {
    const XYSquare &cell = m_grid.getElement(ix);
    // if the the center of the cell is inside the sensor area
    if (sensorArea.containsPoint(cell.getCenterX(), cell.getCenterY()))
    {
      gridModifyCell(ix, 1); // Increment the value of the first cell variable ("x") 0 by 1
      registerMissionStartTime = true;
    }
  }

  if (registerMissionStartTime && m_missionStartTime == 0)
  {
    m_missionStartTime = MOOSTime();
    Notify("MISSION_START_TIME", m_missionStartTime);
  }

  // Only post the circle visualization if configured to do so
  sensorArea.set_active(m_visualize_sensor_area);
  Notify("VIEW_CIRCLE", sensorArea.get_spec());
  
}

//------------------------------------------------------------
// Procedure: handleMailIgnoredRegion()
void GridSearchViz::handleMailIgnoredRegion(std::string str)
{
  str = stripBlankEnds(str);

  if (strContains(str, "unreg::"))
  {
    unregisterIgnoredRegion(str.substr(7));
    return;
  }
  else if (strContains(str, "reg::"))
  {
    registerIgnoredRegion(str.substr(5));
    return;
  }

  reportRunWarning("Received Invalid region string: " + str);
  Logger::warning("Received Invalid region string: " + str);
  return;
}

void GridSearchViz::registerIgnoredRegion(std::string str)
{
  str = stripBlankEnds(str);

  std::string type = tokStringParse(str, "format");
  if (std::find(m_validRegionTypes.begin(), m_validRegionTypes.end(), type) == m_validRegionTypes.end())
  {
    reportRunWarning("Received Invalid region type: " + type);
    Logger::warning("Received Invalid region type: " + type);
    return;
  }

  PolyRegion polyRegion;
  XYPolygon region = parseStringIgnoredRegion(str, type);
  if (region.size() == 0)
  {
    reportRunWarning("Received Invalid region string: " + str);
    Logger::warning("Received Invalid region string: " + str);
    return;
  }

  std::vector<int>::iterator it = m_valid_cell_indices.begin();
  while (it != m_valid_cell_indices.end())
  {
    const XYSquare &cell = m_grid.getElement(*it);

    // if the the center of the cell is inside the sensor area
    if (region.contains(cell.getCenterX(), cell.getCenterY()))
      it = ignoreCellIndex(it, polyRegion.ignored_cell_indices);
    else
      it++;
  }

  // visualization
  std::string display_name = region.get_msg();
  const int id = m_ignoredRegions.size();
  std::string label_id = display_name + "_" + std::to_string(id);
  region.set_active(true);
  region.set_label(label_id);
  region.set_label_color("off");
  region.set_vertex_color("off");
  region.set_edge_color("off");
  region.set_color("fill", "brown");
  region.set_transparency(0.4);

  XYMarker marker(region.get_center_x(), region.get_center_y());
  marker.set_label("marker_" + label_id);
  marker.set_msg(display_name);
  Logger::info("Display name: " + display_name);
  marker.set_label_color("white");
  marker.set_type("efield");
  marker.set_width(REGION_MARKER_WIDTH);
  marker.set_active(true);
  marker.set_color("primary_color", "green");
  marker.set_color("secondary_color", "yellow");

  Notify("VIEW_POLYGON", region.get_spec());
  Notify("VIEW_MARKER", marker.get_spec());

  polyRegion.region = region;
  polyRegion.marker = marker;
  m_ignoredRegions.push_back(polyRegion);
}

void GridSearchViz::unregisterIgnoredRegion(std::string str)
{
  str = stripBlankEnds(str);
  double x = tokDoubleParse(str, "x");
  double y = tokDoubleParse(str, "y");

  PolyRegion polyRegion;

  auto it = std::find_if(m_ignoredRegions.begin(), m_ignoredRegions.end(),
                         [&](const PolyRegion &polyreg)
                         { return polyreg.region.contains(x, y); });

  if (it == m_ignoredRegions.end())
    return;

  PolyRegion &polyreg = *it;

  polyreg.region.set_active(false);
  polyreg.marker.set_active(false);

  Notify("VIEW_POLYGON", polyreg.region.get_spec());
  Notify("VIEW_MARKER", polyreg.marker.get_spec());

  registerCellIndeces(polyreg.ignored_cell_indices);

  m_ignoredRegions.erase(it);
}

XYPolygon GridSearchViz::parseStringIgnoredRegion(std::string str, std::string type) const
{
  XYPolygon region;
  if (type == "hexagon")
  {
    // Logger::info("Parsing hexagon: " + str);
    region = stringHexagon2Poly(str);
  }
  else if (type == "rectangle")
  {
    // Logger::info("Parsing rectangle: " + str);
    region = stringRectangle2Poly(str);
  }
  else
  {
    Logger::info("Parsing poly: " + str);
    region = string2Poly(str);

    // since, oval doesn't seem to register the message:
    if (type == "oval")
      region.set_msg(tokStringParse(str, "msg"));
  }
  return region;
}

XYPolygon GridSearchViz::stringHexagon2Poly(std::string str) const
{
  std::string msg = tokStringParse(str, "msg");
  double x = tokDoubleParse(str, "x");
  double y = tokDoubleParse(str, "y");
  double rad = tokDoubleParse(str, "rad");
  unsigned int pts = floor(tokDoubleParse(str, "pts"));
  double snap = tokDoubleParse(str, "snap_val");

  // Logger::info("Hexagon: x: " + doubleToStringX(x) + ", y: " + doubleToStringX(y) + ", rad: " + doubleToStringX(rad) + ", pts: " + doubleToStringX(pts) + ", snap: " + doubleToStringX(snap));

  XYPolygon region = XYPolygon(x, y, rad, pts);
  region.set_msg(msg);
  region.apply_snap(snap);
  return region;
}

XYPolygon GridSearchViz::stringRectangle2Poly(std::string str) const
{
  std::string msg = tokStringParse(str, "msg");
  double cx = tokDoubleParse(str, "cx");
  double cy = tokDoubleParse(str, "cy");
  double width = tokDoubleParse(str, "width");
  double height = tokDoubleParse(str, "height");
  double degs = tokDoubleParse(str, "degs");

  // compute the points of the rectangle from the center, width and heigh
  XYSegList corners;
  double half_width = width / 2.0;
  double half_height = height / 2.0;
  corners.add_vertex(cx + half_width, cy + half_height); // Top-right
  corners.add_vertex(cx - half_width, cy + half_height); // Top-left
  corners.add_vertex(cx - half_width, cy - half_height); // Bottom-left
  corners.add_vertex(cx + half_width, cy - half_height); // Bottom-right

  XYPolygon region = XYPolygon(corners);
  region.rotate(degs);
  region.set_msg(msg);

  return region;
}

//------------------------------------------------------------
// Procedure: postGrid()
void GridSearchViz::postGrid()
{
  std::string spec = m_grid.get_spec();

  // By default m_grid_var_name="VIEW_GRID"
  Notify(m_grid_var_name, spec);
}

//------------------------------------------------------------
// Procedure: postGridUpdates()

void GridSearchViz::postGridUpdates()
{
  if (m_map_deltas.size() == 0)
    return;

  XYGridUpdate update(m_grid_label);

  std::map<unsigned int, double>::iterator p;
  for (p = m_map_deltas.begin(); p != m_map_deltas.end(); p++)
  {
    unsigned int ix = p->first;
    double delta = p->second;
    update.addUpdate(ix, "x", delta);
  }
  std::string msg = update.get_spec();

  m_map_deltas.clear();

  // By default m_grid_var_name="VIEW_GRID"
  Notify(m_grid_var_name + "_DELTA", msg);
}

//------------------------------------------------------------
// Procedure: buildReport()
//
//  Grid characteristics:
//        Cells: 1024
//    Cell size: 10
//
//             Initial  Min    Max    Min      Max      Cells
//    CellVar  Value    SoFar  SoFar  Limited  Limited  Written
//    -------  -------  -----  -----  -------  -------  -------
//    time           0      0      -  true     false    0
//    temp          70      -      -  false    false    172
//    confid.        0   -100    100  true     true     43
//
//  Reports Sent: 534
//  Report  Freq: 0.8

bool GridSearchViz::buildReport()
{
  unsigned int grid_cells = m_grid.size();
  double cell_sizex = 0;
  double cell_sizey = 0;
  if (grid_cells > 0)
  {
    cell_sizex = m_grid.getElement(0).getLengthX();
    cell_sizey = m_grid.getElement(0).getLengthY();
  }

  unsigned int ignored_cells = m_grid.size() - m_valid_cell_indices.size();
  m_msgs << "Grid characteristics: " << std::endl;
  m_msgs << "        Cells: " << m_grid.size() << std::endl;
  m_msgs << "    Cell size: " << doubleToStringX(cell_sizex) << "x" << doubleToStringX(cell_sizey, 4) << std::endl;
  m_msgs << "  Valid cells: " << m_valid_cell_indices.size() << std::endl;
  m_msgs << "Ignored cells: " << ignored_cells << std::endl
         << std::endl;

  ACTable actab(6, 2);
  actab.setColumnJustify(1, "right");
  actab.setColumnJustify(2, "right");
  actab.setColumnJustify(3, "right");
  actab << "        | Initial | Min   | Max   | Min     | Max     ";
  actab << "CellVar | Value   | SoFar | SoFar | Limited | Limited ";
  actab.addHeaderLines();

  unsigned int i, cell_var_cnt = m_grid.getCellVarCnt();
  for (i = 0; i < cell_var_cnt; i++)
  {
    std::string cell_var = m_grid.getVar(i);
    std::string init_val = doubleToStringX(m_grid.getInitVal(i), 5);
    std::string cell_min_sofar = doubleToStringX(m_grid.getMin(i), 5);
    std::string cell_max_sofar = doubleToStringX(m_grid.getMax(i), 5);
    bool cell_min_limited = m_grid.cellVarMinLimited(i);
    bool cell_max_limited = m_grid.cellVarMinLimited(i);
    std::string cell_min_limit = "-";
    std::string cell_max_limit = "-";
    if (cell_min_limited)
      cell_min_limit = doubleToStringX(m_grid.getMinLimit(i), 5);
    if (cell_max_limited)
      cell_max_limit = doubleToStringX(m_grid.getMaxLimit(i), 5);
    actab << cell_var << init_val << cell_min_sofar << cell_max_sofar << cell_min_limit << cell_max_limit;
  }
  m_msgs << actab.getFormattedString();

  m_msgs << "\n\nSensor data " << std::endl;
  m_msgs << "---------------------------------" << std::endl;
  m_msgs << "       sensor_radius : " << doubleToStringX(m_sensor_radius_max, 1) << std::endl;
  m_msgs << "       sensor_color  : " << m_sensor_color << std::endl;
  m_msgs << " sensor_altitude_max : " << doubleToStringX(m_sensor_altitude_max, 1) << std::endl;
  m_msgs << " sensor_radius_fixed : " << boolToString(m_sensor_radius_fixed) << std::endl;
  m_msgs << "     viz_sensor_area : " << boolToString(m_visualize_sensor_area) << std::endl;
  m_msgs << std::endl;

  m_msgs << "Sensor Radius" << std::endl;
  m_msgs << "---------------------------------" << std::endl;
  ACTable actab2(4, 1);
  actab2.setColumnJustify(0, "left");
  actab2.setColumnJustify(1, "center");
  actab2.setColumnJustify(2, "center");
  actab2.setColumnJustify(3, "center");
  actab2 << "Vehicle | current | max | altitude";
  actab2.addHeaderLines();
  for (const auto &[drone, data] : m_map_drone_records)
  {
    actab2 << drone << doubleToStringX(data.sensor_radius, 3) << m_sensor_radius_max << doubleToStringX(data.altitude, 2);
  }
  m_msgs << actab2.getFormattedString();

  m_msgs << "\n\nCoverage statistics " << std::endl;
  m_msgs << "---------------------------------" << std::endl;
  m_msgs << "   Mission started: " << boolToString(m_missionStartTime != 0) << std::endl;
  if (m_missionStartTime != 0)
    m_msgs << "Mission Start Time: " << doubleToStringX(m_missionStartTime, 2) << std::endl;
  m_msgs << "       Coverage % : " << doubleToStringX(m_map_coverage_statistics["coverage_%"], 2) << std::endl;
  m_msgs << "        Decay time: " << doubleToStringX(m_grid_cell_decay_time, 2) << " s" << std::endl;
  m_msgs << std::endl;

  ACTable actab3(2, 2);
  actab3.setColumnJustify(0, "left");
  actab3.setColumnJustify(1, "right");
  actab3 << "Coverage % | Time";
  actab3.addHeaderLines();
  for (const auto &[key, value] : m_map_coverage_statistics)
  {
    // Logger::info("Key: " + key + " Value: " + doubleToStringX(value, 2));

    if ((key.find("coverage_") == 0 && key != "coverage_%"))
    {
      // extract the perentage from key
      std::string percentage = key.substr(9, key.length() - 9);
      actab3 << percentage << doubleToStringX(value, 2);
      Notify("COVERAGE_TIME_" + percentage, value);
    }
  }
  m_msgs << actab3.getFormattedString();

  return (true);
}

//------------------------------------------------------------
// Procedure: gridSetCell()
void GridSearchViz::gridSetCell(const int ix, const double val)
{
  const double curr = m_grid.getVal(ix, 0);
  if (curr == val)
    return;

  // m_grid.setVal(ix, val, 0);

  double delta = val - curr;
  // m_map_deltas[ix] = m_map_deltas[ix] + delta; // std::max(std::min(delta, m_grid.getMaxLimit(0)), m_grid.getMinLimit(0));

  gridModifyCell(ix, delta);
}
//------------------------------------------------------------
// Procedure: gridIncrementCell()
void GridSearchViz::gridModifyCell(const int ix, const double val)
{
  m_map_deltas[ix] = m_map_deltas[ix] + val;
  m_grid.incVal(ix, val, 0);
}

//------------------------------------------------------------
// Procedure: ignoreCellIndex()
std::vector<int>::iterator GridSearchViz::ignoreCellIndex(std::vector<int>::iterator it, std::vector<int> &cell_indices)
{
  if (it == m_valid_cell_indices.end())
    return it;

  gridSetCell(*it, m_grid.getMaxLimit(0)); // Increment the value of the first cell variable ("x") 0 by 1
  cell_indices.push_back(*it);

  return m_valid_cell_indices.erase(it);
}
//------------------------------------------------------------
// Procedure: ignoreCellIndex()
void GridSearchViz::registerCellIndeces(std::vector<int> &cell_indices)
{

  for (const int &ix : cell_indices)
  {
    gridSetCell(ix, m_grid.getMinLimit(0)); // Increment the value of the first cell variable ("x") 0 by 1
    m_valid_cell_indices.push_back(ix);
  }
  cell_indices.clear();
}

//------------------------------------------------------------
// Procedure: calculateCoverageStatistics()
void GridSearchViz::calculateCoverageStatistics()
{
  if (m_missionStartTime == 0)
    return;

  static double decay_time = m_grid_cell_decay_time;

  const double MoosNow = MOOSTime();
  const double time_elapsed = MoosNow - m_missionStartTime;

  bool should_decay = decay_time > 0 && time_elapsed > decay_time;

  // Calculate the coverage statistics
  double total_cells = m_grid.size();
  double ignored_cells = m_grid.size() - m_valid_cell_indices.size();
  double covered_cells = ignored_cells;

  for (auto ix : m_valid_cell_indices)
  {
    auto value = m_grid.getVal(ix, 0);

    if (should_decay)
    {
      value--;
    }

    if (value > 0)
    {
      covered_cells++;
    }
    gridSetCell(ix, value);
  }

  if (should_decay)
    decay_time += m_grid_cell_decay_time;

  double coverage_percentage = (covered_cells / total_cells) * 100;
  m_map_coverage_statistics["coverage_%"] = coverage_percentage;

  Notify("COVERAGE_PERCENTAGE", coverage_percentage);

  // Calculate the time for 40, 60, 90+ coverage

  m_map_coverage_statistics["time_elapsed"] = time_elapsed;

  if (coverage_percentage >= 90 && m_map_coverage_statistics.find("coverage_90%") == m_map_coverage_statistics.end())
    m_map_coverage_statistics["coverage_90%"] = time_elapsed;
  else if (coverage_percentage >= 60 && m_map_coverage_statistics.find("coverage_60%") == m_map_coverage_statistics.end())
    m_map_coverage_statistics["coverage_60%"] = time_elapsed;
  else if (coverage_percentage >= 40 && m_map_coverage_statistics.find("coverage_40%") == m_map_coverage_statistics.end())
    m_map_coverage_statistics["coverage_40%"] = time_elapsed;
  else if (coverage_percentage >= 20 && m_map_coverage_statistics.find("coverage_20%") == m_map_coverage_statistics.end())
    m_map_coverage_statistics["coverage_20%"] = time_elapsed;
  else if (coverage_percentage >= 10 && m_map_coverage_statistics.find("coverage_10%") == m_map_coverage_statistics.end())
    m_map_coverage_statistics["coverage_10%"] = time_elapsed;
}