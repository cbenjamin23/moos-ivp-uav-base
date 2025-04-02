/*****************************************************************/
/*    NAME: Steve Nomeny                                         */
/*    ORGN: NTNU, Trondheim                       */
/*    FILE: GridSearchPlanner.cpp                                       */
/*    DATE: Feb 2025                                            */
/*****************************************************************/

#include <iterator>
#include "GridSearchPlanner.h"
#include "MBUtils.h"
#include "NodeRecordUtils.h"
#include "XYGridUpdate.h"
#include "ACTable.h"
#include "XYFormatUtilsPoly.h"
#include "XYFormatUtilsConvexGrid.h"

#include <numeric>
#include <algorithm>

#include "common.h"
#include "Logger.h"
#include "TMSTCVisualization.h"
//---------------------------------------------------------
// Constructor()

GridSearchPlanner::GridSearchPlanner()
{
  m_sensor_radius = 10;
  m_region_grid_size_ratio = 0.5;
  m_do_plan_paths = false;
  m_is_paths_calculated = false;

  m_visualize_planner_grids = false;
  m_visualize_planner_paths = false;
  m_map_print_version = 0; // 0=off, 1=init, 2=cover, 3=direction

  m_start_point_closest = false;

  m_path_publish_variable = "SURVEY_UPDATE";

  m_missionEnabled = false;
  m_isRunningMoosPid = false;

  // Configure the TMSTC* algorithm
  TMSTCStarConfig config;
  config.allocate_method = "MSTC";
  config.mst_shape = "DINIC"; // MSTC and DINIC are what constitutes TMSTC*
  config.robot_num = m_map_drone_records.size();
  config.cover_and_return = false;                         // return to start position after cover
  config.vehicle_params.omega_rad = 0.8;                   // rad/s (angular velocity)
  config.vehicle_params.acc = 1.2;                         // m/s^2 (acceleration)
  config.vehicle_params.vmax = 18;                         // m/s (max velocity)
  config.vehicle_params.phi_max_rad = 45 * (M_PI / 180.0); // rad (maximum banking angle)
  config.vehicle_params.cellSize_m = 30;                   // meters (grid cell size)

  m_tmstc_star_ptr = std::move(std::make_unique<TMSTCStar>(config));
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool GridSearchPlanner::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  bool handled = false;
  for (p = NewMail.begin(); p != NewMail.end(); p++)
  {
    CMOOSMsg &msg = *p;

    std::string key = msg.GetKey();
    std::string sval = msg.GetString();
    double dval = msg.GetDouble();
    // double mtime = msg.GetTime();
    // bool   mdbl  = msg.IsDouble();
    // bool   mstr  = msg.IsString();
    // string msrc  = msg.GetSource();
    // std::string community = msg.GetCommunity();

    if ((key == "NODE_REPORT") || (key == "NODE_REPORT_LOCAL"))
      handled = handleMailNodeReport(sval);
    else if (key == "IGNORED_REGION_ALERT")
      handled = handleMailIgnoredRegionAlert(sval);
    else if (key == "DO_PLAN_PATHS")
      handled = setBooleanOnString(m_do_plan_paths, sval);
    else if (key == "GSP_VISUALIZE_PLANNER_GRIDS")
      handled = setBooleanOnString(m_visualize_planner_grids, sval);
    else if (key == "GSP_VISUALIZE_PLANNER_PATHS")
      handled = setBooleanOnString(m_visualize_planner_paths, sval);
    else if (key == "GSP_MAP_PRINT")
    {
      m_map_print_version = static_cast<int>(dval);
      handled = true;
      // Logger::info("OnNewMail: Received map print version: " + std::to_string(m_map_print_version));
    }
    else if (key == "GSP_START_POINT_CLOSEST")
      handled = setBooleanOnString(m_start_point_closest, sval);
    else if (key == "XENABLE_MISSION")
      handled = setBooleanOnString(m_missionEnabled, sval);
    else if (key == "VIEW_GRID")
      handled = handleMailViewGrid(sval);

    if (!handled)
    {
      reportRunWarning("Unhandled mail: " + key);
    }
  }

  return (true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool GridSearchPlanner::OnConnectToServer()
{
  registerVariables();
  return (true);
}

//---------------------------------------------------------
// Procedure: Iterate()

bool GridSearchPlanner::Iterate()
{
  AppCastingMOOSApp::Iterate();

  if (m_do_plan_paths)
  {
    doPlanPaths();
    notifyCalculatedPathsAndExecute(m_missionEnabled);
    m_do_plan_paths = false;
  }

  postCalculatedPaths(m_visualize_planner_paths);
  postTMSTCGrids(m_visualize_planner_grids);

  AppCastingMOOSApp::PostReport();
  return (true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()

bool GridSearchPlanner::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  CMOOSApp::OnStartUp();

  std::string poly_region;

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
      if (param == "search_region")
      {
        poly_region = value;
        handled = true;
      }
      else if (param == "sensor_radius")
        handled = setDoubleOnString(m_sensor_radius, value);
      else if (param == "region_grid_size_ratio")
        handled = setDoubleOnString(m_region_grid_size_ratio, value);
      else if (param == "visualize_planner_grids")
        handled = setBooleanOnString(m_visualize_planner_grids, value);
      else if (param == "visualize_planner_paths")
        handled = setBooleanOnString(m_visualize_planner_paths, value);
      else if (param == "map_print_version")
        handled = setIntOnString(m_map_print_version, value);
      else if (param == "start_point_closest")
        handled = setBooleanOnString(m_start_point_closest, value);
      else if (param == "is_running_moos_pid")
        handled = setBooleanOnString(m_isRunningMoosPid, value);
      else if (param == "path_publish_variable")
      {
        m_path_publish_variable = value;
        handled = true;
      }
      else if (param == "tmstc_star_config_vmax")
      {
        double value_double;
        if (setDoubleOnString(value_double, value))
        {
          m_tmstc_star_ptr->getConfig().vehicle_params.vmax = value_double;
          handled = true;
        }
      }
      else if (param == "tmstc_star_config_phi_max_rad")
      {
        double deg;
        if (setDoubleOnString(deg, value))
        {
          m_tmstc_star_ptr->getConfig().vehicle_params.phi_max_rad = deg * (M_PI / 180.0);
          handled = true;
        }
      }

      if (!handled)
        reportUnhandledConfigWarning(orig);
    }
  }

  if (poly_region == "")
    reportConfigWarning("No search polyregion defined.");

  XYPolygon polygon = string2Poly(poly_region);
  if (polygon.size() == 0)
  {
    Logger::warning("Failed to generate polyregion defined.");
    reportConfigWarning("Failed to generate polyregion defined.");
    return false;
  }

  m_tmstc_grid_converter.setSearchRegion(polygon);
  m_tmstc_grid_converter.setSensorRadius(m_sensor_radius * m_region_grid_size_ratio);

  m_tmstc_star_ptr->getConfig().vehicle_params.cellSize_m = m_sensor_radius * m_region_grid_size_ratio * MOOSDIST2METERS;

  convertGridToTMSTC();
  postTMSTCGrids();

  registerVariables();
  return (true);
}

//------------------------------------------------------------
// Procedure: registerVariables()

void GridSearchPlanner::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NODE_REPORT_LOCAL", 0);
  Register("NODE_REPORT", 0);

  Register("GSP_VISUALIZE_PLANNER_GRIDS", 0);
  Register("GSP_VISUALIZE_PLANNER_PATHS", 0);

  Register("IGNORED_REGION_ALERT", 0);
  Register("GSP_MAP_PRINT", 0);

  Register("DO_PLAN_PATHS", 0);
  Register("GSP_START_POINT_CLOSEST", 0);
  Register("XENABLE_MISSION", 0);

  Register("VIEW_GRID", 0);
}

void GridSearchPlanner::doPlanPaths()
{

  if (m_tmstc_star_ptr == nullptr)
  {
    Logger::error("doPlanPaths: TMSTC* instance is null.");
    reportRunWarning("Failed to calculate paths. TMSTC* instance is not initialized.");
    m_is_paths_calculated = false;
    return;
  }

  m_tmstc_grid_converter.transformGrid();

  Mat spanningMap = m_tmstc_grid_converter.getSpanningGrid();
  std::vector<int> robot_region_indeces = m_tmstc_grid_converter.getUniqueVehicleRegionIndices();

  if (robot_region_indeces.size() != m_map_drone_records.size())
  {
    std::string msg = "Number of robot region indeces (" + std::to_string(robot_region_indeces.size()) + ") does not match number of drones (" + std::to_string(m_map_drone_records.size()) + ").";
    m_generate_warnings.push_back(msg);
    Logger::error("doPlanPaths:" + msg);
    reportRunWarning(msg);
    postCalculatedPaths(false);
    m_is_paths_calculated = false;
    return;
  }

  // Create the TMSTC* instance and calculate paths
  m_tmstc_star_ptr->reconfigureMapRobot(spanningMap, robot_region_indeces);

  Mat paths_robot_indx;
  try
  {
    m_tmstc_star_ptr->eliminateIslands();
    reportEvent("Calculating paths...");
    Logger::info("doPlanPaths: Calculating paths...");
    paths_robot_indx = m_tmstc_star_ptr->calculateRegionIndxPaths();
    Logger::info("doPlanPaths: Calculating paths... Region Index paths calculated.");
    m_is_paths_calculated = true;
  }
  catch (const std::exception &e)
  {

    std::string msg = "Failed to calculate paths. Exception: " + std::string(e.what());
    m_generate_warnings.push_back(msg);

    Logger::error("doPlanPaths:" + msg);
    reportRunWarning(msg);
    postCalculatedPaths(false);
    m_is_paths_calculated = false;
    return;
  }

  assignPathsToVehicles(paths_robot_indx);

  Logger::info("doPlanPaths: Paths calculated.");
  reportEvent("Paths calculated.");

  clearAllGenerateWarnings();
}

void GridSearchPlanner::assignPathsToVehicles(Mat paths_robot_indx)
{

  auto paths_robot_coords = m_tmstc_star_ptr->pathsIndxToRegionCoords(paths_robot_indx);

  std::set<std::string> drone_names;
  for (const auto &[drone_name, record] : m_map_drone_records)
  {
    drone_names.insert(drone_name);
  }
  for (const auto &path : paths_robot_coords)
  {
    // Convert the paths to XYSegList format
    XYSegList seglist = m_tmstc_grid_converter.regionCoords2XYSeglisMoos(path);


    // prune wpts in path that are already discovered by looking at m_grid_viz


    // find the closest drone
    XYPoint firstPoint = seglist.get_first_point();
    std::string drone = "";
    double min_dist = std::numeric_limits<double>::max();
    for (const auto drone_name : drone_names)
    {
      double dist = hypot(firstPoint.x() - m_map_drone_records[drone_name].getX(), firstPoint.y() - m_map_drone_records[drone_name].getY());
      if (dist < min_dist)
      {
        min_dist = dist;
        drone = drone_name;
      }
    }

    drone_names.erase(drone); // remove the drone from the set

    if (m_start_point_closest)
    {
      XYPoint firstPoint = seglist.get_first_point();
      XYPoint lastPoint = seglist.get_last_point();

      double dist_firstPoint = hypot(firstPoint.x() - m_map_drone_records[drone].getX(), firstPoint.y() - m_map_drone_records[drone].getY());
      double dist_lastPoint = hypot(lastPoint.x() - m_map_drone_records[drone].getX(), lastPoint.y() - m_map_drone_records[drone].getY());

      if (dist_lastPoint < dist_firstPoint)
        seglist.reverse();
    }
    m_map_drone_paths[drone] = seglist;
  }
  // Logger::info("doPlanPaths: Paths distributed to vehicles.");
}

void GridSearchPlanner::notifyCalculatedPathsAndExecute(bool executePath)
{
  if (!m_is_paths_calculated)
    return;

  for (const auto &[drone, records] : m_map_drone_records)
  {
    if (m_map_drone_paths.count(drone) == 0)
      continue;

    XYSegList path_seg = m_map_drone_paths[drone];

    std::string pts_str = path_seg.get_spec_pts();
    std::string spec = "points = " + pts_str;
    std::string notify_var_str = m_path_publish_variable + '_' + MOOSToUpper(drone);
    Notify(notify_var_str, spec);

    if (executePath)
    {

      if (m_isRunningMoosPid)
      {
        Notify("DO_SURVEY_" + MOOSToUpper(drone), "true"); // If running MOOS PID simulation
        Notify("DEPLOY_" + MOOSToUpper(drone), "true");
        Notify("RETURN_" + MOOSToUpper(drone), "false");
        Notify("MOOS_MANUAL_OVERRIDE_" + MOOSToUpper(drone), "false");
      }
      else
      {

        Notify("HELM_STATUS_" + MOOSToUpper(drone), "ON");
        notify_var_str = "GCS_COMMAND_" + MOOSToUpper(drone);
        Notify(notify_var_str, "SURVEY");
      }
    }
  }
}
void GridSearchPlanner::clearAllGenerateWarnings()
{
  for (const auto &warning : m_generate_warnings)
    retractRunWarning(warning);

  m_generate_warnings.clear();
}

void GridSearchPlanner::postCalculatedPaths(bool visible)
{
  if (!m_is_paths_calculated)
    return;

  for (const auto &[drone, records] : m_map_drone_records)
  {
    if (m_map_drone_paths.count(drone) == 0)
      continue;

    XYSegList path_seg = m_map_drone_paths[drone];

    // Path
    constexpr double width = 10;
    std::string color = records.getColor();
    path_seg.set_edge_color(color);
    path_seg.set_vertex_size(3);
    path_seg.set_label(drone + "_path");
    path_seg.set_label_color("white");
    path_seg.set_transparency(1);
    path_seg.set_active(visible);
    path_seg.set_edge_size(width);

    std::string path_str = path_seg.get_spec();
    Notify("VIEW_SEGLIST", path_str);

    // Start marker
    XYMarker start_marker(path_seg.get_point(0).x(), path_seg.get_point(0).y());
    start_marker.set_type("circle");
    start_marker.set_width(width);
    start_marker.set_label(drone + "_start");
    start_marker.set_label_color("off");
    start_marker.set_color("primary_color", "green");
    start_marker.set_edge_color(color);
    start_marker.set_active(visible);
    Notify("VIEW_MARKER", start_marker.get_spec());

    // End marker
    XYMarker end_marker(path_seg.get_point(path_seg.size() - 1).x(), path_seg.get_point(path_seg.size() - 1).y());
    end_marker.set_type("circle");
    end_marker.set_width(width);
    end_marker.set_label(drone + "_end");
    end_marker.set_label_color("off");
    end_marker.set_color("primary_color", "red");
    end_marker.set_edge_color(color);
    end_marker.set_active(visible);
    Notify("VIEW_MARKER", end_marker.get_spec());
  }
}

void GridSearchPlanner::updateTMSTCVehiclePositions()
{
  std::vector<XYPoint> vpos;
  for (auto [name, record] : m_map_drone_records)
  {
    double posx = record.getX();
    double posy = record.getY();
    vpos.emplace_back(posx, posy);
  }

  m_tmstc_grid_converter.setVehiclePositions(vpos);
}
void GridSearchPlanner::updateTMSTCIgnoredRegions()
{
  std::vector<XYPolygon> iregs;
  for (auto [name, poly] : m_map_ignored_regions_poly)
    iregs.push_back(poly);

  m_tmstc_grid_converter.setIgnoreRegions(iregs);
}
//------------------------------------------------------------
// Procedure: handleMailNodeReport()

bool GridSearchPlanner::handleMailNodeReport(std::string str)
{
  NodeRecord record = string2NodeRecord(str);
  if (!record.valid())
    return false;

  std::string name = record.getName();
  m_map_drone_records[name] = record;

  updateTMSTCVehiclePositions();
  // Logger::info("NodeReport position: (" + doubleToStringX(posx, 2) + ", " + doubleToStringX(posy, 2) + "), name: " + name);
  return true;
}

bool GridSearchPlanner::handleMailViewGrid(std::string str)
{
  auto grid = string2ConvexGrid(str);

  if (!grid.valid())
  {

    reportRunWarning("Received Invalid VIEW_GRID");
    Logger::warning("Received Invalid VIEW_GRID: " + str);
    return false;
  }

  m_grid_viz = grid;
  return true;
}

//------------------------------------------------------------
// Procedure: handleMailIgnoredRegion()

/// @brief Handles alerts for ignored regions
/// @param str The input string containing region information\
///            Format: "reg:: name" \
///                    "unreg:: ignored_region_str"
bool GridSearchPlanner::handleMailIgnoredRegionAlert(std::string str)
{
  str = stripBlankEnds(str);

  if (strContains(str, "unreg::"))
  {
    unregisterIgnoredRegion(str.substr(7));
    m_do_plan_paths = true;
    return true;
  }
  else if (strContains(str, "reg::"))
  {
    registerIgnoredRegion(str.substr(5));
    m_do_plan_paths = true;
    return true;
  }

  reportRunWarning("Received Invalid region string: " + str);
  Logger::warning("Received Invalid region string: " + str);
  return false;
}

void GridSearchPlanner::registerIgnoredRegion(std::string str)
{
  str = stripBlankEnds(str);

  IgnoredRegion ignoredRegion = stringToIgnoredRegion(str);
  if (!ignoredRegion.isValid())
  {
    reportRunWarning("Bad IgnoredRegion string: " + str);
    Logger::warning("Bad IgnoredRegion string: " + str);
    return;
  }

  std::string name = ignoredRegion.getName();

  if (m_map_ignored_regions_poly.count(name) != 0)
  {
    reportRunWarning("Region name already exist: " + name);
    Logger::warning("Region name already exist: " + name);
    return;
  }

  XYPolygon region = ignoredRegion.getPoly();

  m_map_ignored_regions_poly[name] = region;

  updateTMSTCIgnoredRegions();
  // Logger::info("Registered ignored region: " + name);
}

void GridSearchPlanner::unregisterIgnoredRegion(std::string name)
{
  name = stripBlankEnds(name);

  if (m_map_ignored_regions_poly.count(name) == 0)
    return;

  m_map_ignored_regions_poly.erase(name);

  updateTMSTCIgnoredRegions();
  // Logger::info("Unregistered ignored region: " + name);
}

//------------------------------------------------------------
// Procedure: buildReport()

std::string mapPrintVersion2string(int version)
{
  switch (version)
  {
  case 0:
    return "Off";
  case 1:
    return "Init";
  case 2:
    return "Cover";
  case 3:
    return "Direction";
  default:
    return "Unknown";
  }
}

bool GridSearchPlanner::buildReport()
{

  m_msgs << "Grid Search Planner Configuration" << std::endl;
  m_msgs << "---------------------------------" << std::endl;
  m_msgs << "          Sensor radius: " << doubleToStringX(m_sensor_radius, 1) << std::endl;
  m_msgs << " Region grid size ratio: " << doubleToStringX(m_region_grid_size_ratio, 1) << std::endl;
  m_msgs << "       Region grid size: " << doubleToStringX(m_region_grid_size_ratio * m_sensor_radius, 1) << std::endl;
  m_msgs << "Visualize planner grids: " << boolToString(m_visualize_planner_grids) << std::endl;
  m_msgs << "Visualize planner paths: " << boolToString(m_visualize_planner_paths) << std::endl;
  m_msgs << "      Map print version: " << mapPrintVersion2string(m_map_print_version) << std::endl;
  m_msgs << " Is start point closest: " << boolToString(m_start_point_closest) << std::endl;
  m_msgs << "        Mission enabled: " << boolToString(m_missionEnabled) << std::endl;
  m_msgs << "       isRunningMoosPid: " << boolToString(m_isRunningMoosPid) << std::endl;
  m_msgs << std::endl;

  m_msgs << "TMSTC* algorithm" << std::endl;
  m_msgs << "---------------------------------" << std::endl;
  // m_msgs << "   Allocate method: " << m_tmstc_star_ptr->getConfig().allocate_method << std::endl;
  // m_msgs << "   MST shape: " << m_tmstc_star_ptr->getConfig().mst_shape << std::endl;
  // m_msgs << "   Robot num: " << m_tmstc_star_ptr->getConfig().robot_num << std::endl;
  // m_msgs << "   Cover and return: " << boolToString(m_tmstc_star_ptr->getConfig().cover_and_return) << std::endl;
  m_msgs << "   Vehicle params:" << std::endl;
  // m_msgs << "     omega_rad: " << doubleToStringX(m_tmstc_star_ptr->getConfig().vehicle_params.omega_rad, 2) << std::endl;
  // m_msgs << "     acc: " << doubleToStringX(m_tmstc_star_ptr->getConfig().vehicle_params.acc, 2) << std::endl;
  m_msgs << "     vmax: " << doubleToStringX(m_tmstc_star_ptr->getConfig().vehicle_params.vmax, 2) << std::endl;
  m_msgs << "     phi_max_rad: " << doubleToStringX(m_tmstc_star_ptr->getConfig().vehicle_params.phi_max_rad * (180.0 / M_PI), 2) << std::endl;
  m_msgs << "     cellSize_m: " << doubleToStringX(m_tmstc_star_ptr->getConfig().vehicle_params.cellSize_m, 2) << std::endl;
  m_msgs << std::endl;

  bool grids_converted = m_tmstc_grid_converter.isGridsConverted();
  m_msgs << "Drone Positions" << std::endl;
  m_msgs << "---------------------------------" << std::endl;
  ACTable actab(5, 2);
  actab.setColumnJustify(1, "center");
  actab.setColumnJustify(2, "center");
  actab.setColumnJustify(3, "center");
  actab << "Drone | - | - | Region coord | Spanning coord";
  actab << " Name | X | Y | (col , row) | (col , row)";
  actab.addHeaderLines();
  for (const auto &[drone, record] : m_map_drone_records)
  {
    double posx = record.getX();
    double posy = record.getY();
    XYPoint pos(posx, posy);
    std::string posx_str = doubleToStringX(posx, 2);
    std::string posy_str = doubleToStringX(posy, 2);

    std::string region_coord_str = "-";
    std::string spanning_coord_str = "-";

    if (grids_converted)
    {
      auto region_coord = m_tmstc_grid_converter.getVehicleRegionCoordinate(pos);
      region_coord_str = "(" + doubleToStringX(region_coord.get_vx(), 0) + ", " + doubleToStringX(region_coord.get_vy(), 0) + ")";
      auto spanning_coord = m_tmstc_grid_converter.getVehicleSpanningCoordinate(pos);
      spanning_coord_str = "(" + doubleToStringX(spanning_coord.get_vx(), 0) + ", " + doubleToStringX(spanning_coord.get_vy(), 0) + ")";
    }

    actab << drone << posx_str << posy_str << region_coord_str << spanning_coord_str;
  }

  m_msgs << actab.getFormattedString();
  m_msgs << std::endl;

  auto num_ignored_regions = m_map_ignored_regions_poly.size();
  m_msgs << "Number of ignored Regions: " << uintToString(num_ignored_regions) << std::endl;
  m_msgs << "Is path calculated: " << boolToString(m_is_paths_calculated) << std::endl;
  m_msgs << "Do plan paths: " << boolToString(m_do_plan_paths) << std::endl;
  m_msgs << std::endl;

  if (grids_converted)
  {

    m_msgs << std::endl;
    m_msgs << "TMSTC Grids" << std::endl;
    m_msgs << "---------------------------------" << std::endl;
    ACTable actab2(6, 2);
    actab2.setColumnJustify(1, "center");
    actab2.setColumnJustify(2, "center");
    actab2.setColumnJustify(3, "center");
    actab2.setColumnJustify(4, "center");
    actab2.setColumnJustify(5, "center");
    actab2 << " Grid | cell | cell |  total | free | occupied";
    actab2 << " type | cols|  rows | cells | cells | cells";
    actab2.addHeaderLines();

    Mat regionMap = m_tmstc_grid_converter.getRegionGrid();
    std::vector<std::pair<int, int>> robot_start_positions = m_tmstc_grid_converter.getUniqueVehicleRegionCoordinates();

    Mat spanningMap = m_tmstc_grid_converter.getSpanningGrid();
    // std::vector<std::pair<int, int>> robot_start_positions_spanning = m_tmstc_grid_converter.getVehicleSpanningPositions();

    auto region_width = regionMap[0].size();
    std::string region_width_str = uintToString(region_width);
    auto region_height = regionMap.size();
    std::string region_height_str = uintToString(region_height);
    auto spanning_width = spanningMap[0].size();
    std::string spanning_width_str = uintToString(spanning_width);
    auto spanning_height = spanningMap.size();
    std::string spanning_height_str = uintToString(spanning_height);

    auto region_total_cells = region_width * region_height;
    std::string region_total_cells_str = uintToString(region_total_cells);
    auto spanning_total_cells = spanning_width * spanning_height;
    std::string spanning_total_cells_str = uintToString(spanning_total_cells);

    auto region_grid_centers = m_tmstc_grid_converter.getRegionGridCenters();
    auto spanning_grid_centers = m_tmstc_grid_converter.getSpanningGridCenters();

    int region_free_cells = 0;
    int region_occupied_cells = 0;

    for (auto const pos : region_grid_centers)
    {
      int z = pos.get_vz(); // 1 is free, 0 is occupied
      region_free_cells += z;
    }
    region_occupied_cells = region_total_cells - region_free_cells;
    std::string region_free_cells_str = uintToString(region_free_cells);
    std::string region_occupied_cells_str = uintToString(region_occupied_cells);

    int spanning_free_cells = 0;
    int spanning_occupied_cells = 0;
    for (auto const pos : spanning_grid_centers)
    {
      int z = pos.get_vz(); // 1 is free, 0 is occupied
      spanning_free_cells += z;
    }
    spanning_occupied_cells = spanning_total_cells - spanning_free_cells;
    std::string spanning_free_cells_str = uintToString(spanning_free_cells);
    std::string spanning_occupied_cells_str = uintToString(spanning_occupied_cells);

    actab2 << "Region Grid " << region_width_str << region_height_str << region_total_cells_str << region_free_cells_str << region_occupied_cells_str;
    actab2 << "Spanning Grid" << spanning_width_str << spanning_height_str << spanning_total_cells_str << spanning_free_cells_str << spanning_occupied_cells_str;
    m_msgs << actab2.getFormattedString();
    m_msgs << std::endl;

    std::stringstream ss;
    if (m_is_paths_calculated)
    {
      m_msgs << std::endl;
      m_msgs << "Calcultated paths Mat Grids" << std::endl;
      m_msgs << "---------------------------------" << std::endl;

      robot_start_positions.clear();

      //
      const auto paths_indx = m_tmstc_star_ptr->getPaths();

      for (const auto &path : paths_indx)
      {
        auto start = path.front();
        robot_start_positions.push_back(m_tmstc_star_ptr->indexToRegionCoord(start));
      }

      if (m_map_print_version == 1)
        TMSTCViz::visualizeInitialMap(regionMap, robot_start_positions, ss, false);
      else if (m_map_print_version == 2)
        TMSTCViz::visualizePaths(regionMap, paths_indx, robot_start_positions, ss, false);
      else if (m_map_print_version == 3)
        TMSTCViz::visualizeDirectionalPaths(regionMap, paths_indx, robot_start_positions, ss, false);
    }
    else
    { // if paths are not calculated
      m_msgs << std::endl;
      m_msgs << "Initial Grids" << std::endl;
      m_msgs << "---------------------------------" << std::endl;
      TMSTCViz::visualizeInitialMap(regionMap, robot_start_positions, ss, false);
    }

    m_msgs << ss.str();
    m_msgs << std::endl;
  }
  else
    m_msgs << "TMSTC Grids not converted" << std::endl;

  return (true);
}

//------------------------------------------------------------
// Procedure: convertGridToTMSTC()
void GridSearchPlanner::convertGridToTMSTC()
{

  m_tmstc_grid_converter.transformGrid();
}

void GridSearchPlanner::postTMSTCGrids(bool visible)
{

  static bool prev_active = false;

  if (!visible && !prev_active) // if not active and not previously active
    return;

  if (!m_tmstc_grid_converter.isGridsConverted())
    return;

  std::vector<XYPoint> regionGridPoints = m_tmstc_grid_converter.getRegionGridCenters();
  std::vector<XYPoint> downsampledGridPoints = m_tmstc_grid_converter.getSpanningGridCenters();

  double transparancy = 0.1;

  int idx = 0;
  for (auto point : regionGridPoints)
  {
    XYCircle circle(point.x(), point.y(), m_sensor_radius * m_region_grid_size_ratio);

    circle.set_label("Sr_" + std::to_string(idx++));
    circle.set_label_color("off");
    circle.set_edge_color("yellow");
    circle.set_color("fill", "off");
    circle.set_transparency(transparancy);
    circle.set_edge_size(2);
    circle.set_vertex_size(2);
    circle.set_active(visible);

    if (point.z() == 0)
    {
      circle.set_color("fill", "yellow");
      circle.set_transparency(0.4);
    }
    // Logger::info("Region point: " + doubleToStringX(point.x(), 2) + ", " + doubleToStringX(point.y(), 2) + ", " + doubleToStringX(point.z(), 2));
    Notify("VIEW_CIRCLE", circle.get_spec());
  }

  idx = 0;
  for (auto point : downsampledGridPoints)
  {
    XYCircle circle(point.x(), point.y(), m_sensor_radius * m_region_grid_size_ratio);

    circle.set_label("Sdr_" + std::to_string(idx++));
    circle.set_label_color("off");
    circle.set_edge_color("red");
    circle.set_color("fill", "off");
    circle.set_transparency(transparancy);
    circle.set_edge_size(2);
    circle.set_vertex_size(2);
    circle.set_active(visible);

    if (point.z() == 0)
    {
      circle.set_color("fill", "red");
      circle.set_transparency(0.6);
    }

    Notify("VIEW_CIRCLE", circle.get_spec());
  }

  prev_active = visible;

  m_tmstc_grid_converter.saveSpanningGridToFile("downsampled_grid.txt");
}
