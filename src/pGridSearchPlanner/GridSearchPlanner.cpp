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
  m_coveragecellradius = m_sensor_radius * m_region_grid_size_ratio;
  m_do_plan_paths = false;
  m_do_start_voronoi_searching = false;
  m_is_paths_calculated = false;

  m_visualize_planner_grids = false;
  m_visualize_planner_paths = false;
  m_map_print_version = 0; // 0=off, 1=init, 2=cover, 3=direction

  m_start_point_closest = false;
  m_tmstc_star_point_filtering = false;

  m_path_publish_variable = "SURVEY_UPDATE";

  m_missionEnabled = false;
  m_isRunningMoosPid = false;

  m_planner_mode = Planner::PlannerMode::VORONOI_SEARCH;

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
    else if (key == "XENABLE_MISSION" )
    {
      handled = setBooleanOnString(m_missionEnabled, sval);
      if(m_missionEnabled)
        raisePlannerFlag();
    }
    else if (key == "VIEW_GRID")
      handled = handleMailViewGrid(sval);
    else if (key == "VIEW_GRID_DELTA")
      handled = handleMailViewGridUpdate(sval);

    else if (key == "CHANGE_PLANNER_MODEX")
    {
      MOOSToUpper(sval);
      try
      {
        m_planner_mode = Planner::stringToMode(sval);
        handled = true;
      }
      catch (std::exception &e)
      {
        std::string msg = "Failed to set planner mode. Exception: " + std::string(e.what());
        m_generate_warnings.push_back(msg);
        Logger::error("OnNewMail:" + msg);
        reportRunWarning(msg);
        handled = false;
      }
    }

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
  else if (m_do_start_voronoi_searching)
  {
    notifyVoronoiSearching();
    m_do_start_voronoi_searching = false;
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
      else if (param == "tmstc_star_point_filtering")
        handled = setBooleanOnString(m_tmstc_star_point_filtering, value);
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
      else if (param == "planner_mode")
      {
        std::string mode_str = toupper(value);

        try
        {
          m_planner_mode = Planner::stringToMode(mode_str);
          handled = true;
        }
        catch (std::exception &e)
        {
          std::string msg = "Failed to set planner mode. Exception: " + std::string(e.what());
          m_generate_warnings.push_back(msg);
          Logger::error("OnStartUp:" + msg);
          reportRunWarning(msg);
          handled = false;
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

  m_coveragecellradius = m_sensor_radius * m_region_grid_size_ratio;

  m_tmstc_grid_converter.setSearchRegion(polygon);
  m_tmstc_grid_converter.setSensorRadius(m_coveragecellradius);

  m_tmstc_star_ptr->getConfig().vehicle_params.cellSize_m = 2 * m_coveragecellradius * MOOSDIST2METERS;

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
  Register("VIEW_GRID_DELTA", 0);

  Register("CHANGE_PLANNER_MODEX", 0);
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

  m_tmstc_star_ptr->getConfig().is_point_filtered_func = [this](int point_idx)
  {
    return is_pathIdx_filtered(point_idx);
  };

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

bool GridSearchPlanner::is_pathIdx_filtered(int idx)
{

  std::pair<int, int> region_coord = m_tmstc_star_ptr->indexToRegionCoord(idx);
  XYPoint xy_point = m_tmstc_grid_converter.regionCoord2XYPointMoos(region_coord.first, region_coord.second);
  if (!xy_point.valid())
  {
    Logger::error("is_pathIdx_filtered: Invalid XYPoint for index " + std::to_string(idx));
    return true; // Filter out invalid points
  }

  // Check if the point is inside any ignored region
  // Get grid properties
  static const XYPolygon searchRegion = m_tmstc_grid_converter.getSearchRegion();
  static const XYSquare bbox = m_grid_viz.getSBound();
  static const double bbox_min_x = bbox.get_min_x();
  static const double bbox_min_y = bbox.get_min_y();
  static const double grid_cell_size = m_grid_viz.getCellSize();

  double x = xy_point.get_vx();
  double y = xy_point.get_vy();

  // Check if waypoint is in an ignored region or outside the search region
  if (!searchRegion.contains(x, y))
  {
    // Logger::info("Pruning waypoint at: (" + doubleToStringX(x, 2) +
    //             ", " + doubleToStringX(y, 2) + ") - Outside search region");
    return true; // Filter out invalid points
  }

  // Check if waypoint is in an ignored region
  for (const auto &[_, region] : m_map_ignored_regions_poly)
  {
    if (region.contains(x, y))
    {
      // Logger::info("Pruning waypoint at: (" + doubleToStringX(x, 2) +
      //             ", " + doubleToStringX(y, 2) + ") - In ignored region");
      return true; // Filter out invalid points
    }
  }

  if (!m_tmstc_star_point_filtering)
    return false; // No filtering needed

  // Define the square area around the waypoint (sensor coverage)
  double x_min = x - m_coveragecellradius;
  double x_max = x + m_coveragecellradius;
  double y_min = y - m_coveragecellradius;
  double y_max = y + m_coveragecellradius;

  // Convert world coordinates to grid cell indices
  int col_start = static_cast<int>(std::floor((x_min - bbox_min_x) / grid_cell_size));
  int col_end = static_cast<int>(std::floor((x_max - bbox_min_x) / grid_cell_size));
  int row_start = static_cast<int>(std::floor((y_min - bbox_min_y) / grid_cell_size));
  int row_end = static_cast<int>(std::floor((y_max - bbox_min_y) / grid_cell_size));

  // Clamp to valid grid indices
  col_start = std::max(0, col_start);
  row_start = std::max(0, row_start);
  col_end = std::min(col_end, static_cast<int>(bbox.getLengthX() / grid_cell_size) - 1);
  row_end = std::min(row_end, static_cast<int>(bbox.getLengthY() / grid_cell_size) - 1);

  // Check if area is completely outside the grid
  if (col_start > col_end || row_start > row_end)
  {
    return false; // No cells to check, dont filter
  }

  // Count discovered cells in this area
  int total_cells = 0;
  int discovered_cells = 0;

  for (int row = row_start; row <= row_end; row++)
  {
    for (int col = col_start; col <= col_end; col++)
    {
      // Calculate the center of this cell in world coordinates
      double cell_center_x = bbox_min_x + (col + 0.5) * grid_cell_size;
      double cell_center_y = bbox_min_y + (row + 0.5) * grid_cell_size;

      // Only count cells whose centers are in the waypoint's coverage area
      if (cell_center_x >= x_min && cell_center_x <= x_max &&
          cell_center_y >= y_min && cell_center_y <= y_max)
      {

        total_cells++;

        // Find this cell in the grid
        cellP cell_center(cell_center_x, cell_center_y);
        if (m_map_grid_cellCenter_idxs.count(cell_center) == 0)
          continue;

        unsigned int ix = m_map_grid_cellCenter_idxs[cell_center];
        double cell_val = m_grid_viz.getVal(ix, 0);
        if (cell_val > 0)
          discovered_cells++;
      }
    }
  }

  // If more than 50% of cells in this area are already discovered, remove the waypoint
  if (total_cells > 0 && ((double)discovered_cells / total_cells) > 0.5)
  {

    Logger::info("Pruning waypoint at: (" + doubleToStringX(x, 2) +
                 ", " + doubleToStringX(y, 2) + ") - " +
                 uintToString(discovered_cells) + "/" + uintToString(total_cells) +
                 " (" + doubleToStringX(discovered_cells / double(total_cells), 2) +
                 ") cells already discovered");

    return true; // Filter out invalid points
  }

  return false;
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
    XYSegList seglist = m_tmstc_grid_converter.regionCoords2XYSeglistMoos(path);

    // Prune waypoints in path that are already discovered by looking at m_grid_viz
    // seglist = pruneDiscoveredWaypoints(seglist);

    // Find the closest drone
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
        Notify("DO_SURVEY_" + MOOSToUpper(drone), boolToString(m_missionEnabled)); // If running MOOS PID simulation
        Notify("DEPLOY_" + MOOSToUpper(drone), "false");
        Notify("LOITER_" + MOOSToUpper(drone), "false");
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

void GridSearchPlanner::notifyVoronoiSearching()
{

  if (m_isRunningMoosPid)
  {
    Notify("DO_SURVEY_ALL", "false"); // If running MOOS PID simulation
    Notify("LOITER_ALL", "false");
    Notify("DEPLOY_ALL", boolToString(m_missionEnabled));
    Notify("RETURN_ALL", "false");
    Notify("MOOS_MANUAL_OVERRIDE_ALL", "false");
  }
  else
  {
    Notify("HELM_STATUS_ALL", "ON");
    std::string notify_var_str = "GCS_COMMAND_ALL";
    Notify(notify_var_str, "DO_VORONOI");
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
  str = stripBlankEnds(str);
  auto grid = string2ConvexGrid(str);
  if (!grid.valid())
  {
    reportRunWarning("Received invalid grid: " + str);
    Logger::warning("Received invalid grid: " + str);
    return false;
  }
  m_grid_viz = grid;

  for (unsigned int ix = 0; ix < m_grid_viz.size(); ix++)
  {
    XYSquare cell = m_grid_viz.getElement(ix);
    cellP cell_center(cell.getCenterX(), cell.getCenterY());
    m_map_grid_cellCenter_idxs[cell_center] = ix;

    // Logger::info("Grid cell index: " + doubleToStringX(ix, 2) + " at center: (" + doubleToStringX(cell_center.first, 2) + ", " + doubleToStringX(cell_center.second, 2) + ")");
  }

  return true;
}

bool GridSearchPlanner::handleMailViewGridUpdate(std::string str)
{

  m_grid_viz.processDelta(str);
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
    raisePlannerFlag();
    return true;
  }
  else if (strContains(str, "reg::"))
  {
    registerIgnoredRegion(str.substr(5));
    raisePlannerFlag();
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
  m_msgs << "       isRunningMoosPid: " << boolToString(m_isRunningMoosPid) << std::endl;
  m_msgs << "        Mission enabled: " << boolToString(m_missionEnabled) << std::endl;
  m_msgs << std::endl;

  if (m_planner_mode == Planner::PlannerMode::TMSTC_STAR)
  {

    m_msgs << "TMSTC* algorithm" << std::endl;
    m_msgs << "---------------------------------" << std::endl;
    m_msgs << "   TMSTC* point filtering: " << boolToString(m_tmstc_star_point_filtering) << std::endl;
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
  }
  else if (m_planner_mode == Planner::PlannerMode::VORONOI_SEARCH)
  {
    m_msgs << "Voronoi Search algorithm" << std::endl;
    m_msgs << "---------------------------------" << std::endl;
  }

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

  if (grids_converted && m_planner_mode == Planner::PlannerMode::TMSTC_STAR)
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
    XYCircle circle(point.x(), point.y(), m_coveragecellradius);

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
    XYCircle circle(point.x(), point.y(), m_coveragecellradius);

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

//------------------------------------------------------------
// Procedure: getCellsInSquare()
// Finds all cell centers that lie within a specified square area
std::vector<XYPoint> GridSearchPlanner::getCellsInSquare(double center_x, double center_y,
                                                         double square_side_length,
                                                         const XYConvexGrid &grid) const
{
  std::vector<XYPoint> covered_cell_centers;

  // Calculate bounds of the square
  double half_side = square_side_length / 2.0;
  double x_min = center_x - half_side;
  double x_max = center_x + half_side;
  double y_min = center_y - half_side;
  double y_max = center_y + half_side;

  // Check each cell in the grid
  for (unsigned int ix = 0; ix < grid.size(); ix++)
  {
    const XYSquare &cell = grid.getElement(ix);
    double cell_center_x = cell.getCenterX();
    double cell_center_y = cell.getCenterY();

    // Check if this cell center is within our square
    if (cell_center_x >= x_min && cell_center_x <= x_max &&
        cell_center_y >= y_min && cell_center_y <= y_max)
    {
      covered_cell_centers.emplace_back(cell_center_x, cell_center_y, ix);
    }
  }

  return covered_cell_centers;
}

//------------------------------------------------------------
// Procedure: pruneDiscoveredWaypoints()
// Prunes waypoints from a path if they are already largely discovered
XYSegList GridSearchPlanner::pruneDiscoveredWaypoints(const XYSegList &original_path)
{

  if (!m_grid_viz.valid() || original_path.size() == 0)
  {
    return original_path; // Return original path if grid is invalid or path is empty
  }

  XYSegList pruned_path = original_path;

  // Get grid properties
  XYPolygon searchRegion = m_tmstc_grid_converter.getSearchRegion();
  XYSquare bbox = m_grid_viz.getSBound();
  double bbox_min_x = bbox.get_min_x();
  double bbox_min_y = bbox.get_min_y();
  double grid_cell_size = m_grid_viz.getCellSize();

  // Process from the end of the path towards the beginning
  // This allows us to safely remove points without affecting our iteration
  for (int i = pruned_path.size() - 1; i >= 0; i--)
  {
    double x = pruned_path.get_vx(i);
    double y = pruned_path.get_vy(i);

    // Check if waypoint is in an ignored region or outside the search region
    bool is_wpt_ignored = false;
    if (!searchRegion.contains(x, y))
    {
      is_wpt_ignored = true;
      Logger::info("Pruning waypoint at: (" + doubleToStringX(x, 2) +
                   ", " + doubleToStringX(y, 2) + ") - Outside search region");
    }

    for (const auto &[_, region] : m_map_ignored_regions_poly)
    {
      if (region.contains(x, y))
      {
        is_wpt_ignored = true;
        Logger::info("Pruning waypoint at: (" + doubleToStringX(x, 2) +
                     ", " + doubleToStringX(y, 2) + ") - In ignored region");
        break;
      }
    }

    if (is_wpt_ignored)
    {
      pruned_path.delete_vertex(i);
      continue;
    }

    // Define the square area around the waypoint (sensor coverage)
    double x_min = x - m_coveragecellradius;
    double x_max = x + m_coveragecellradius;
    double y_min = y - m_coveragecellradius;
    double y_max = y + m_coveragecellradius;

    // Convert world coordinates to grid cell indices
    int col_start = static_cast<int>(std::floor((x_min - bbox_min_x) / grid_cell_size));
    int col_end = static_cast<int>(std::floor((x_max - bbox_min_x) / grid_cell_size));
    int row_start = static_cast<int>(std::floor((y_min - bbox_min_y) / grid_cell_size));
    int row_end = static_cast<int>(std::floor((y_max - bbox_min_y) / grid_cell_size));

    // Clamp to valid grid indices
    col_start = std::max(0, col_start);
    row_start = std::max(0, row_start);
    col_end = std::min(col_end, static_cast<int>(bbox.getLengthX() / grid_cell_size) - 1);
    row_end = std::min(row_end, static_cast<int>(bbox.getLengthY() / grid_cell_size) - 1);

    // Check if area is completely outside the grid
    if (col_start > col_end || row_start > row_end)
    {
      continue;
    }

    // Count discovered cells in this area
    int total_cells = 0;
    int discovered_cells = 0;

    for (int row = row_start; row <= row_end; row++)
    {
      for (int col = col_start; col <= col_end; col++)
      {
        // Calculate the center of this cell in world coordinates
        double cell_center_x = bbox_min_x + (col + 0.5) * grid_cell_size;
        double cell_center_y = bbox_min_y + (row + 0.5) * grid_cell_size;

        // Only count cells whose centers are in the waypoint's coverage area
        if (cell_center_x >= x_min && cell_center_x <= x_max &&
            cell_center_y >= y_min && cell_center_y <= y_max)
        {

          total_cells++;

          // Find this cell in the grid
          cellP cell_center(cell_center_x, cell_center_y);
          if (m_map_grid_cellCenter_idxs.count(cell_center) == 0)
            continue;

          unsigned int ix = m_map_grid_cellCenter_idxs[cell_center];
          double cell_val = m_grid_viz.getVal(ix, 0);
          if (cell_val > 0)
            discovered_cells++;
        }
      }
    }

    // If more than 50% of cells in this area are already discovered, remove the waypoint
    if (total_cells > 0 && ((double)discovered_cells / total_cells) > 0.5)
    {
      pruned_path.delete_vertex(i);
      Logger::info("Pruning waypoint at: (" + doubleToStringX(x, 2) +
                   ", " + doubleToStringX(y, 2) + ") - " +
                   uintToString(discovered_cells) + "/" + uintToString(total_cells) +
                   " (" + doubleToStringX(discovered_cells / double(total_cells), 2) +
                   ") cells already discovered");
    }
  }

  if (pruned_path.size() != original_path.size())
  {
    Logger::info("Pruned path from " + uintToString(original_path.size()) +
                 " to " + uintToString(pruned_path.size()) + " waypoints");
  }

  return pruned_path;
}

bool GridSearchPlanner::raisePlannerFlag()
{

  switch (m_planner_mode)
  {
  case Planner::PlannerMode::TMSTC_STAR:
    m_do_plan_paths = true;
    m_do_start_voronoi_searching = false;

    break;

  case Planner::PlannerMode::VORONOI_SEARCH:
    m_do_plan_paths = false;
    m_do_start_voronoi_searching = true;
    break;

  default:
    reportRunWarning("Unknown planner mode");
    Logger::warning("Unknown planner mode");
    return false;
  }

  return true;
}