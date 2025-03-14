/*****************************************************************/
/*    NAME: Steve Nomeny                                         */
/*    ORGN: NTNU, Trondheim                       */
/*    FILE: GridSearchPlanner.cpp                                       */
/*    DATE: Feb 2025                                            */
/*****************************************************************/

#include <iterator>
#include "GridSearchPlanner.h"
#include "MBUtils.h"
#include "NodeRecord.h"
#include "NodeRecordUtils.h"
#include "XYFormatUtilsConvexGrid.h"
#include "XYGridUpdate.h"
#include "ACTable.h"
#include "XYFormatUtilsPoly.h"

#include <numeric>
#include <algorithm>

#include "Logger.h"
#include "TMSTCVisualization.h"
//---------------------------------------------------------
// Constructor()

GridSearchPlanner::GridSearchPlanner()
{
  m_sensor_radius = 10;
  m_do_plan_paths = false;
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool GridSearchPlanner::OnNewMail(MOOSMSG_LIST &NewMail)
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
    // std::string community = msg.GetCommunity();

    if ((key == "NODE_REPORT") || (key == "NODE_REPORT_LOCAL"))
      handleMailNodeReport(sval);
    else if (key == "IGNORED_REGION_ALERT")
      handleMailIgnoredRegionAlert(sval);
    else if (key == "GSP_VISUALIZE_PLANNER_GRIDS")
      setBooleanOnString(m_visualize_planner_grids, sval);
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
    m_do_plan_paths = false;
  }

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
      else if (param == "visualize_planner_grids")
        handled = setBooleanOnString(m_visualize_planner_grids, value);

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
  m_tmstc_grid_converter.setSensorRadius(m_sensor_radius * 0.5);

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
  Register("IGNORED_REGION_ALERT", 0);
}

void GridSearchPlanner::doPlanPaths()
{
  m_tmstc_grid_converter.transformGrid();
}


void GridSearchPlanner::updateTMSTCVehiclePositions(){
  std::vector<XYPoint> vpos;
  for (auto [name, pos] : m_map_drone_positions)
    vpos.push_back(pos);

  m_tmstc_grid_converter.setVehiclePositions(vpos);

}
void GridSearchPlanner::updateTMSTCIgnoredRegions(){
  std::vector<XYPolygon> iregs;
  for (auto [name, poly] : m_map_ignored_regions_poly)
    iregs.push_back(poly);

  m_tmstc_grid_converter.setIgnoreRegions(iregs);

}
//------------------------------------------------------------
// Procedure: handleMailNodeReport()

void GridSearchPlanner::handleMailNodeReport(std::string str)
{
  NodeRecord record = string2NodeRecord(str);
  if (!record.valid())
    return;

  std::string name = record.getName();
  double posx = record.getX();
  double posy = record.getY();

  XYPoint position(posx, posy);

  m_map_drone_positions[name] = position;

  updateTMSTCVehiclePositions();
  // Logger::info("NodeReport position: (" + doubleToStringX(posx, 2) + ", " + doubleToStringX(posy, 2) + "), name: " + name);
}

//------------------------------------------------------------
// Procedure: handleMailIgnoredRegion()

/// @brief Handles alerts for ignored regions
/// @param str The input string containing region information\
///            Format: "reg:: name" \
///                    "unreg:: ignored_region_str"
void GridSearchPlanner::handleMailIgnoredRegionAlert(std::string str)
{
  str = stripBlankEnds(str);

  if (strContains(str, "unreg::"))
  {
    unregisterIgnoredRegion(str.substr(7));
    m_do_plan_paths = true;
    return;
  }
  else if (strContains(str, "reg::"))
  {
    registerIgnoredRegion(str.substr(5));
    m_do_plan_paths = true;
    return;
  }

  reportRunWarning("Received Invalid region string: " + str);
  Logger::warning("Received Invalid region string: " + str);
  return;
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

bool GridSearchPlanner::buildReport()
{

  m_msgs << "Grid Search Planner Configuration" << std::endl;
  m_msgs << "---------------------------------" << std::endl;
  m_msgs << "Sensor radius: " << doubleToStringX(m_sensor_radius, 1) << std::endl;
  m_msgs << "Visualize planner grids: " << boolToString(m_visualize_planner_grids) << std::endl;
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
  for (const auto &[drone, pos] : m_map_drone_positions)
  {
    std::string posx_str = doubleToStringX(pos.get_vx(), 2);
    std::string posy_str = doubleToStringX(pos.get_vy(), 2);
    
    std::string region_coord_str = "-";
    std::string spanning_coord_str = "-";

    if(grids_converted){
      auto region_coord = m_tmstc_grid_converter.getVehicleRegionPosition(pos);
      region_coord_str = "(" + doubleToStringX(region_coord.get_vx(), 0) + ", " + doubleToStringX(region_coord.get_vy(), 0) + ")";
      auto spanning_coord = m_tmstc_grid_converter.getVehicleSpanningPosition(pos);
      spanning_coord_str = "(" + doubleToStringX(spanning_coord.get_vx(), 0) + ", " + doubleToStringX(spanning_coord.get_vy(), 0) + ")";
    }

    actab << drone << posx_str << posy_str << region_coord_str << spanning_coord_str;
  }

  m_msgs << actab.getFormattedString();
  m_msgs << std::endl;
  
  auto num_ignored_regions = m_map_ignored_regions_poly.size();
  m_msgs << "Number of ignored Regions: " << uintToString(num_ignored_regions) << std::endl;
  m_msgs << std::endl;
  
  
  if(grids_converted){
    
    
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
    std::vector<std::pair<int, int>> robot_start_positions = m_tmstc_grid_converter.getVehicleRegionPositions();
    
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

    for(auto const pos : region_grid_centers){
      int z = pos.get_vz(); // 1 is free, 0 is occupied
      region_free_cells += z;
    }
    region_occupied_cells = region_total_cells - region_free_cells;
    std::string region_free_cells_str = uintToString(region_free_cells);
    std::string region_occupied_cells_str = uintToString(region_occupied_cells);

    int spanning_free_cells = 0;
    int spanning_occupied_cells = 0;
    for(auto const pos : spanning_grid_centers){
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
 
  
    m_msgs << std::endl;
    m_msgs << "Generated Mat Grids" << std::endl;
    m_msgs << "---------------------------------" << std::endl;
    std::stringstream ss;
    TMSTCViz::visualizeInitialMap(regionMap, robot_start_positions, ss, false);
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

void GridSearchPlanner::postTMSTCGrids(bool active)
{

  static bool prev_active = false;

  if (!active && !prev_active) // if not active and not previously active
    return;

  if (!m_tmstc_grid_converter.isGridsConverted())
    return;

  std::vector<XYPoint> regionGridPoints = m_tmstc_grid_converter.getRegionGridCenters();
  std::vector<XYPoint> downsampledGridPoints = m_tmstc_grid_converter.getSpanningGridCenters();


  double transparancy = 0.1;

  int idx = 0;
  for (auto point : regionGridPoints)
  {
    XYCircle circle(point.x(), point.y(), m_sensor_radius*0.5);

    circle.set_label("Sr_" + std::to_string(idx++));
    circle.set_label_color("off");
    circle.set_edge_color("yellow");
    circle.set_color("fill", "off");
    circle.set_transparency(transparancy);
    circle.set_edge_size(2);
    circle.set_vertex_size(2);
    circle.set_active(active);

    if (point.z() == 0)
      circle.set_color("fill", "yellow");
    // Logger::info("Region point: " + doubleToStringX(point.x(), 2) + ", " + doubleToStringX(point.y(), 2) + ", " + doubleToStringX(point.z(), 2));
    Notify("VIEW_CIRCLE", circle.get_spec());
  }

  idx = 0;
  for (auto point : downsampledGridPoints)
  {
    XYCircle circle(point.x(), point.y(), m_sensor_radius*0.5);

    circle.set_label("Sdr_" + std::to_string(idx++));
    circle.set_label_color("off");
    circle.set_edge_color("red");
    circle.set_color("fill", "off");
    circle.set_transparency(transparancy);
    circle.set_edge_size(2);
    circle.set_vertex_size(2);
    circle.set_active(active);

    if (point.z() == 0)
      circle.set_color("fill", "red");

    Notify("VIEW_CIRCLE", circle.get_spec());
  }

  prev_active = active;

  m_tmstc_grid_converter.saveSpanningGridToFile("downsampled_grid.txt");
}
