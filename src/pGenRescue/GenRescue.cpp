/************************************************************/
/*    NAME: Steve Nomeny and Filip Stromstad                */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: GenRescue.cpp                                   */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "GenRescue.h"


#include "XYSegList.h"
#include <algorithm>


#include "NodeRecord.h"
#include "NodeRecordUtils.h"

#include "NodeMessage.h"
#include "NodeMessageUtils.h"

#include "XYPoint.h"
#include "XYFormatUtilsPoint.h"


#include <limits>


MissionRaw::MissionItem make_mission_item_wp(
    float latitude_deg1e7,
    float longitude_deg1e7,
    int32_t altitude_m,
    float param1,
    MAV_FRAME frame,
    MAV_CMD command,
    float p2 = 0,
    float p3 = 0)
{
    // WARNING this is done in consideration of CLEAN!! mission
    static uint32_t seq_num = 0;
    MissionRaw::MissionItem new_item{};
    new_item.seq = seq_num;
    new_item.frame = static_cast<uint32_t>(frame);
    new_item.command = static_cast<uint32_t>(command);
    new_item.param1 = param1;
    new_item.param2 = p2;
    new_item.param3 = p3;   
    new_item.x = latitude_deg1e7 * 1e7;
    new_item.y = longitude_deg1e7 * 1e7;
    new_item.z = altitude_m;
    new_item.mission_type = MAV_MISSION_TYPE_MISSION;
    new_item.autocontinue = 1;

    if (seq_num == 1) {
        new_item.current = 1;
    }

    seq_num++;

    return new_item;
}
bool create_missionPlan(std::vector<mavsdk::MissionRaw::MissionItem>& mission_plan, float lat_deg_home = -35.359833, float lon_deg_home = 149.164703){


    // in case of ardupilot we want to set lat lon to 0, to use current position as takeoff position
    mission_plan.push_back(make_mission_item_wp( //0
        lat_deg_home, // lat home
        lon_deg_home, // lon home
        100, // alt home
        0,
        MAV_FRAME_GLOBAL_RELATIVE_ALT,
        MAV_CMD_NAV_WAYPOINT));

    mission_plan.push_back(make_mission_item_wp( // 1 takeoff
        -35.359833, // lat
        149.164703, // lon
        41.03,
        15,
        MAV_FRAME_GLOBAL_RELATIVE_ALT,
        MAV_CMD_NAV_TAKEOFF));

    // // setup speed during mission execution
    // mission_plan.push_back(make_mission_item_wp(
    //     0, 0, 0, 0, MAV_FRAME_GLOBAL_RELATIVE_ALT, MAV_CMD_DO_CHANGE_SPEED, 9.35f, -1.0f));

    mission_plan.push_back(make_mission_item_wp( //2
        -35.359585,
        149.161392,
        100.00,
        0,
        MAV_FRAME_GLOBAL_RELATIVE_ALT,
        MAV_CMD_NAV_WAYPOINT));

    mission_plan.push_back(make_mission_item_wp( //3
        -35.366463,
        149.162231,
        100.00,
        0,
        MAV_FRAME_GLOBAL_RELATIVE_ALT,
        MAV_CMD_NAV_WAYPOINT));


    mission_plan.push_back(make_mission_item_wp( //4
        -35.366131,
        149.164581,
        100.00,
        0,
        MAV_FRAME_GLOBAL_RELATIVE_ALT,
        MAV_CMD_NAV_WAYPOINT));
 
    mission_plan.push_back(make_mission_item_wp( //5
        -35.359272,
        149.163757,
        100.00,
        0,
        MAV_FRAME_GLOBAL_RELATIVE_ALT,
        MAV_CMD_NAV_WAYPOINT));

    mission_plan.push_back(make_mission_item_wp( //6
        -35.366131, // wont do anything
        149.164581, // wont do anything
        100.00,     // wont do anything
        SPEED_TYPE_AIRSPEED,          // param 1
        MAV_FRAME_GLOBAL_RELATIVE_ALT,
        MAV_CMD_DO_CHANGE_SPEED,
        6) // param 2 - 6m/s
    );

    mission_plan.push_back(make_mission_item_wp( //7
        -35.359272,
        149.163757,
        100.00,
        0,
        MAV_FRAME_GLOBAL_RELATIVE_ALT,
        MAV_CMD_NAV_WAYPOINT));

    mission_plan.push_back(make_mission_item_wp( //8
        -35.3608654,
        149.1648848,
        41.00,
        0,
        MAV_FRAME_GLOBAL_RELATIVE_ALT,
        MAV_CMD_NAV_WAYPOINT));

    mission_plan.push_back(make_mission_item_wp( //9
        lat_deg_home,
        lon_deg_home,
        0.00,
        1, //m Minimum abort altitude
        MAV_FRAME_GLOBAL_RELATIVE_ALT,
        MAV_CMD_NAV_LAND,
        PRECISION_LAND_MODE_OPPORTUNISTIC));

    return true;
}



using namespace std;

//---------------------------------------------------------
// Constructor()

GenRescue::GenRescue()
: m_mavsdk_ptr{std::make_shared<Mavsdk>(Mavsdk::Configuration{Mavsdk::ComponentType::GroundStation})}
{
  m_visit_radius = 3;
  m_points_visited = vector<XYPoint>();
  m_unvisited_points = vector<XYPoint>();

  m_ship_status = ShipStatus();
  m_advesary_ship_status = ShipStatus();

  m_num_received_points = 0;
  m_look_ahead_steps = 2;
  
  m_generate_path = false; 
  m_deploy = false;
  m_path_algorithm = path::PathAlgorithm::GREEDY_SHORTEST_PATH;


  m_path_update_period = 2.0;
  m_park_point = {0,0};
  m_skip_point = {0,0};
  m_skip_next_point = false;

  m_pav90 = false;

  m_position_update_period = 7.0; //s
  m_still_radius = 3; //m


  m_do_fly_to_waypoint = false;
  m_do_takoff = false;

  m_health_all_ok = false;

}

//---------------------------------------------------------
// Destructor

GenRescue::~GenRescue()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool GenRescue::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  static bool newest_x = false;
  static bool newest_y = false;
  static double pos_x = 0;
  static double pos_y = 0;

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();


    if(key == "NAV_X"){
      pos_x = msg.GetDouble();
      newest_x = true;
    }
    else if(key == "NAV_Y"){
      pos_y = msg.GetDouble();
      newest_y = true;
    }
    else if(key == "NAV_SPEED"){
      m_ship_status.speed = msg.GetDouble();
    } 

    else if(key == "GENPATH_REGENERATE"){
      setBooleanOnString(m_generate_path, msg.GetString());
      // cout << "Regenerate Path: " << m_generate_path << " and " << msg.GetString() << endl;
    }
    else if(key == "DEPLOY"){
      setBooleanOnString(m_deploy, msg.GetString());
    }

    else if(key == "NODE_REPORT"){
      m_node_report_received++;
      string node_report = msg.GetString();
      NodeRecord node = string2NodeRecord(node_report);
      if(node.getName() == m_adversaryName){
        m_advesary_ship_status.position.set_vertex(node.getX(), node.getY());
        m_advesary_ship_status.speed = node.getSpeed();
        m_node_report_from_adversary_received++;

      }
    }

    else if (key == "PARK_POINT"){
      string node_msg = msg.GetString();
      Notify("DEBUG_PARK", "Park point received:" + node_msg);
      m_park_point = string2Point(node_msg);
      Notify("DEBUG_PARK", "Park point parsed: (" + to_string(m_park_point.x()) + ", " + to_string(m_park_point.y()) + ")");
    }
    else if(key == "SKIP_NEXT_POINT"){
      string node_msg = msg.GetString();
      Notify("DEBUG_SKIP", "Skip point received:" + node_msg);
      setBooleanOnString(m_skip_next_point,node_msg);
    }
    else if(key == "FOUND_SWIMMER"){
      string msg_str = msg.GetString();

      vector<string> point_info = parseString(msg_str, ',');
      if(point_info.size() != 2){
        reportRunWarning("Error parsing swimmer location: " + msg_str);
        continue;
      }
    

      string label = return_id_value(msg_str);

      // remove the id=XX from the vector of unvisited points
      auto it = std::find_if(m_unvisited_points.begin(), m_unvisited_points.end(), [label](const XYPoint& point) {
        return (point.get_label() == label);
      });

      if(it != m_unvisited_points.end()){
        m_unvisited_points.erase(it);
        m_points_visited.push_back(*it);
      }

      m_generate_path = true; // generate a new path after the swimmer is found

    } 

    else if(key == "FLY_WAYPOINT"){
      setBooleanOnString(m_do_fly_to_waypoint, msg.GetString());
    }
    else if(key == "DO_TAKEOFF"){
      setBooleanOnString(m_do_takoff, msg.GetString());
    }

    else if(key == "SWIMMER_ALERT"){
      string waypointInfo = msg.GetString();

      double point_x = return_x_value(waypointInfo);
      double point_y = return_y_value(waypointInfo);
      string label = return_id_value(waypointInfo);

      //TODO: Move this further up?
      if (find(m_known_swimmers.begin(), m_known_swimmers.end(), label) != m_known_swimmers.end()) {
        continue; //move on to the next new mail
      }
      
      XYPoint new_point{point_x, point_y, label};
      m_unvisited_points.push_back(new_point);
      m_known_swimmers.push_back(label);
      m_num_received_points++;
      m_generate_path = true;
    }
    else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);
   }


  if(newest_x && newest_y){
    m_ship_status.position.set_vertex(pos_x,pos_y);
    newest_x = false;
    newest_y = false;
  }

  //TODO: set m_generate_path to true when we receive a new point
  //TODO: also update the m_numb_received_points when we receive a new point
	
  return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool GenRescue::OnConnectToServer()
{
   registerVariables();
  //  Notify("VEHICLE_READY", "true");
   Notify("VEHICLE_READY", m_vname );
  
  return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool GenRescue::Iterate()
{
  AppCastingMOOSApp::Iterate();
  // Do your thing here!

  static bool is_armed = false;
  
  if(m_health_all_ok && !is_armed){
    
    m_action_ptr->arm_async([&](Action::Result result) {
        MOOSTrace("Arming result: %d\n", result);
        if (result == Action::Result::Success) {
            is_armed = true;
        } else {
            stringstream ss;
            ss << "Arming failed: " << result << '\n';
            std::cout << ss.str();
            reportRunWarning("Failed to arm");
            MOOSTrace(ss.str().c_str());
        }
    }); 

  }

  if(m_do_takoff && is_armed){
    // send the takeoff command
    auto start_result = m_mission_raw_ptr->start_mission();
    if (start_result != MissionRaw::Result::Success) {
        std::cout << "start failed" << std::endl;
        reportRunWarning("Failed to start mission");
    }else{
        std::cout << "Mission started" << std::endl;
        MOOSTrace("Mission started\n");
    }
    m_do_takoff = false;
  }

  if(m_do_fly_to_waypoint){
    // send the fly to waypoint command

    auto res = m_action_ptr->goto_location(m_lat_deg_home+0.0011,
                         m_lon_deg_home+0.0011,
                         564 + 60,
                         0.0);

    if(res != Action::Result::Success){
        std::cerr << "goto_location failed: " << res << '\n';
        reportRunWarning("goto_location failed");
    }else{
        std::cout << "goto_location succeeded" << '\n';
        MOOSTrace("goto_location succeeded\n");
    }
    m_do_fly_to_waypoint = false;
  }



  static double last_path_update_time = -1;

  if (m_path_algorithm == path::PathAlgorithm::ADVERSARY_PATH_PLANNING) {
    if (MOOSTime() - last_path_update_time >= m_path_update_period) {
      m_generate_path = true;
      last_path_update_time = MOOSTime();
    }
  }

  static double last_position_check_update_time = -1;
  static XYPoint last_position = m_ship_status.position;

  bool ship_still = false;

  if( MOOSTime() - last_position_check_update_time >= m_position_update_period){
    last_position_check_update_time = MOOSTime();
    
    // if the current position of the ship is within a defined radius of the last position, then we are not moving
    if(calculateDistance(m_ship_status.position, last_position) <= m_still_radius){
      ship_still = true;
    }
    else{
      ship_still = false;
    }
    last_position = m_ship_status.position;
  }


  /* If all points are received, generate the path and notify*/
  if(m_generate_path && m_deploy){
    
    switch (m_path_algorithm) {
      case path::PathAlgorithm::GREEDY_SHORTEST_PATH:
        m_unvisited_points = solveGreedyShortestPath();
        break;
      case path::PathAlgorithm::LOOK_AHEAD:
        m_unvisited_points = solveGreedyNLegs(m_ship_status.position, m_unvisited_points, m_look_ahead_steps);
        break;
      case path::PathAlgorithm::ADVERSARY_PATH_PLANNING:
        m_unvisited_points = adversaryPathPlanning();

        
        
        break;
    }
    
    
    if (!m_unvisited_points.empty()) {
      
      string update_str;
      vector<XYPoint> points_to_send = m_unvisited_points;


      if(m_skip_next_point && points_to_send.size() > 1){
        m_skip_point = points_to_send[0];
        m_skip_next_point = false;
      }


      // remove park point if it is not the only point in the list
      if (!(points_to_send.size() == 1  && points_to_send[0] == m_park_point)){
        // remove park_point if both x and y is equal

        removeParkPoint(points_to_send, m_park_point);

        removeParkPoint(points_to_send, m_skip_point);

        string path = generatePathSpec(points_to_send);
        
        update_str = path + "# capture_radius=" + to_string(m_visit_radius);


      } else{
        
        static int shipx = m_ship_status.position.x();
        static int shipy = m_ship_status.position.y();


        double degs = -25.5 + int(ship_still) * 180; 
        if(m_pav90)
          update_str = "points = format=lawnmower, label=pav90, x=11, y=-61.0, height=81, width=144, \
                                      lane_width=6, rows=east-west, startx="+to_string(shipx)+", starty="+to_string(shipy)+", degs=-25.5";
        
        else
          update_str = "points = format=lawnmower, label=pav60, x=26.6, y=-37.4, height=51, width=94, \
                                      lane_width=6, rows=east-west, startx="+to_string(shipx)+", starty="+to_string(shipy)+", degs=-25.5";
      }

      Notify("SURVEY_UPDATE", update_str);
      Notify("TRANSIT", "true");
      Notify("DEPLOY", "true");
      Notify("RETURN", "false");
      Notify("STATION_KEEP", "false");
      Notify("AT_ORIGIN", "false");
      Notify("REFUEL_NEEDED", "false");
      Notify("MOOS_MANUAL_OVERRIDE", "false");
      
      m_generate_path = false;
    }
    
  }

  // vector<XYPoint>::const_iterator it = findWaypointVisited();
  // if(it != m_unvisited_points.end()){
  //   m_points_visited.push_back(*it);
  //   m_unvisited_points.erase(it);
  // }


  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool GenRescue::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();
  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  // // Get community name from the config file
  //   if(!m_MissionReader.GetConfigurationParam("Community", m_vname))
  //   reportConfigWarning("No community name found in configuration file");


  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    if(param == "visit_radius") {
      int num;
      setIntOnString(num, value);
      if(num < 0){
        reportConfigWarning("Visit radius must be a positive integer");
        continue;
      }
      m_visit_radius = num;
      handled = true;
    }
    else if(param == "vname"){
      m_vname = value;
      handled = true;
      Notify("VEHICLE_READY", m_vname );
    }
    else if(param == "scout_name"){
      m_scout_name = value;
      handled = true;
    }
    else if(param == "look_ahead_steps"){
      int num;
      setIntOnString(num, value);
      if(num < 0){
        reportConfigWarning("Look ahead steps must be a positive integer");
        continue;
      }
      m_look_ahead_steps = num;
      handled = true;
    }
    else if( param == "adversary_name" || param == "adversary"  || param == "advName"){
      m_adversaryName = value;
      handled = true;
    }
    else if(param == "path_algorithm"){
      auto alg = path::stringToEnum(value);
      if (alg != path::PathAlgorithm::INVALID ) {
        m_path_algorithm = alg;
        handled = true;
      }
      else {
        reportConfigWarning("Invalid path algorithm: " + value);
      }
    }
    else if (param == "path_update_period" || param == "period") {
      double num;
      setDoubleOnString(num, value);
      if(num < 0){
        reportConfigWarning("Path update period must be a positive number");
        continue;
      }
      m_path_update_period = num;
      handled = true;
    } 
    else if (param == "pav90") {
      if (setBooleanOnString(m_pav90, value)){
        handled = true;
      } else {
        reportConfigWarning("Invalid boolean value for pav90: " + value); 
      }
    }
    
    else if(param == "position_update_period"){
      double num;
      setDoubleOnString(num, value);
      if(num < 0){
        reportConfigWarning("Position update period must be a positive number");
        continue;
      }
      m_position_update_period = num;
      handled = true;
    }
    else if(param == "still_radius"){
      double num;
      setDoubleOnString(num, value);
      if(num < 0){
        reportConfigWarning("Still radius must be a positive number");
        continue;
      }
      m_still_radius = num;
      handled = true;
    }


    if(!handled)
      reportUnhandledConfigWarning(orig);

  }
  
  if(m_path_algorithm == path::PathAlgorithm::ADVERSARY_PATH_PLANNING && m_adversaryName.empty()) {
    reportConfigWarning("No adversary name found in configuration file, when adversary path planning is selected");
  }
  if(m_vname.empty()) {
    reportConfigWarning("No vehicle name found in configuration file");

  registerVariables();	
  }
  if(m_scout_name.empty()) {
    reportConfigWarning("No scout name found in configuration file");
  }


  std::cout << "Vehicle name: " << m_vname << std::endl;

  ConnectionResult connection_result = m_mavsdk_ptr->add_any_connection("udp://0.0.0.0:14550");

  std::cout << "Connection result: " << connection_result << std::endl;

  if (connection_result != ConnectionResult::Success) {
    stringstream ss;
    ss << "Connection failed: " << connection_result << '\n';
    MOOSTrace(ss.str().c_str());
    std::cout << ss.str();
    reportRunWarning(ss.str());
  }

  std::cout << "Waiting to discover system..." << std::endl;
  auto system = m_mavsdk_ptr->first_autopilot(3.0);
  if (!system.has_value()) {
    stringstream ss;
    ss << "Timed out waiting for system\n";
    reportRunWarning(ss.str()); 
  }

  // m_mission_raw = MissionRaw{system.value()};
  m_mission_raw_ptr = std::make_unique<MissionRaw>(system.value());
  m_action_ptr = std::make_unique<Action>(system.value());
  m_telemetry_ptr = std::make_unique<Telemetry>(system.value());


  auto clear_result = m_mission_raw_ptr->clear_mission();
  if (clear_result != MissionRaw::Result::Success) {
      std::cout << "clear failed" << std::endl;
      reportRunWarning("Failed to clear mission");
  }

  auto download_result = m_mission_raw_ptr->download_mission();
  if (download_result.first != MissionRaw::Result::Success) {
      std::cout << "Download failed" << std::endl;
      reportRunWarning("Failed to download mission");
  }

  // first point in case of ardupilot is always home
  auto mission_plan = download_result.second;

  MissionRaw::MissionItem home_point = mission_plan[0];

  stringstream ss;
  ss << "Home point: " << home_point << std::endl;
  ss << "-----------------------------------------------\n";
  MOOSDebugWrite(ss.str());
  ss.clear();

  std::cout << "Home point: " << home_point << std::endl;
  std::cout << "-----------------------------------------------" << std::endl;

  mission_plan.clear();

  m_lat_deg_home = home_point.x * 1e-7;
  m_lon_deg_home = home_point.y * 1e-7;

  create_missionPlan(mission_plan,  m_lat_deg_home, m_lon_deg_home);

  auto upload_result = m_mission_raw_ptr->upload_mission(mission_plan);
    if (upload_result != MissionRaw::Result::Success) {
        std::cout << "upload failed" << std::endl;
        std::cout << "upload result: " << upload_result << std::endl;
        stringstream ss;
        ss << "Failed to upload mission" << std::endl;
        ss << "upload result: " << upload_result << std::endl;
        reportRunWarning(ss.str());
    }

  m_mission_raw_ptr->set_current_mission_item(0);


  m_telemetry_ptr->subscribe_health_all_ok([&, this](bool is_health_all_ok) {
    this->m_health_all_ok = is_health_all_ok;
  });

  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void GenRescue::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
  Register("NAV_SPEED", 0);
  Register("VISIT_POINT", 0);

  Register("GENPATH_REGENERATE", 0);
  Register("DEPLOY", 0);
  Register("FOUND_SWIMMER", 0);
  Register("SWIMMER_ALERT", 0);
  Register("NODE_REPORT", 0);
  Register("PARK_POINT", 0);
  Register("SKIP_NEXT_POINT", 0);

  Register("FLY_WAYPOINT", 0);
  Register("DO_TAKEOFF", 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool GenRescue::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "File:  pGenRescue                           " << endl;
  m_msgs << "============================================" << endl;

  ACTable actab(2);
  actab << "Visit Radius:" << m_visit_radius;
  actab << "Path Algorithm:" << path::enumToString(m_path_algorithm);
  if (m_path_algorithm == path::PathAlgorithm::LOOK_AHEAD) {
    actab << "Look Ahead Steps:" << m_look_ahead_steps;
  }
  actab << "Total Points Received:" << m_num_received_points;
  actab << "Current Location:" << m_ship_status.position.get_spec_xy();
  actab << "Generate Path:" << m_generate_path;
  actab << "Using pav90:" << m_pav90;
  actab << "Position Update Period (s):" << m_position_update_period;
  actab << "Still Radius (m):" << m_still_radius;
  actab << "skip_next_point:" << m_skip_next_point;
  actab << "Skip Point:" << m_skip_point.get_spec_xy();
  actab << "Do set fly waypoint:" << boolToString(m_do_fly_to_waypoint);
  actab << "Do takeoff:" << boolToString(m_do_takoff);
  m_msgs << actab.getFormattedString();
  m_msgs << "\n\n" << endl;


  if(m_path_algorithm == path::PathAlgorithm::ADVERSARY_PATH_PLANNING){
    m_msgs << "Adversary Info" << endl;
    m_msgs << "--------------------------" << endl;
    ACTable actab1(2);
    actab1 << "Adversary Name:" << m_adversaryName;
    actab1 << "Adversary Position:" << m_advesary_ship_status.position.get_spec_xy();
    actab1 << "Adversary Speed:" << m_advesary_ship_status.speed;
    actab1 << "Adversary Path update frequency (Hz):" << 1.0/m_path_update_period; 
    m_msgs << actab1.getFormattedString();
    m_msgs << "\n\n" << endl;
  }

  m_msgs << "Node Report Status" << endl;
  m_msgs << "--------------------------" << endl;
  ACTable actab3(2);
  actab3 << "Node Reports Received:" << m_node_report_received;
  actab3 << "Node Reports from Adversary Received:" << m_node_report_from_adversary_received;
  m_msgs << actab3.getFormattedString();
  m_msgs << "\n\n" << endl;
  
  
  m_msgs << "Tour Status" << endl;
  m_msgs << "--------------------------" << endl;
  uint visited = m_points_visited.size();
  uint unvisited = m_unvisited_points.size();
  ACTable actab2(2);
  actab2 << "Points Visited:" << visited;
  actab2 << "Points Unvisited:" << unvisited;
  m_msgs << actab2.getFormattedString();


  return(true);
}


std::vector<XYPoint> GenRescue::solveGreedyShortestPath() const {
    std::vector<XYPoint> path;

    XYPoint temp_location = m_ship_status.position;

    // Add the virtual initial point (vehicle's present position) to the path
    path.push_back(temp_location);

    // Create a copy of the received points to work with
    std::vector<XYPoint> remainingPoints = m_unvisited_points;

    while (!remainingPoints.empty()) {
        // Find the nearest unvisited point
        auto nearestPointIt = std::min_element(remainingPoints.begin(), remainingPoints.end(),
            [temp_loc = temp_location](const XYPoint& point1, const XYPoint& point2) {
                return (calculateDistance(temp_loc, point1) < calculateDistance(temp_loc, point2));
            });

        // Add the nearest point to the path
        path.push_back(*nearestPointIt);

        // Update the current location to the newly added point
        temp_location = *nearestPointIt;

        // Remove the point from the remaining unvisited points
        remainingPoints.erase(nearestPointIt);
    }


    path.erase(path.begin()); // Remove the virtual initial point from the path

    return path;
}



// Greedy algorithm to determine the next point in the path
XYPoint getNextGreedyPoint(const XYPoint& currentLocation, const std::vector<XYPoint>& remainingPoints) {
    auto it = std::min_element(remainingPoints.begin(), remainingPoints.end(),
        [&currentLocation](const XYPoint& a, const XYPoint& b) {
            return calculateDistance(currentLocation, a) < calculateDistance(currentLocation, b);
        });
    return *it;
}

// Main function to solve the problem with a greedy algorithm considering the first N legs
std::vector<XYPoint> GenRescue::solveGreedyNLegs(const XYPoint& currentLocation, std::vector<XYPoint> unvisitedPoints, int N) const{
    
    // If the total distance is 0 (only one point remain unvisited), return the unvisited points as the path
    if ( unvisitedPoints.size() < 2) {
        return unvisitedPoints;
    }


    double bestNLegDistance = std::numeric_limits<double>::max();
    std::vector<XYPoint> bestNLegPath;
    std::vector<XYPoint> finalPath;

    // Loop through each unvisited point to find the best N legs path
    for (auto& startPoint : unvisitedPoints) {
        std::vector<XYPoint> tempRemainingPoints = unvisitedPoints;
        tempRemainingPoints.erase(std::remove(tempRemainingPoints.begin(), tempRemainingPoints.end(), startPoint), tempRemainingPoints.end());
        std::vector<XYPoint> tempPath;
        double totalDistance = 0.0;
        XYPoint current = currentLocation;

        // Calculate the greedy path for the first N legs
        for (int i = 0; i < N && !tempRemainingPoints.empty(); ++i) {
            XYPoint nextPoint = getNextGreedyPoint(current, tempRemainingPoints);
            totalDistance += calculateDistance(current, nextPoint);
            current = nextPoint;
            tempPath.push_back(nextPoint);
            tempRemainingPoints.erase(std::remove(tempRemainingPoints.begin(), tempRemainingPoints.end(), nextPoint), tempRemainingPoints.end());
        }

        
        // Check if the current N legs path is better than the previous best
        if (totalDistance < bestNLegDistance) {
            bestNLegDistance = totalDistance;
            bestNLegPath = tempPath;
        }
    }

    // Construct the final path by starting with the best N legs and continuing greedily
    for (auto& point : bestNLegPath) {
        finalPath.push_back(point);
        unvisitedPoints.erase(std::remove(unvisitedPoints.begin(), unvisitedPoints.end(), point), unvisitedPoints.end());
    }

    XYPoint current = bestNLegPath.back();
    while (!unvisitedPoints.empty()) {
        XYPoint nextPoint = getNextGreedyPoint(current, unvisitedPoints);
        finalPath.push_back(nextPoint);
        current = nextPoint;
        unvisitedPoints.erase(std::remove(unvisitedPoints.begin(), unvisitedPoints.end(), nextPoint), unvisitedPoints.end());
    }

    return finalPath;
}

std::vector<XYPoint> GenRescue::twoStepLookahead(const XYPoint& ship_position, const std::vector<XYPoint>& points) const {
    return solveGreedyNLegs(ship_position, points, 2 );  // This should be replaced with actual lookahead logic
}

string GenRescue::generatePathSpec(std::vector<XYPoint> points) const
{
  XYSegList seglist;
  for(auto point : points){
    seglist.add_vertex(point);
  }

  string update_str = "points = ";
  update_str       += seglist.get_spec();
  return update_str;
}


vector<XYPoint>::const_iterator GenRescue::findWaypointVisited() const
{
  auto it = std::find_if(m_unvisited_points.begin(), m_unvisited_points.end(), [this](const XYPoint& point) {
    return (calculateDistance(point, m_ship_status.position) <= m_visit_radius);
  });

  return it;
}

int return_x_value(string sval){
  sval = sval + ","; //easier parsing
  size_t start = sval.find("x=") + 2; //+2 to skip "x="
  size_t end = sval.find(",", start);
  string x_str = sval.substr(start, end - start);
  return stoi(x_str);
}

int return_y_value(string sval){
  sval = sval + ","; //easier parsing
  size_t start = sval.find("y=") + 2; //+2 to skip "y="
  size_t end = sval.find(",", start);
  string y_str = sval.substr(start, end - start);
  return stoi(y_str);
}

string return_id_value(string sval){
  sval = sval + ","; //easier parsing
  size_t start = sval.find("id=") + 3; //+3 to skip "id="
  size_t end = sval.find(",", start);
  string id_str = sval.substr(start, end - start);
  return id_str;
}

// Optimal path algorithm




double calculateDistance(const XYPoint& point1, const XYPoint& point2) {
    return std::sqrt(std::pow(point1.x() - point2.x(), 2) + std::pow(point1.y() - point2.y(), 2));
}

double calculateETA(const ShipStatus& myShip, const XYPoint& point){
    double distance = calculateDistance(myShip.position, point);

    if (myShip.speed == 0) {
        return std::numeric_limits<double>::infinity();
    }

    return distance / myShip.speed;  // Time = Distance / Speed
}

std::vector<double> calculatePathETAs(const ShipStatus& ship, const std::vector<XYPoint>& path) {
    std::vector<double> etas;
    double totalTime = 0.0;
    XYPoint previousPoint = ship.position;

    for (const auto& point : path) {
        double travelTime = calculateDistance(previousPoint, point) / ship.speed;
        totalTime += travelTime;
        etas.push_back(totalTime);
        previousPoint = point;
    }

    return etas;
}


void writeStrToFile(const std::string& str, const std::string& filename="/home/steve/moos-ivp-scnomeny/src/pGenRescue/log.txt") {

    // append to file 



    std::ofstream file(filename,std::ios::app);
    file << str << endl;
    file.close();
}

std::vector<XYPoint> GenRescue::updateOurPath(const ShipStatus& myShip, const ShipStatus& adversaryShip, const std::vector<XYPoint>& adversaryPath, std::vector<double>& adversaryETA) const {
    std::vector<XYPoint> path;
    std::vector<XYPoint> remainingPoints = m_unvisited_points;
    std::vector<XYPoint> discardedPoints;

    auto tempAdversaryShip = adversaryShip;
    auto tempAdversaryPath = adversaryPath;

    while (!remainingPoints.empty()) {
        std::vector<XYPoint> tempPath = twoStepLookahead(myShip.position, remainingPoints);
        
        if(tempPath.size() == remainingPoints.size()){
            cout << "both paths are the same size" << endl;
            writeStrToFile("both paths are the same size " + std::to_string(tempPath.size()));
           
        }
    // print elements in remaining points
        for (const auto& point : tempPath) {
            cout << "Remaining tempPath points: (" << point.x() << ", " << point.y() << ")" << endl;
            writeStrToFile("Remaining tempPath points: (" + std::to_string(point.x()) + ", " + std::to_string(point.y()) + ")");
        }
    // print elements in remaining points
        for (const auto& point : tempAdversaryPath) {
            cout << "Remaining adversary points: (" << point.x() << ", " << point.y() << ")" << endl;
            writeStrToFile("Remaining points: (" + std::to_string(point.x()) + ", " + std::to_string(point.y()) + ")");
        }   
        
        
        if (!tempPath.empty()) {

            XYPoint firstPoint = tempPath.front();
            double myETA = calculateETA(myShip, firstPoint);


            XYPoint firstAdversaryPoint = tempAdversaryPath.front();
            

            if( !(firstPoint == firstAdversaryPoint)){
                cout << "First point in tempPath is not equal to first point in tempAdversaryPath" << endl;
                writeStrToFile("First point in tempPath is not equal to first point in tempAdversaryPath \n First point in tempPath: (" + std::to_string(firstPoint.x()) + ", " + std::to_string(firstPoint.y()) + 
                + "\n First point in tempAdversaryPath: (" + std::to_string(firstAdversaryPoint.x()) + ", " + std::to_string(firstAdversaryPoint.y()) + ")");
                path = tempPath;



                // TODO: Check if by following the adversarys path that we can get a better time than them
                // Maybe not, we technically do not know what path the adversary is taking
                break;
            }

            
            
            auto it = std::find(adversaryPath.begin(), adversaryPath.end(), firstPoint);
            int index = it - adversaryPath.begin();

            double advETA = adversaryETA[index];

            
            cout << "My ETA: " << myETA << " Adversary ETA: " << advETA << endl; 
            writeStrToFile("My ETA: " + std::to_string(myETA) + " Adversary ETA: " + std::to_string(advETA));
            if (myETA < advETA || advETA == std::numeric_limits<double>::infinity() ) {
                path = tempPath;
                break;
            } else {
                cout << "Discarding point: (" << firstPoint.x() << ", " << firstPoint.y() << ")" << endl;
                writeStrToFile("Discarding point: (" + std::to_string(firstPoint.x()) + ", " + std::to_string(firstPoint.y()) + ")");
                
                tempAdversaryShip.position = firstAdversaryPoint;
                tempAdversaryPath.erase(tempAdversaryPath.begin());
                adversaryETA.erase(adversaryETA.begin());

                discardedPoints.push_back(firstPoint);
                remainingPoints.erase(std::remove(remainingPoints.begin(), remainingPoints.end(), firstPoint), remainingPoints.end());
            }
        } else {
            break;
        }
        // cout << "Remaining points: " << remainingPoints.size() << endl;
    }

    cout << "Checking fallback ..." << endl;
    writeStrToFile("Checking fallback ...");

    // Fallback: Target points with the highest adversary ETA from discarded points
    if (path.empty() && !discardedPoints.empty()) {
        writeStrToFile("Fallback: Target points with the highest adversary ETA from discarded points");
        // std::sort(discardedPoints.begin(), discardedPoints.end(), [&](const XYPoint& a, const XYPoint& b) {
        //     return calculateETA(adversaryShip, a) > calculateETA(adversaryShip, b); // Sorting in descending order by adversary ETA
        // });
        path.resize(adversaryPath.size());
        // reverse copy 
        std::reverse_copy(adversaryPath.begin(), adversaryPath.end(), path.begin()); 
        for (const auto& point : path) {
            cout << "Final path points: (" << point.x() << ", " << point.y() << ")" << endl;
            writeStrToFile("Final path points: (" + std::to_string(point.x()) + ", " + std::to_string(point.y()) + ")");
        } 
    }

    // // Output the path we plan to follow
    // for (const auto& point : path) {
    //     std::cout << "We will visit: (" << point.x() << ", " << point.y() << ")" << std::endl;
    // }

    cout << "Path updated..." << endl;
    return path;
}

std::vector<XYPoint> GenRescue::adversaryPathPlanning() {
    // Compute adversary path ETAs
    cout << "Calculating adversary path..." << endl;
    std::vector<XYPoint> adversaryPath = twoStepLookahead(m_advesary_ship_status.position, m_unvisited_points);
    

    NodeMessage node_msg;

    XYSegList seglist;
    for(auto point : adversaryPath){
      seglist.add_vertex(point);
    }
    string path_str =  seglist.get_spec();
    
    node_msg.setSourceNode(m_vname);
    node_msg.setDestNode(m_scout_name);
    node_msg.setVarName("ADVERSARY_PATH");
    node_msg.setStringVal(path_str);


    string msg = node_msg.getSpec();
    Notify("NODE_MESSAGE_LOCAL", msg);
    

    cout << "Calculating adversary eta..." << endl;

    auto adversaryETAs = calculatePathETAs(m_advesary_ship_status, adversaryPath);

    cout << "Begin updating path..." << endl;

    auto calculated_path = updateOurPath(m_ship_status, m_advesary_ship_status, adversaryPath, adversaryETAs);

    return calculated_path;
}



void GenRescue::removeParkPoint(std::vector<XYPoint>& points, const XYPoint& parkPoint) {
  Notify("DEBUG_PARK" , "Park point: (" + std::to_string(parkPoint.x()) + ", " + std::to_string(parkPoint.y()) + ")");
  auto it = find_if(points.begin(), points.end(), [parkPoint](const XYPoint& point) {
    return (point.x() == parkPoint.x() && point.y() == parkPoint.y());
  });

  Notify("DEBUG_PARK" ,"Park point found: " + std::to_string(it != points.end()));

  if(it != points.end()){
    points.erase(it);
    Notify("DEBUG_PARK" ,"Park point removed");
  }

  // write out the points in the unvisited points
  for (const auto& point : points) {
    Notify("DEBUG_PARK" ," Unvisited points: (" + std::to_string(point.x()) + ", " + std::to_string(point.y()) + ")");
  }
}