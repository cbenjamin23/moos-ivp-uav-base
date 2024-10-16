/************************************************************/
/*    NAME: Steve Carter Feujo Nomeny                       */
/*    ORGN: NTNU, MIT                                       */
/*    FILE: UAV_Model.h                                     */
/*    DATE: September 9th, 2024                             */
/************************************************************/

#include <iostream>
#include <cmath> 
#include <cstdlib>
#include "UAV_Model.h"
#include "MBUtils.h"
#include "AngleUtils.h"
#include "MOOSGeodesy.h"

#include "thread"

#include "definitions.h"




#include <cmath>

//------------------------------------------------------------------------
// Constructor

UAV_Model::UAV_Model(std::shared_ptr<WarningSystem> ws):
  m_mavsdk_ptr{std::make_shared<mavsdk::Mavsdk>(mavsdk::Mavsdk::Configuration{mavsdk::Mavsdk::ComponentType::GroundStation})},

  callbackMOOSTrace{nullptr},
  callbackReportRunW{nullptr},
  callbackRetractRunW{nullptr},
  callbackReportEvent{nullptr},

  m_health_all_ok{false},
  m_is_armed{false},
  m_in_air{false},
  m_target_altitudeAGL{120.0},
  m_last_sent_altitudeAGL{double(NAN)}
{
  // Initalize the configuration variables

  m_warning_system_ptr = ws;


  // Initalize the state variables
  m_position = mavsdk::Telemetry::Position();
  m_attitude_ned = mavsdk::Telemetry::EulerAngle();
  m_velocity_ned = mavsdk::Telemetry::VelocityNed();
  m_battery = mavsdk::Telemetry::Battery();
  m_flight_mode = mavsdk::Telemetry::FlightMode::Unknown;


  m_home_coord = XYPoint(0, 0);
  m_current_loiter_coord = XYPoint(0, 0);
  m_next_waypoint_coord = XYPoint(0, 0);
  m_heading_waypoint_coord = XYPoint(0, 0);

}


//------------------------------------------------------------------------
// Procedure: connectToUAV()

bool UAV_Model::connectToUAV(std::string url)
{
  if(url.empty()){
    // reportRunWarning("No URL specified");
    return false;
  }


  MOOSTraceFromCallback("Connecting to the URL: " + url + "\n");

  std::cout << "Connecting to the URL: " << url << std::endl;
  mavsdk::ConnectionResult connection_result = m_mavsdk_ptr->add_any_connection(url);




  if (connection_result != mavsdk::ConnectionResult::Success) {
    std::stringstream ss;
    ss << "Connection failed: " << connection_result << '\n';
    MOOSTraceFromCallback(ss.str().c_str());

    std::cout << ss.str() << std::endl;
    m_warning_system_ptr->monitorWarningForXseconds(ss.str(), WARNING_DURATION);

    return false;
  }

  std::cout << "Connected to UAV\n";


  MOOSTraceFromCallback("Connecting to the URL: " + url + "\n");
  MOOSTraceFromCallback("Waiting to discover system...\n");

  auto system = m_mavsdk_ptr->first_autopilot(3.0);
  if (!system.has_value()) {
    m_warning_system_ptr->monitorWarningForXseconds(WARNING_TIMED_OUT, WARNING_DURATION);
    std::cout << "UAV System NOT discovered\n";
    return false;
  }

  std::cout << "UAV System discovered\n";

  m_system_ptr = system.value();

  // m_mission_raw = MissionRaw{system.value()};
  m_mission_raw_ptr = std::make_unique<mavsdk::MissionRaw>(m_system_ptr);
  m_action_ptr = std::make_unique<mavsdk::Action>(m_system_ptr);
  m_telemetry_ptr = std::make_unique<mavsdk::Telemetry>(m_system_ptr);
  m_mavPass_ptr = std::make_unique<mavsdk::MavlinkPassthrough>(m_system_ptr);
  m_param_ptr = std::make_unique<mavsdk::Param>(m_system_ptr);

  std::cout << "Created mission_raw, action, telemetry, mavlinkPassthrough and param\n";

  auto clear_result = m_mission_raw_ptr->clear_mission();
  if (clear_result != mavsdk::MissionRaw::Result::Success) {
      m_warning_system_ptr->monitorWarningForXseconds("Failed to clear mission", WARNING_DURATION);

  }

  auto download_result = m_mission_raw_ptr->download_mission();


  std::vector<mavsdk::MissionRaw::MissionItem> mission_plan;

  if (download_result.first != mavsdk::MissionRaw::Result::Success) {
      m_warning_system_ptr->monitorWarningForXseconds("Failed to download mission", WARNING_DURATION);
      std::cout << "Failed to download mission - Using default home location\n";

      mavsdk::MissionRaw::MissionItem home_item{};
      home_item.seq = 0;
      home_item.frame = static_cast<uint32_t>(MAV_FRAME_GLOBAL);
      home_item.command = static_cast<uint32_t>(MAV_CMD_NAV_WAYPOINT);
      home_item.current = 1;
      home_item.autocontinue = 1;
      home_item.param1 = 0;
      home_item.param2 = 0;
      home_item.param3 = 0;   
      home_item.param4 = 0;   
      home_item.x = 633975181; // default at NTNU airport
      home_item.y = 101435316; // default at NTNU airport
      home_item.z = 106.25;    // default at NTNU airport
      home_item.mission_type = MAV_MISSION_TYPE_MISSION;
      mission_plan.push_back(home_item);
  } else {
      mission_plan = download_result.second;
  }

  mavsdk::MissionRaw::MissionItem home_point = mission_plan[0];

  std::stringstream ss;
  ss << "Home point: " << home_point << std::endl;
  ss << "-----------------------------------------------\n";
  MOOSTraceFromCallback(ss.str());
  std::cout << ss.str();
  ss.clear();

  mission_plan.clear();


  m_home_coord.set_vx( home_point.x * 1e-7);
  m_home_coord.set_vy( home_point.y * 1e-7);

  if(home_point.frame == MAV_FRAME_GLOBAL){
    m_home_coord.set_vz( home_point.z);
  }else{
    std::cout << "Home point is not in global frame, but in frame" << home_point.frame << std::endl;
    m_warning_system_ptr->monitorWarningForXseconds("Home point is not in global frame, but in frame" + intToString(home_point.frame) , WARNING_DURATION);
  }


  std::cout << "Home point: " << m_home_coord.x() << ", " << m_home_coord.y()  << " , " << home_point.z << std::endl;

  create_missionPlan(mission_plan,  m_home_coord.x(), m_home_coord.y());

  auto upload_result = m_mission_raw_ptr->upload_mission(mission_plan);
    if (upload_result != mavsdk::MissionRaw::Result::Success) {
        m_warning_system_ptr->monitorWarningForXseconds("Mission upload failed", WARNING_DURATION);
        std::stringstream ss;
        ss << "Failed to upload mission" << std::endl;
        ss << "upload result: " << upload_result << std::endl;
        MOOSTraceFromCallback(ss.str());
    }

  m_mission_raw_ptr->set_current_mission_item(0);

  return true;
}

bool UAV_Model::startMission() const{

  if(!m_is_armed){
    // reportRunWarning("Not armed");

    m_warning_system_ptr->monitorCondition( WARNING_UAV_NOT_ARMED,
                                      [&](){return !m_is_armed;}
                                      );   
    return false;
  }

  auto start_result = m_mission_raw_ptr->start_mission();

  if (start_result != mavsdk::MissionRaw::Result::Success) {
      m_warning_system_ptr->monitorWarningForXseconds("Failed to start mission", WARNING_DURATION);
      return false;
  }
  
  MOOSTraceFromCallback("Mission started\n");
  return true;
  
}





bool UAV_Model::sendArmCommandIfHealthyAndNotArmed() const{
  
  if(m_health_all_ok && !m_is_armed){
    commandArmAsync();
    return true;
  }

  m_warning_system_ptr->monitorWarningForXseconds("UAV is not healthy or is already armed", WARNING_DURATION);
  return false;
}



bool UAV_Model::subscribeToTelemetry(){

  m_telemetry_ptr->subscribe_armed([&](bool is_armed) {
    m_is_armed = is_armed;
  });


  m_telemetry_ptr->subscribe_health_all_ok([&, this](bool is_health_all_ok) {
    this->m_health_all_ok = is_health_all_ok;
  });


  m_telemetry_ptr->subscribe_position([&](mavsdk::Telemetry::Position position) {
    m_position = position;
    
    m_in_air = (m_position.relative_altitude_m >= IN_AIR_HIGHT_THRESHOLD);
    
  });

  m_telemetry_ptr->subscribe_attitude_euler([&](mavsdk::Telemetry::EulerAngle attitude_ned) {
    m_attitude_ned = attitude_ned;
  });


  m_telemetry_ptr->subscribe_velocity_ned([&](mavsdk::Telemetry::VelocityNed vel) {
    m_velocity_ned = vel;
  });


  m_telemetry_ptr->subscribe_battery([&](mavsdk::Telemetry::Battery battery) {
    m_battery = battery;
  });



  m_telemetry_ptr->subscribe_flight_mode([&](mavsdk::Telemetry::FlightMode flight_mode) {
    m_flight_mode = flight_mode;
  });


  // gives wrong data
  // m_telemetry_ptr->subscribe_in_air([&](bool in_air) {
  //   m_in_air = in_air;
  // });






  return true;
}


///////////////////////////////////
/////////////  POLLING  //////////
///////////////////////////////////

void UAV_Model::pollAllParametersAsync()
{

  getParameterAsync(Parameters::AIRSPEED_TARGET_CRUISE);
  getParameterAsync(Parameters::AIRSPEED_MAX);
  getParameterAsync(Parameters::AIRSPEED_MIN);
  
}

bool UAV_Model::getParameterAsync(Parameters param_enum)
{
    switch (param_enum)
    {
    case Parameters::AIRSPEED_TARGET_CRUISE:
      m_action_ptr->get_target_speed_async([this](mavsdk::Action::Result result, float target_speed) {
        if (result != mavsdk::Action::Result::Success) {
            std::stringstream ss;
            ss << "Failed to get target speed: " << result;
            m_warning_system_ptr->monitorWarningForXseconds(ss.str(), WARNING_DURATION);
            return;
        }
        std::cout << "target speed: " << target_speed << std::endl;
        m_polled_params.target_airspeed_cruise = target_speed;
      });
      break;
      
    case Parameters::AIRSPEED_MAX:
      m_action_ptr->get_maximum_speed_async([this](mavsdk::Action::Result result, float max_speed) {
        if (result != mavsdk::Action::Result::Success) {
            std::stringstream ss;
            ss << "Failed to get maximum speed: " << result;
            m_warning_system_ptr->monitorWarningForXseconds(ss.str(), WARNING_DURATION);
            return;
        }

        m_polled_params.max_airspeed = static_cast<int>(std::round(max_speed));
      });
      break;
    case Parameters::AIRSPEED_MIN:
      m_action_ptr->get_minimum_speed_async([this](mavsdk::Action::Result result, float min_speed) {
        if (result != mavsdk::Action::Result::Success) {
            std::stringstream ss;
            ss << "Failed to get minimum speed: " << result;
            m_warning_system_ptr->monitorWarningForXseconds(ss.str(), WARNING_DURATION);
            return;
        }

        m_polled_params.min_airspeed = static_cast<int>(std::round(min_speed));
      });
      break;
    default:
      m_warning_system_ptr->monitorWarningForXseconds("Parameter unknown: " + intToString(static_cast<int>(param_enum)), WARNING_DURATION);
      return false;

    }
    return true;
}

bool UAV_Model::setParameterAsync(Parameters param_enum, double value) const
{

    switch (param_enum)
    {
      case Parameters::AIRSPEED_TARGET_CRUISE:
        m_action_ptr->set_target_speed_async(value, [this](mavsdk::Action::Result result) {
          if (result != mavsdk::Action::Result::Success) {
              std::stringstream ss;
              ss << "Failed to set target speed: " << result;
              m_warning_system_ptr->monitorWarningForXseconds(ss.str(), WARNING_DURATION);
              return;
          }
        });
        break;
      case Parameters::AIRSPEED_MAX:
        m_action_ptr->set_maximum_speed_async(value, [this](mavsdk::Action::Result result) {
          if (result != mavsdk::Action::Result::Success) {
              std::stringstream ss;
              ss << "Failed to set maximum speed: " << result;
              m_warning_system_ptr->monitorWarningForXseconds(ss.str(), WARNING_DURATION);
              return;
          }
        });
        break;
      case Parameters::AIRSPEED_MIN:
        m_action_ptr->set_minimum_speed_async(value, [this](mavsdk::Action::Result result) {
          if (result != mavsdk::Action::Result::Success) {
              std::stringstream ss;
              ss << "Failed to set minimum speed: " << result;
              m_warning_system_ptr->monitorWarningForXseconds(ss.str(), WARNING_DURATION);
              return;
          }
        });
        break;  
    default:
      m_warning_system_ptr->monitorWarningForXseconds("Parameter unknown: " + intToString(static_cast<int>(param_enum)), WARNING_DURATION);
      return false;
      break;
    }

    return true;
}

///////////////////////////////////
/////////////  COMMANDS  //////////
///////////////////////////////////

bool UAV_Model::commandReturnToLaunchAsync() const{
  
  m_action_ptr->return_to_launch_async([&, this](mavsdk::Action::Result result) {
    if (result != mavsdk::Action::Result::Success) {
        std::stringstream ss;
        ss << "Return to launch failed: " << result;
        m_warning_system_ptr->monitorWarningForXseconds(ss.str(), WARNING_DURATION);
        return;
    }
  });
  
  return true;

}

bool UAV_Model::commandLoiterAtPos(XYPoint pos, bool holdCurrentAltitude) {
  
  // lat lon 0 0  should not be possible
  if (pos == XYPoint(0, 0)){
    m_current_loiter_coord = XYPoint(m_position.latitude_deg, m_position.longitude_deg);
    m_warning_system_ptr->monitorWarningForXseconds("Received empty loiter pos: Loitering at current position", WARNING_DURATION);
  } else{
    m_current_loiter_coord = pos;
  }

  if(commandGoToLocationXY(m_current_loiter_coord, holdCurrentAltitude)){
    std::stringstream ss;
    ss << "Loitering at (Lat/Long): " << m_current_loiter_coord.x() << "/" << m_current_loiter_coord.y() << "\n";
    reportEventFromCallback(ss.str());
    return true;
  }

  m_warning_system_ptr->monitorWarningForXseconds("Loitering failed", WARNING_DURATION);
  return false;


}


bool UAV_Model::commandAndSetAirSpeed(double speed) const { 
  
  if(commandSpeed(speed, SPEED_TYPE::SPEED_TYPE_AIRSPEED)){ // Fails with airspeed

    setParameterAsync(Parameters::AIRSPEED_TARGET_CRUISE, speed);
    return true;
  };
  return false;
}

bool UAV_Model::commandArmAsync() const{

  m_action_ptr->arm_async([&, this](mavsdk::Action::Result result) {
    // MOOSTrace("Arming result: %d\n", result);
    if (result != mavsdk::Action::Result::Success) {
        std::stringstream ss;
        ss << "Arming failed: " << result << '\n';
        std::cout << ss.str();
        m_warning_system_ptr->monitorWarningForXseconds(ss.str(), WARNING_DURATION);
    }
  }); 

  return true;
}

bool UAV_Model::commandDisarmAsync() const{
  
    m_action_ptr->disarm_async([&, this](mavsdk::Action::Result result) {
      // MOOSTrace("Disarming result: %d\n", result);
      if (result != mavsdk::Action::Result::Success) {
          std::stringstream ss;
          ss << "Disarming failed: " << result << '\n';
          std::cout << ss.str();
          m_warning_system_ptr->monitorWarningForXseconds(ss.str(), WARNING_DURATION);
      }
    });

    return true;
}

// bool UAV_Model::setTargetAltitudeAGL(double altitude) const{
  
//   if(!m_in_air){
//     m_warning_system_ptr->monitorWarningForXseconds("UAV is not in air! Cannot send altitude", WARNING_DURATION);
//     return false;
//   }



//   mavsdk::MavlinkPassthrough::CommandLong command_mode;
//   command_mode.command = MAV_CMD_CONDITION_CHANGE_ALT;
//   command_mode.target_sysid = m_system_ptr->get_system_id();
//   command_mode.target_compid = MAV_COMP_ID_AUTOPILOT1; // assuming first component is autopilot
//   command_mode.param1 = altitude; // altitude in meters
//   command_mode.param2 = MAV_FRAME_GLOBAL_RELATIVE_ALT; 
//   command_mode.param3 = 0; // Not used
//   command_mode.param4 = 0; // 
//   command_mode.param5 = 0; // 
//   command_mode.param6 = 0; // 
//   command_mode.param7 = 0; //

//   // blocking
//   auto result = m_mavPass_ptr->send_command_long(command_mode);

//   if(result != mavsdk::MavlinkPassthrough::Result::Success){
//     std::stringstream ss;
//     ss << "command Set Altitude AGL error: " << result <<  " with altitude " << altitude;
//     m_warning_system_ptr->monitorWarningForXseconds( ss.str(), WARNING_DURATION);
//     return false;
//   }

//   MOOSTraceFromCallback("command Set Altitude AGL succeeded\n");

//   return true;
// }

bool UAV_Model::commandGoToLocationXY(const XYPoint pos, bool holdCurrentAltitudeAGL) {


  float alt_msl = m_position.absolute_altitude_m; 
  double terrain_altitude = m_position.absolute_altitude_m - m_position.relative_altitude_m;
  if(!holdCurrentAltitudeAGL){
    alt_msl = terrain_altitude + m_target_altitudeAGL;
  }
  mavsdk::Telemetry::Position wpt = {pos.x(), pos.y(), alt_msl};



  m_last_sent_altitudeAGL = alt_msl - terrain_altitude;   

  return commandGoToLocation(wpt);
}

bool UAV_Model::commandGoToLocation(const mavsdk::Telemetry::Position& position) const{

  double loiter_direction = 0; // 0 for clockwise, 1 for counter clockwise
  
  // blocking //TODO modify so it is non
  auto res = m_action_ptr->goto_location(position.latitude_deg, position.longitude_deg, position.absolute_altitude_m, loiter_direction);


  if(res != mavsdk::Action::Result::Success){
    std::stringstream ss;
    ss << "goto_location failed: " << res;
    m_warning_system_ptr->monitorWarningForXseconds(ss.str(), WARNING_DURATION);
    return false;
  }
     
  MOOSTraceFromCallback("goto_location succeeded\n");
  
  return true;
}



bool UAV_Model::commandSpeed(double speed_m_s, SPEED_TYPE speed_type) const{
  
  if(!m_in_air){
    m_warning_system_ptr->monitorWarningForXseconds("UAV is not in air! Cannot send speed", WARNING_DURATION);
    return false;
  }

  if(speed_type == SPEED_TYPE::SPEED_TYPE_AIRSPEED){
    // check if speed is within bounds  
    if(speed_m_s < m_polled_params.min_airspeed || speed_m_s > m_polled_params.max_airspeed){
      std::stringstream ss;
      ss << "Speed out of bounds: " << speed_m_s << " min: " << m_polled_params.min_airspeed << " max: " << m_polled_params.max_airspeed;
      m_warning_system_ptr->monitorWarningForXseconds(ss.str(), WARNING_DURATION);
      return false;
    }
  }


  mavsdk::MavlinkPassthrough::CommandLong command_mode;
  command_mode.command = MAV_CMD_DO_CHANGE_SPEED;
  command_mode.target_sysid = m_system_ptr->get_system_id();
  command_mode.target_compid = MAV_COMP_ID_AUTOPILOT1; //system.value()->component_ids().front(); // assuming first component is autopilot
  command_mode.param1 = speed_type;
  command_mode.param2 = speed_m_s;
  command_mode.param3 = -1; // -1 throttle no change
  command_mode.param4 = -1; // Not used
  command_mode.param5 = -1; // 
  command_mode.param6 = -1; // 
  command_mode.param7 = -1; //

  // blocking
  auto result = m_mavPass_ptr->send_command_long(command_mode);

  // auto res = m_action_ptr->set_takeoff_speed(speed_m_s);

  if(result != mavsdk::MavlinkPassthrough::Result::Success){
    std::stringstream ss;
    ss << "command Speed error: " << result <<  " with speed " << speed_m_s << " and type ";
    switch (speed_type)
    {
    case SPEED_TYPE_AIRSPEED:
      ss << "SPEED_TYPE_AIRSPEED";
      break;
    case SPEED_TYPE_GROUNDSPEED:
      ss << "SPEED_TYPE_GROUNDSPEED";
      break;
    default:
      ss << doubleToString(speed_type);
      break;
    }
    m_warning_system_ptr->monitorWarningForXseconds( ss.str(), WARNING_DURATION);
    return false;
  }


  MOOSTraceFromCallback("command Speed succeeded\n");


  return true;
}


void UAV_Model::setHeadingWyptFromHeading(double heading_deg){

    heading_deg = angle360(heading_deg); // make sure heading is within 0-360
    
    static const auto deg_to_rad = [](double degrees) { return degrees * deg2rad;};
    static const auto rad_to_deg = [](double radians) { return radians * rad2deg;};

    // Convert heading to radians
    double heading_rad = deg_to_rad(heading_deg);

    // Get current position in radians
    double lat_rad = deg_to_rad(m_position.latitude_deg);
    double lon_rad = deg_to_rad(m_position.longitude_deg);

    // Calculate new latitude using the bearing formula
    double new_lat_rad = std::asin(
        std::sin(lat_rad) * std::cos(DISTANCE_TO_HEADING_WAYPOINT / EARTH_RADIUS) +
        std::cos(lat_rad) * std::sin(DISTANCE_TO_HEADING_WAYPOINT / EARTH_RADIUS) * std::cos(heading_rad)
    );

    // Calculate new longitude
    double new_lon_rad = lon_rad + std::atan2(
        std::sin(heading_rad) * std::sin(DISTANCE_TO_HEADING_WAYPOINT / EARTH_RADIUS) * std::cos(lat_rad),
        std::cos(DISTANCE_TO_HEADING_WAYPOINT / EARTH_RADIUS) - std::sin(lat_rad) * std::sin(new_lat_rad)
    );

    // Convert the new latitude and longitude back to degrees
    double new_lat_deg = rad_to_deg(new_lat_rad);
    double new_lon_deg = rad_to_deg(new_lon_rad);

    m_heading_waypoint_coord.set_vertex(new_lat_deg, new_lon_deg);
    
}



bool UAV_Model::commandHeadingHold(double heading){
  
  if(!m_in_air){
    m_warning_system_ptr->monitorWarningForXseconds("UAV is not in air! Cannot send heading", WARNING_DURATION);
    return false;
  }

  setHeadingWyptFromHeading(heading);

  // Command the plane to the new location
  return commandGoToLocationXY(m_heading_waypoint_coord);
}



mavsdk::MissionRaw::MissionItem make_mission_item_wp(
    float latitude_deg1e7,
    float longitude_deg1e7,
    int32_t altitude_m,
    float param1,
    MAV_FRAME frame,
    MAV_CMD command,
    float p2,
    float p3)
{
    // WARNING this is done in consideration of CLEAN!! mission
    static uint32_t seq_num = 0;
    mavsdk::MissionRaw::MissionItem new_item{};
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
bool create_missionPlan(std::vector<mavsdk::MissionRaw::MissionItem>& mission_plan, double lat_deg_home, double lon_deg_home){

    // in case of ardupilot we want to set lat lon to waypoint 0
    // Home position (same as original, set to lat/lon home)
    mission_plan.push_back(make_mission_item_wp( // 0
        lat_deg_home, // lat home
        lon_deg_home, // lon home
        100, // alt home
        0,
        MAV_FRAME_GLOBAL_RELATIVE_ALT,
        MAV_CMD_NAV_WAYPOINT));

    // Waypoint 1 
    mission_plan.push_back(make_mission_item_wp( // 1 takeoff
        lat_deg_home + 0.003429, // 382.4 meters north
        lon_deg_home - 0.000534, // 49 meters west
        41.03,
        15,
        MAV_FRAME_GLOBAL_RELATIVE_ALT,
        MAV_CMD_NAV_TAKEOFF));

    // Waypoint 2 
    mission_plan.push_back(make_mission_item_wp( // 2
        lat_deg_home + 0.003677, // 409.2 meters north
        lon_deg_home - 0.003845, // 341.4 meters west
        120.00,
        0,
        MAV_FRAME_GLOBAL_RELATIVE_ALT,
        MAV_CMD_NAV_WAYPOINT));

    // Waypoint 3 
    mission_plan.push_back(make_mission_item_wp( // 3
        lat_deg_home - 0.003201, // 356.8 meters south
        lon_deg_home - 0.002996, // 265.2 meters west
        200.00,
        0,
        MAV_FRAME_GLOBAL_RELATIVE_ALT,
        MAV_CMD_NAV_WAYPOINT));

    // Waypoint 4 
    mission_plan.push_back(make_mission_item_wp( // 4
        lat_deg_home - 0.002869, // 320.4 meters south
        lon_deg_home - 0.000656, // 57.2 meters west
        210.00,
        0,
        MAV_FRAME_GLOBAL_RELATIVE_ALT,
        MAV_CMD_NAV_WAYPOINT));

    // Waypoint 5 
    mission_plan.push_back(make_mission_item_wp( // 5
        lat_deg_home + 0.004198, // 444.2 meters north
        lon_deg_home - 0.001480, // 131.6 meters west
        130.00,
        0,
        MAV_FRAME_GLOBAL_RELATIVE_ALT,
        MAV_CMD_NAV_WAYPOINT));

    // Waypoint 6 (Speed Change)
    mission_plan.push_back(make_mission_item_wp( // 6
        lat_deg_home - 0.002869, // relative to waypoint 4
        lon_deg_home - 0.000656, // relative to waypoint 4
        110.00, 
        SPEED_TYPE_AIRSPEED, 
        MAV_FRAME_GLOBAL_RELATIVE_ALT, 
        MAV_CMD_DO_CHANGE_SPEED, 
        6)); // speed 6 m/s

    // Waypoint 7 (repeat of WP5)
    mission_plan.push_back(make_mission_item_wp( // 7
        lat_deg_home + 0.004198, // same as waypoint 5
        lon_deg_home - 0.001480, 
        100.00,
        0,
        MAV_FRAME_GLOBAL_RELATIVE_ALT,
        MAV_CMD_NAV_WAYPOINT));

    // Waypoint 8 
    mission_plan.push_back(make_mission_item_wp( // 8
        lat_deg_home + 0.002396, // 267.2 meters north
        lon_deg_home - 0.000352, // 31.1 meters west
        41.00,
        0,
        MAV_FRAME_GLOBAL_RELATIVE_ALT,
        MAV_CMD_NAV_WAYPOINT));

    // Landing waypoint (back to home)
    mission_plan.push_back(make_mission_item_wp( // 9
        lat_deg_home, // return to home position
        lon_deg_home,
        0.00,
        1, // Minimum abort altitude
        MAV_FRAME_GLOBAL_RELATIVE_ALT,
        MAV_CMD_NAV_LAND,
        PRECISION_LAND_MODE_OPPORTUNISTIC));
    return true;
}









