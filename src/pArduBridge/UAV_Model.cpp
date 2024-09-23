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

#include "thread"

#include "definitions.h"


//------------------------------------------------------------------------
// Constructor

UAV_Model::UAV_Model(WarningSystem &ws):
  m_mavsdk_ptr{std::make_shared<mavsdk::Mavsdk>(mavsdk::Mavsdk::Configuration{mavsdk::Mavsdk::ComponentType::GroundStation})},

  callbackMOOSTrace{nullptr},
  callbackReportRunW{nullptr},
  callbackRetractRunW{nullptr},
  m_warning_system{ws},

  m_health_all_ok{false},
  m_is_armed{false},
  m_in_air{false}
{
  // Initalize the configuration variables


  // Initalize the state variables
  m_position = mavsdk::Telemetry::Position();
  m_attitude_ned = mavsdk::Telemetry::EulerAngle();
  m_velocity_ned = mavsdk::Telemetry::VelocityNed();
  m_battery = mavsdk::Telemetry::Battery();
  m_flight_mode = mavsdk::Telemetry::FlightMode::Unknown;
  m_home_coord = XYPoint(0, 0);

  m_target_airspeed_cruise = 0;
  m_min_airspeed = 0;
  m_max_airspeed = 0;

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

  mavsdk::ConnectionResult connection_result = m_mavsdk_ptr->add_any_connection(url);

  if (connection_result != mavsdk::ConnectionResult::Success) {
    std::stringstream ss;
    ss << "Connection failed: " << connection_result << '\n';
    MOOSTraceFromCallback(ss.str().c_str());

    m_warning_system.monitorWarningForXseconds("Failed to connect to ArduPilot", WARNING_DURATION);

    return false;
  }



  MOOSTraceFromCallback("Connecting to the URL: " + url + "\n");
  MOOSTraceFromCallback("Waiting to discover system...\n");

  auto system = m_mavsdk_ptr->first_autopilot(3.0);
  if (!system.has_value()) {
    m_warning_system.monitorWarningForXseconds(WARNING_TIMED_OUT, WARNING_DURATION);
  }

  m_system_ptr = system.value();

  // m_mission_raw = MissionRaw{system.value()};
  m_mission_raw_ptr = std::make_unique<mavsdk::MissionRaw>(m_system_ptr);
  m_action_ptr = std::make_unique<mavsdk::Action>(m_system_ptr);
  m_telemetry_ptr = std::make_unique<mavsdk::Telemetry>(m_system_ptr);
  m_mavPass_ptr = std::make_unique<mavsdk::MavlinkPassthrough>(m_system_ptr);
  m_param_ptr = std::make_unique<mavsdk::Param>(m_system_ptr);

  auto clear_result = m_mission_raw_ptr->clear_mission();
  if (clear_result != mavsdk::MissionRaw::Result::Success) {
      m_warning_system.monitorWarningForXseconds("Failed to clear mission", WARNING_DURATION);
  }

  auto download_result = m_mission_raw_ptr->download_mission();
  if (download_result.first != mavsdk::MissionRaw::Result::Success) {
      m_warning_system.monitorWarningForXseconds("Failed to download mission", WARNING_DURATION);
  }

  // first point in case of ardupilot is always home
  auto mission_plan = download_result.second;

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
    m_warning_system.monitorWarningForXseconds("Home point is not in global frame, but in frame" + intToString(home_point.frame) , WARNING_DURATION);
  }


  std::cout << "Home point: " << m_home_coord.x() << ", " << m_home_coord.y()  << " , " << home_point.z << std::endl;

  create_missionPlan(mission_plan,  m_home_coord.x(), m_home_coord.y());

  auto upload_result = m_mission_raw_ptr->upload_mission(mission_plan);
    if (upload_result != mavsdk::MissionRaw::Result::Success) {
        m_warning_system.monitorWarningForXseconds("Mission upload failed", WARNING_DURATION);
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

    m_warning_system.monitorCondition( WARNING_UAV_NOT_ARMED,
                                      [&](){return !m_is_armed;}
                                      );   
    return false;
  }

  auto start_result = m_mission_raw_ptr->start_mission();

  if (start_result != mavsdk::MissionRaw::Result::Success) {
      m_warning_system.monitorWarningForXseconds("Failed to start mission", WARNING_DURATION);
      return false;
  }
  
  MOOSTraceFromCallback("Mission started\n");
  return true;
  
}






bool UAV_Model::sendArmCommandIfHealthyAndNotArmed() const{
  
  if(m_health_all_ok && !m_is_armed){
    commandArm();
    return true;
  }

  m_warning_system.monitorWarningForXseconds("UAV is not healthy or is already armed", WARNING_DURATION);
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

  m_telemetry_ptr->subscribe_in_air([&](bool in_air) {
    m_in_air = in_air;
  });


  // start a thread that get param every 5 seconds
  // std::thread get_param_thread([&, this](){
  //   while(true){
  //     std::this_thread::sleep_for(std::chrono::seconds(5));
  //     auto param_result = m_param_ptr->get_param_int("AIRSPEED_CRUISE");
  //     if(param_result.first == mavsdk::Param::Result::Success){
  //       m_target_airspeed_cruise = param_result.second;
  //       std::cout << "Arming require: " << param_result.second << std::endl;
  //     }else{
  //       this->m_warning_system.monitorWarningForXseconds("Failed to get param AIRSPEED_CRUISE", WARNING_DURATION);
  //     }
  //   }
  // });


  return true;
}

bool UAV_Model::getParameter(Parameters param_enum)
{
  // Check if the parameter exists in the string-to-enum map
    auto it = paramEnum2string.find(param_enum);
    if (it == paramEnum2string.end()) {
        m_warning_system.monitorWarningForXseconds("Parameter unknown: " + intToString(static_cast<int>(param_enum)), WARNING_DURATION);
        return false;
    }

    std::string param_name = it->second;
    mavsdk::Param::Result result;
    double param_value = 0.0;  // Store the retrieved value (int or float as double)

    // Determine whether to retrieve an integer or float parameter
    if (param_enum == Parameters::AIRSPEED_CRUISE) {
        auto res = m_param_ptr->get_param_float(param_name);
        result = res.first;
        param_value = res.second;  // Float value
    } else {
        auto res = m_param_ptr->get_param_int(param_name);
        result = res.first;
        param_value = static_cast<double>(res.second);  // Int value, cast to double
    }

    // Check if the result was successful
    if (result != mavsdk::Param::Result::Success) {
        std::stringstream ss;
        ss << "Parameter retrieval failed: " << param_name << "):" << result << '\n';
        m_warning_system.monitorWarningForXseconds(ss.str() , WARNING_DURATION);
        return false;
    }

    // Assign the value to the appropriate member based on the parameter
    switch (param_enum) {
        case Parameters::AIRSPEED_MIN:
            m_min_airspeed = param_value;
            break;
        case Parameters::AIRSPEED_MAX:
            m_max_airspeed = param_value;
            break;
        case Parameters::AIRSPEED_CRUISE:
            m_target_airspeed_cruise = param_value;
            break;
        default:
            m_warning_system.monitorWarningForXseconds("Unknown parameter", WARNING_DURATION);
            return false;
    }

    return true;

}

bool UAV_Model::commandSetParameter(Parameters param_enum, double value) const
{
  // Check if the parameter exists in the string-to-enum map
    auto it = paramEnum2string.find(param_enum);
    if (it == paramEnum2string.end()) {
        m_warning_system.monitorWarningForXseconds("Parameter unknown: " + intToString(static_cast<int>(param_enum)), WARNING_DURATION);
        return false;
    }

    std::string param_name = it->second;
    mavsdk::Param::Result result;

    // Determine whether to set an integer or float parameter with a switch statement
    
    if (param_enum == Parameters::AIRSPEED_CRUISE) {
        result = m_param_ptr->set_param_float(param_name, value);
    } else {
        result = m_param_ptr->set_param_int(param_name, static_cast<int>(value));
    }


    // Check if the result was successful
    if (result != mavsdk::Param::Result::Success) {
        std::stringstream ss;
        ss << "Parameter setting failed (" << param_name << "):" << result << '\n';
        m_warning_system.monitorWarningForXseconds( ss.str(), WARNING_DURATION);
        return false;
    }

    return true;
}

///////////////////////////////////
/////////////  COMMANDS  //////////
///////////////////////////////////

bool   UAV_Model::commandAndSetAirSpeed(double speed) const {
  
  if(commandSpeed(speed, SPEED_TYPE::SPEED_TYPE_AIRSPEED)){

    return commandSetParameter(Parameters::AIRSPEED_CRUISE, speed);

  };
  return false;
}

bool UAV_Model::commandArm() const{

  m_action_ptr->arm_async([&, this](mavsdk::Action::Result result) {
    // MOOSTrace("Arming result: %d\n", result);
    if (result != mavsdk::Action::Result::Success) {
        std::stringstream ss;
        ss << "Arming failed: " << result << '\n';
        std::cout << ss.str();
        m_warning_system.monitorWarningForXseconds(ss.str(), WARNING_DURATION);
    }
  }); 

  return true;
}

bool UAV_Model::commandDisarm() const{
  
    m_action_ptr->disarm_async([&, this](mavsdk::Action::Result result) {
      // MOOSTrace("Disarming result: %d\n", result);
      if (result != mavsdk::Action::Result::Success) {
          std::stringstream ss;
          ss << "Disarming failed: " << result << '\n';
          std::cout << ss.str();
          m_warning_system.monitorWarningForXseconds(ss.str(), WARNING_DURATION);
      }
    });

    return true;
}

bool UAV_Model::commandGoToLocation(mavsdk::Telemetry::Position& position) const{

  double loiter_direction = 0; // 0 for clockwise, 1 for counter clockwise
  
  // blocking //TODO modify so it is non
  auto res = m_action_ptr->goto_location(position.latitude_deg, position.longitude_deg, position.absolute_altitude_m, loiter_direction);


  if(res != mavsdk::Action::Result::Success){
    m_warning_system.monitorWarningForXseconds("goto_location failed", WARNING_DURATION);
    return false;
  }

  MOOSTraceFromCallback("goto_location succeeded\n");
  
  return true;
}

bool UAV_Model::commandSpeed(double speed_m_s, SPEED_TYPE speed_type) const{
  
  if(!m_in_air){
    m_warning_system.monitorWarningForXseconds("UAV is not in air! Cannot send speed", WARNING_DURATION);
    return false;
  }

  mavsdk::MavlinkPassthrough::CommandLong command_mode;
  command_mode.command = MAV_CMD_DO_CHANGE_SPEED;
  command_mode.target_sysid = m_system_ptr->get_system_id();
  command_mode.target_compid = MAV_COMP_ID_AUTOPILOT1; //system.value()->component_ids().front(); // assuming first component is autopilot
  command_mode.param1 = speed_type;
  command_mode.param2 = speed_m_s;
  command_mode.param3 = -1; // -1 throttle no change

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
    m_warning_system.monitorWarningForXseconds( ss.str(), WARNING_DURATION);
    return false;
  }


  MOOSTraceFromCallback("command Speed succeeded\n");


  return true;
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









