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

UAV_Model::UAV_Model(std::shared_ptr<WarningSystem> ws):
  m_mavsdk_ptr{std::make_shared<mavsdk::Mavsdk>(mavsdk::Mavsdk::Configuration{mavsdk::Mavsdk::ComponentType::GroundStation})},

  callbackMOOSTrace{nullptr},
  callbackReportRunW{nullptr},
  callbackRetractRunW{nullptr},
  callbackReportEvent{nullptr},

  m_health_all_ok{false},
  m_is_armed{false},
  m_in_air{false}
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
  m_next_waypoint = XYPoint(0, 0);

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
      m_warning_system_ptr->monitorWarningForXseconds("Failed to clear mission", WARNING_DURATION);
  }

  auto download_result = m_mission_raw_ptr->download_mission();
  if (download_result.first != mavsdk::MissionRaw::Result::Success) {
      m_warning_system_ptr->monitorWarningForXseconds("Failed to download mission", WARNING_DURATION);
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

bool UAV_Model::commandLoiter() {
  
  m_current_loiter_coord = XYPoint(m_position.latitude_deg, m_position.longitude_deg);

  mavsdk::Telemetry::Position loiter_position;
  loiter_position.latitude_deg = m_current_loiter_coord.x();
  loiter_position.longitude_deg = m_current_loiter_coord.y();
  loiter_position.absolute_altitude_m = m_position.absolute_altitude_m;

  if(commandGoToLocation(loiter_position)){
    std::stringstream ss;
    ss << "Loitering at (Lat/Long): " << m_current_loiter_coord.x() << "/" << m_current_loiter_coord.y() << "\n";
    reportEventFromCallback(ss.str());
    return true;
  }

  m_warning_system_ptr->monitorWarningForXseconds("Loitering failed", WARNING_DURATION);
  return false;


}


bool UAV_Model::commandAndSetAirSpeed(double speed) const { 
  
  if(commandSpeed(speed, SPEED_TYPE::SPEED_TYPE_GROUNDSPEED)){ // Fails with airspeed

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
    m_warning_system_ptr->monitorWarningForXseconds( ss.str(), WARNING_DURATION);
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









