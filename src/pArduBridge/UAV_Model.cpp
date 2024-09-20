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
  m_is_armed{false}
{
  // Initalize the configuration variables


  // Initalize the state variables
  m_rudder       = 0;


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

  // m_mission_raw = MissionRaw{system.value()};
  m_mission_raw_ptr = std::make_unique<mavsdk::MissionRaw>(system.value());
  m_action_ptr = std::make_unique<mavsdk::Action>(system.value());
  m_telemetry_ptr = std::make_unique<mavsdk::Telemetry>(system.value());


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

  std::cout << "Home point: " << m_home_coord.x() << ", " << m_home_coord.y() << std::endl;

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


  m_telemetry_ptr->subscribe_health_all_ok([&, this](bool is_health_all_ok) {
    this->m_health_all_ok = is_health_all_ok;
  });

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


bool UAV_Model::goToLocation(double latitude_deg, double longitude_deg, float absolute_altitude_m, float yaw_deg) const{

    // blocking //TODO modify so it is non
  auto res = m_action_ptr->goto_location(latitude_deg, longitude_deg, absolute_altitude_m, yaw_deg);


  if(res != mavsdk::Action::Result::Success){
    m_warning_system.monitorWarningForXseconds("goto_location failed", WARNING_DURATION);
    return false;
  }

  MOOSTraceFromCallback("goto_location succeeded\n");
  
  return true;
}


bool UAV_Model::sendArmCommandIfHealthyAndNotArmed(){
  
  if(m_health_all_ok && !m_is_armed){
    
    m_action_ptr->arm_async([&, this](mavsdk::Action::Result result) {
        // MOOSTrace("Arming result: %d\n", result);
        if (result == mavsdk::Action::Result::Success) {
            m_is_armed = true;
        } else {
            std::stringstream ss;
            ss << "Arming failed: " << result << '\n';
            std::cout << ss.str();
            m_warning_system.monitorWarningForXseconds(ss.str(), WARNING_DURATION);
        }
    }); 

    return true;
  }
  return false;
}



bool UAV_Model::gatherTelemetry(){

  m_telemetry_ptr->subscribe_armed([&](bool is_armed) {
    m_is_armed = is_armed;
  });


  m_telemetry_ptr->subscribe_position([&](mavsdk::Telemetry::Position position) {
    m_position = position;
  });

  m_telemetry_ptr->subscribe_attitude_euler([&](mavsdk::Telemetry::EulerAngle attitude_ned) {
    m_attitude_ned = attitude_ned;
  });


  m_telemetry_ptr->subscribe_velocity_ned([&](mavsdk::Telemetry::VelocityNed ground_speed) {
    m_velocity_ned = ground_speed;
  });


  m_telemetry_ptr->subscribe_battery([&](mavsdk::Telemetry::Battery battery) {
    m_battery = battery;
  });


  return true;


}



// //------------------------------------------------------------------------
// // Procedure: setParam

// bool UAV_Model::setParam(std::string param, double value)
// {
//   param = stripBlankEnds(tolower(param));
//   if(param == "start_x") {
//     m_record.setX(value);
//   }
//   else if(param == "start_y") {
//     m_record.setY(value);
//   }
//   else if(param == "start_heading") {
//     m_record.setHeading(value);
//   }
//   else if(param == "start_speed") {
//     m_record.setSpeed(value);
//   }
//   else if(param == "start_depth") {
//     m_record.setDepth(value);
//     if(value < 0) {
//       m_record.setDepth(0);
//       return(false);
//     }
//   }
//   else
//     return(false);
//   return(true);
// }




// //------------------------------------------------------------------------
// // Procedure: initPosition
// //
// //  "x=20, y=-35, speed=2.2, heading=180, depth=20"


// bool UAV_Model::initPosition(const std::string& str)
// {
//   std::vector<std::string> svector = parseString(str, ',');
//   unsigned int i, vsize = svector.size();
//   for(i=0; i<vsize; i++) {
//     svector[i] = tolower(stripBlankEnds(svector[i]));
//    std::string param = biteStringX(svector[i], '=');
//    std::string value = svector[i];

//     // Support older style spec "5,10,180,2.0,0" - x,y,hdg,spd,dep
//     if(value == "") {
//       value = param;
//       if(i==0)      param = "x";
//       else if(i==1) param = "y";
//       else if(i==2) param = "heading";
//       else if(i==3) param = "speed";
//       else if(i==4) param = "depth";
//     }

//     double dval  = atof(value.c_str());
//     if(param == "x") {
//       m_record.setX(dval);
//     }
//     else if(param == "y") {
//       m_record.setY(dval);
//     }
//     else if((param == "heading") || (param=="deg") || (param=="hdg")) {
//       m_record.setHeading(dval);
//     }
//     else if((param == "speed") || (param == "spd")) {
//       m_record.setSpeed(dval);
//     }
//     else if((param == "depth") || (param == "dep")) {
//       m_record.setDepth(dval);
//     }
//     else
//       return(false);
//   }
//   return(true);
// }







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









