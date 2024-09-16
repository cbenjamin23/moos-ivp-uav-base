/************************************************************/
/*    NAME: Steve Carter Feujo Nomeny                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: ArduBridge.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "ArduBridge.h"



//---------------------------------------------------------
// Constructor()

ArduBridge::ArduBridge()
: m_mavsdk_ptr{std::make_shared<Mavsdk>(Mavsdk::Configuration{Mavsdk::ComponentType::GroundStation})},
  m_do_fly_to_waypoint{false},
  m_do_takeoff{false},
  m_health_all_ok{false}
{


}

//---------------------------------------------------------
// Destructor

ArduBridge::~ArduBridge()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool ArduBridge::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    std::string key    = msg.GetKey();

#if 0 // Keep these around just for template
    std::string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    std::string sval  = msg.GetString(); 
    std::string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif

    if(key == "FLY_WAYPOINT"){
      setBooleanOnString(m_do_fly_to_waypoint, msg.GetString());
    }
    else if(key == "DO_TAKEOFF"){
      setBooleanOnString(m_do_takeoff, msg.GetString());
    }

    else if(key != "APPCAST_REQ"){  // handled by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);
    }
  
  }
	
  return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool ArduBridge::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool ArduBridge::Iterate()
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
            std::stringstream ss;
            ss << "Arming failed: " << result << '\n';
            std::cout << ss.str();
            reportRunWarning("Failed to arm");
            MOOSTrace(ss.str().c_str());
        }
    }); 

  }

  if(m_do_takeoff && is_armed){
    // send the takeoff command
    auto start_result = m_mission_raw_ptr->start_mission();
    if (start_result != MissionRaw::Result::Success) {
        std::cout << "start failed" << std::endl;
        reportRunWarning("Failed to start mission");
    }else{
        std::cout << "Mission started" << std::endl;
        MOOSTrace("Mission started\n");
    }
    m_do_takeoff = false;
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









  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool ArduBridge::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    std::string orig  = *p;
    std::string line  = *p;
    std::string param = tolower(biteStringX(line, '='));
    std::string value = line;

    bool handled = false;
    if(param == "ardupiloturl" || param == "url") {
      m_ArduPilot_url = "udp://" + value;
      handled = true;
    }
    else if(param == "bar") {
      handled = true;
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);

  }

  if(m_ArduPilot_url.empty()){
    reportConfigWarning("No ArduPilot URL specified - Need to restart with a valid URL");
  }
  else{
    // Connect to autopilot
    ConnectToUAV(m_ArduPilot_url);
  }

  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void ArduBridge::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("FLY_WAYPOINT", 0);
  Register("DO_TAKEOFF", 0);
  // Register("FOOBAR", 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool ArduBridge::buildReport() 
{
  m_msgs << "============================================" << std::endl;
  m_msgs << "File: pArduBridge                                      " << std::endl;
  m_msgs << "============================================" << std::endl;

  ACTable actab(2);
  actab << "Setting | Value ";
  actab.addHeaderLines();
  actab << "Do set fly waypoint:" << boolToString(m_do_fly_to_waypoint);
  actab << "Do takeoff:" << boolToString(m_do_takeoff);
  m_msgs << actab.getFormattedString();
  m_msgs << "\n\n" << std::endl;


  return(true);
}


bool ArduBridge::ConnectToUAV(std::string url)
{
  if(url.empty()){
    reportRunWarning("No URL specified");
    return false;
  }
  std::cout << "Connecting to the URL: " << url << std::endl;

  ConnectionResult connection_result = m_mavsdk_ptr->add_any_connection(url);

  std::cout << "Connection result: " << connection_result << std::endl;

  if (connection_result != ConnectionResult::Success) {
    std::stringstream ss;
    ss << "Connection failed: " << connection_result << '\n';
    MOOSTrace(ss.str().c_str());
    std::cout << ss.str();
    reportRunWarning(ss.str());
  }

  std::cout << "Waiting to discover system..." << std::endl;
  auto system = m_mavsdk_ptr->first_autopilot(3.0);
  if (!system.has_value()) {
    std::stringstream ss;
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

  std::stringstream ss;
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
        std::stringstream ss;
        ss << "Failed to upload mission" << std::endl;
        ss << "upload result: " << upload_result << std::endl;
        reportRunWarning(ss.str());
    }

  m_mission_raw_ptr->set_current_mission_item(0);


  m_telemetry_ptr->subscribe_health_all_ok([&, this](bool is_health_all_ok) {
    this->m_health_all_ok = is_health_all_ok;
  });

  return true;
}



MissionRaw::MissionItem make_mission_item_wp(
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
bool create_missionPlan(std::vector<mavsdk::MissionRaw::MissionItem>& mission_plan, float lat_deg_home, float lon_deg_home){

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



