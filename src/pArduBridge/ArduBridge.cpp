/************************************************************/
/*    NAME: Steve Carter Feujo Nomeny                       */
/*    ORGN: NTNU, MIT                                       */
/*    FILE: ArduBridge.cpp                                  */
/*    DATE: September 9th, 2024                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"

// #include "NodeMessage.h"
#include "XYMarker.h"


#include "ArduBridge.h"
#include "definitions.h"

//---------------------------------------------------------
// Constructor()

ArduBridge::ArduBridge()
: m_do_fly_to_waypoint{false},
  m_do_takeoff{false},
  m_cli_arg{},
  m_do_change_speed_pair{std::make_pair(false, 0)},
  m_do_reset_speed{false},
  m_do_return_to_launch{false},
  m_do_loiter{false},
  m_do_arm{false},
  m_warning_system_ptr{ std::make_shared<WarningSystem>(
    [this](const std::string msg){this->reportRunWarning(msg);}, 
    [this](const std::string msg){this->retractRunWarning(msg);} ) 
  }
{
  m_uav_model.registerWarningSystem(m_warning_system_ptr);
  m_uav_model.setCallbackReportEvent([this](const std::string msg){this->reportEvent(msg);});

  m_uav_prefix = "UAV";
  


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
    std::string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif

    if(key == "FLY_WAYPOINT"){
      setBooleanOnString(m_do_fly_to_waypoint, msg.GetString());
      if(!m_do_fly_to_waypoint) m_warning_system_ptr->monitorWarningForXseconds("Fly waypoint command not set", WARNING_DURATION);
    }
    else if(key == "DO_TAKEOFF"){
      setBooleanOnString(m_do_takeoff, msg.GetString());
      if(!m_do_takeoff) m_warning_system_ptr->monitorWarningForXseconds("Takeoff command not set", WARNING_DURATION);
    }
    else if(key == "CHANGE_SPEED"){
      m_do_change_speed_pair = std::make_pair(true, msg.GetDouble());
    }
    else if(key == "ARM_UAV"){  
      setBooleanOnString(m_do_arm, msg.GetString());
      if(!m_do_arm) m_warning_system_ptr->monitorWarningForXseconds("Arm command not set", WARNING_DURATION);
    }
    else if(key == "RESET_SPEED_MIN"){
      setBooleanOnString(m_do_reset_speed, msg.GetString());
      if(!m_do_reset_speed) m_warning_system_ptr->monitorWarningForXseconds("Reset speed command not set", WARNING_DURATION);
    }
    else if(key == "RETURN_TO_LAUNCH"){
      setBooleanOnString(m_do_return_to_launch, msg.GetString());
      if(!m_do_return_to_launch) m_warning_system_ptr->monitorWarningForXseconds("Return to launch command not set", WARNING_DURATION);
      
    }
    else if(key == "LOITER"){
      setBooleanOnString(m_do_loiter, msg.GetString());
      if(!m_do_loiter) m_warning_system_ptr->monitorWarningForXseconds("Loiter command not set", WARNING_DURATION);
      
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
 
  static bool poll_parameters = true;

  if(m_do_takeoff ){
    // send the takeoff command
    auto start_result = m_uav_model.startMission();
    
    m_do_takeoff = false;
    poll_parameters = true;
  }

  if(m_do_fly_to_waypoint){
    // send the fly to waypoint command
    
    double lat_deg_home = m_uav_model.getHomeLatLong().x();
    double lon_deg_home = m_uav_model.getHomeLatLong().y();
    
    mavsdk::Telemetry::Position dest = {lat_deg_home+0.0011, lon_deg_home+0.0011, 564 + 60};

    auto res = m_uav_model.commandGoToLocation(dest);

    m_do_fly_to_waypoint = false;
    poll_parameters = true;
  }

  if(m_do_arm){
    m_uav_model.sendArmCommandIfHealthyAndNotArmed();
    m_do_arm = false;
  }

  if(m_do_return_to_launch){
    m_uav_model.commandReturnToLaunchAsync();
    m_do_return_to_launch = false;
  }

  if(m_do_loiter){
    if(m_uav_model.commandLoiter()){
      visualizeLoiterLocation();
    }

    m_do_loiter = false;
  }

  if(m_do_change_speed_pair.first){
    double new_speed = m_uav_model.getTargetAirSpeed() + m_do_change_speed_pair.second;
    m_uav_model.commandAndSetAirSpeed(new_speed);
    reportEvent("Trying to changed speed to " + doubleToString(new_speed));
    m_do_change_speed_pair.second = 0;
    m_do_change_speed_pair.first = false;

    poll_parameters = true;
  }


  if(m_do_reset_speed){
    m_uav_model.commandAndSetAirSpeed(m_uav_model.getMinAirSpeed());
    m_do_reset_speed = false;
    poll_parameters = true;
  }

  postTelemetryUpdate(m_uav_prefix);


  m_warning_system_ptr->checkConditions(); // Check for warnings and remove/raise them as needed

  if(poll_parameters){
    m_uav_model.pollAllParametersAsync();
    poll_parameters = false;
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


  std::cout << "Starting up ArduBridge with mission file name: " << GetMissionFileName() << std::endl;
  std::cout << "App name is: " << GetAppName() << std::endl;


  bool is_connected = false;
  std::string ardupilot_url;
  std::pair<bool, std::string> url_protocol_pair{false, ""};

  STRING_LIST sParams;

  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams)){
    std::cout << "No config block found for " << GetAppName() << std::endl;
    reportConfigWarning("No config block found for " + GetAppName());
  }

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    std::string orig  = *p;
    std::string line  = *p;
    std::string param = tolower(biteStringX(line, '='));
    std::string value = line;


    bool handled = false;
    if(param == "ardupiloturl" || param == "url") {
      ardupilot_url = value;
      handled = true;
    }
    else if(param == "prefix"){
      handled = setNonWhiteVarOnString(m_uav_prefix, value);
    }
    else if(param == "url_protocol"){
      if(value == "tcp"){
        url_protocol_pair.first = true;
        url_protocol_pair.second = "tcp://";
      }
      else if(value == "udp"){
        url_protocol_pair.first = true;
        url_protocol_pair.second = "udp://" ;
      }
      else if(value == "serial"){
        url_protocol_pair.first = true;
        url_protocol_pair.second = "serial:///dev/" ;
      }
      
      handled = url_protocol_pair.first;
      std::cout << "URL protocol set to: " << url_protocol_pair.second << std::endl;

    }
    if(!handled)
      reportUnhandledConfigWarning(orig);

  }

  // look for latitude, longitude global variables
  double latOrigin, longOrigin;
  if(!m_MissionReader.GetValue("LatOrigin", latOrigin)) {
    MOOSTrace("pArduBridge: LatOrigin not set in " + GetMissionFileName() + " file.\n");
    
    m_geo_ok = false;
  } 
  else if(!m_MissionReader.GetValue("LongOrigin", longOrigin)) {
    MOOSTrace("pArduBridge: LongOrigin not set in " + GetMissionFileName() + " file\n");
    m_geo_ok = false;      
  }
  else {
    m_geo_ok = true;
    // initialize m_geodesy
    if(!m_geodesy.Initialise(latOrigin, longOrigin)) {
      MOOSTrace("pArduBridge: Geodesy init failed.\n");
      m_geo_ok = false;
    }
  }


  ardupilot_url = url_protocol_pair.second + ardupilot_url;

  std::cout << "ArduPilot URL is: " << ardupilot_url << std::endl;
  
  if (!m_cli_arg.parse(ardupilot_url)) {
    
    if(!url_protocol_pair.first){
      reportConfigWarning("URL protocol not set - Need to restart with a valid URL prefix");
      std::cout << "URL protocol not set - Need to restart with a valid URL prefix" << std::endl;
    }
    else{
      reportConfigWarning("Invalid ArduPilot URL specified - Need to restart with a valid URL");
    }
  }
  else{
    // Connect to autopilot
    is_connected = m_uav_model.connectToUAV(ardupilot_url);

  }

  if(!is_connected){
    std::cout << "Failed to connect to ArduPilot" << std::endl;
    return(false);
  }

  m_uav_model.subscribeToTelemetry();
  m_warning_system_ptr->checkConditions(); // Check for warnings and remove/raise them as needed
  
  
  

  visualizeHomeLocation();

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

  Register("CHANGE_SPEED", 0);
  Register("ARM_UAV", 0);
  Register("RESET_SPEED_MIN", 0);

  Register("RETURN_TO_LAUNCH", 0);
  Register("LOITER", 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool ArduBridge::buildReport() 
{
  m_msgs << "============================================" << std::endl;
  m_msgs << "File: pArduBridge                                      " << std::endl;
  m_msgs << "============================================" << std::endl;


  m_msgs << " -------- Configuration Settings -----------" << std::endl;
  m_msgs << "ArduPilot URL: " << m_cli_arg.get_path() << std::endl;
  m_msgs << "ArduPilot Port: " << m_cli_arg.get_port() << std::endl;
  m_msgs << "ArduPilot Protocol: " << protocol2str.at(m_cli_arg.get_protocol()) << std::endl;

  
  m_msgs << "-------------------------------------------" << std::endl;

  m_msgs << "UAV States: " << std::endl;
  m_msgs << "------------------ " << std::endl;
  m_msgs << "           Is Armed: "  <<  boolToString(m_uav_model.isArmed()) << std::endl;
  m_msgs << "         Is Healthy: "  <<  boolToString(m_uav_model.isHealthy()) << std::endl;
  m_msgs << "             In Air: "  <<  boolToString(m_uav_model.isInAir()) << std::endl;
  m_msgs << "        Flight Mode: "  <<  m_uav_model.getFlightMode() << std::endl;

  double lat = m_uav_model.getLatitude();
  double lon = m_uav_model.getLongitude();
  double nav_x, nav_y;
  if(m_geo_ok) {
    m_geodesy.LatLong2LocalGrid(lat, lon, nav_y, nav_x);
  } 
  
  m_msgs << "State Information: " << std::endl;
  m_msgs << "------------------ " << std::endl;
  m_msgs << "  (Latitude , Longditute): " <<  lat  << " , " << lon << std::endl;
  m_msgs << "                  (X , Y): " << nav_x << " , " << nav_y << std::endl;
  m_msgs << "           Altitude (AGL): " << m_uav_model.getAltitudeAGL() << " m (Depth/Z: " << -m_uav_model.getAltitudeAGL() << " m)" << std::endl;
  m_msgs << "          Target Airspeed: " << m_uav_model.getTargetAirSpeed() <<  " m/s" << std::endl;
  m_msgs << "                 AirSpeed: " << m_uav_model.getAirSpeed() <<  " m/s" <<std::endl;
  m_msgs << " AirSpeed (xy projection): " << m_uav_model.getAirSpeedXY() <<  " m/s" << std::endl;
  m_msgs << "                 Heading: " << m_uav_model.getHeading() <<  " deg" << std::endl;

  m_msgs << "-------------------------------------------" << std::endl;

  ACTable actb(2);
  actb << "Parameters | Value ";
  actb.addHeaderLines();
  actb << "Min AirSpeed:" << m_uav_model.getMinAirSpeed();
  actb << "Max AirSpeed:" << m_uav_model.getMaxAirSpeed();
  actb << "Home Coord Lat:" << m_uav_model.getHomeLatLong().x();
  actb << "Home Coord Lon:" << m_uav_model.getHomeLatLong().y();
  m_msgs << actb.getFormattedString() << std::endl;

  m_msgs << "-------------------------------------------" << std::endl;

  ACTable actab(2);
  actab << "Debug | Value ";
  actab.addHeaderLines();
  actab << "Do set fly waypoint:" << boolToString(m_do_fly_to_waypoint);
  actab << "Do takeoff:" << boolToString(m_do_takeoff);
  m_msgs << actab.getFormattedString();

  return(true);
}


void ArduBridge::postTelemetryUpdate(const std::string& prefix){
  
  double lat = m_uav_model.getLatitude();
  double lon = m_uav_model.getLongitude();

  if(!lat || !lon) {
    m_warning_system_ptr->monitorWarningForXseconds("NAN Values at lat or long", 5);
    return;
  }

  Notify(prefix+"_LAT", lat, m_curr_time);
  Notify(prefix+"_LONG", lon, m_curr_time);
  
  
  if(m_geo_ok) {
    double nav_x, nav_y;
    m_geodesy.LatLong2LocalGrid(lat, lon, nav_y, nav_x);
    
    Notify(prefix+"_X", nav_x, m_curr_time);
    Notify(prefix+"_Y", nav_y, m_curr_time);
  }  

  Notify(prefix+"_SPEED", m_uav_model.getAirSpeed(), m_curr_time);
  
  Notify(prefix+"_ALTITUDE", m_uav_model.getAltitudeAGL(), m_curr_time);
  Notify(prefix+"_DEPTH", -m_uav_model.getAltitudeAGL(), m_curr_time);

  // Notify(prefix+"_Z", -m_uav_model.getDepth(), m_curr_time);


  // Notify(prefix+"_ROLL", m_uav_model.getPitch(), m_curr_time);
  // Notify(prefix+"_PITCH", m_uav_model.getPitch(), m_curr_time);
  Notify(prefix+"_HEADING", m_uav_model.getHeading(), m_curr_time);




  // Notify(prefix+"_HEADING_OVER_GROUND", hog, m_curr_time);
  Notify(prefix+"_SPEED_OVER_GROUND", m_uav_model.getAirSpeedXY(), m_curr_time);
  

}


void ArduBridge::visualizeHomeLocation(){
  double lat = m_uav_model.getHomeLatLong().x();
  double lon = m_uav_model.getHomeLatLong().y();

  if(!lat || !lon) {
    m_warning_system_ptr->monitorWarningForXseconds("NAN Values at lat or long", 5);
    return;
  }
  
  double nav_x, nav_y;
  if(m_geo_ok) {
    m_geodesy.LatLong2LocalGrid(lat, lon, nav_y, nav_x);
  } else {
    m_warning_system_ptr->monitorWarningForXseconds("Geodesy not initialized", 5);
    return;
  }

  XYMarker marker(nav_x, nav_y);
  marker.set_label("Home");
  marker.set_type("gateway");
  marker.set_width(4.5);
  std::string spec = marker.get_spec() + ",color=blue,scale=5";
  Notify("VIEW_MARKER", spec);

 reportEvent("Set marker at home location: " + spec);
}


void ArduBridge::visualizeLoiterLocation(){
  double lat = m_uav_model.getCurrentLoiterLatLong().x();
  double lon = m_uav_model.getCurrentLoiterLatLong().y();

  if(!lat || !lon) {
    m_warning_system_ptr->monitorWarningForXseconds("NAN Values at lat or long", 5);
    return;
  }
  
  double nav_x, nav_y;
  if(m_geo_ok) {
    m_geodesy.LatLong2LocalGrid(lat, lon, nav_y, nav_x);
  } else {
    m_warning_system_ptr->monitorWarningForXseconds("Geodesy not initialized", 5);
    return;
  }

  XYMarker marker(nav_x, nav_y);
  marker.set_label("Loiter point");
  marker.set_type("gateway");
  marker.set_width(4.5);
  std::string spec = marker.get_spec() + ",color=yellow,scale=5";
  Notify("VIEW_MARKER", spec);

 reportEvent("Set marker at loiter location: " + spec);
}