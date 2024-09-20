/************************************************************/
/*    NAME: Steve Carter Feujo Nomeny                       */
/*    ORGN: NTNU, MIT                                       */
/*    FILE: ArduBridge.cpp                                  */
/*    DATE: September 9th, 2024                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "ArduBridge.h"


#include "definitions.h"

//---------------------------------------------------------
// Constructor()

ArduBridge::ArduBridge()
: m_do_fly_to_waypoint{false},
  m_do_takeoff{false},
  m_cli_arg{},
  m_warning_system{
    [this](const std::string msg){this->reportRunWarning(msg);}, 
    [this](const std::string msg){this->retractRunWarning(msg);}, 
    },
  m_uav_model{m_warning_system}
{

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

  m_uav_model.sendArmCommandIfHealthyAndNotArmed();
 

  if(m_do_takeoff ){
    // send the takeoff command
    auto start_result = m_uav_model.startMission();
    
    m_do_takeoff = false;
  }

  if(m_do_fly_to_waypoint){
    // send the fly to waypoint command
    
    double m_lat_deg_home = m_uav_model.getHomeCoord().x();
    double m_lon_deg_home = m_uav_model.getHomeCoord().y();
    
    auto res = m_uav_model.goToLocation(m_lat_deg_home+0.0011,
                         m_lon_deg_home+0.0011,
                         564 + 60,
                         0.0);

    std::cout << "goto_location : " << "lat: " << m_lat_deg_home+0.0011
               << " lon: " << m_lon_deg_home+0.0011
               << " alt: " << 564 + 60
               << " yaw: " << 0.0 << '\n';


    m_do_fly_to_waypoint = false;
  }




  postTelemetryUpdate(m_uav_prefix);


  m_warning_system.checkConditions(); // Check for warnings and remove/raise them as needed

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool ArduBridge::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  std::string ardupilot_url;

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
      ardupilot_url = "udp://" + value;
      handled = true;
    }
    else if(param == "prefix"){
      handled = setNonWhiteVarOnString(m_uav_prefix, value);
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);

  }

  if (!m_cli_arg.parse(ardupilot_url)) {
      reportConfigWarning("Invalid ArduPilot URL specified - Need to restart with a valid URL");
  }
  else{
    // Connect to autopilot
    m_uav_model.connectToUAV(ardupilot_url);

  }

  
  // look for latitude, longitude global variables
  double latOrigin, longOrigin;
  if(!m_MissionReader.GetValue("LatOrigin", latOrigin)) {
    MOOSTrace("pArduBridge: LatOrigin not set in *.moos file.\n");
    m_geo_ok = false;
  } 
  else if(!m_MissionReader.GetValue("LongOrigin", longOrigin)) {
    MOOSTrace("pArduBridge: LongOrigin not set in *.moos file\n");
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
 

  m_uav_model.gatherTelemetry();

  registerVariables();	
  m_warning_system.checkConditions(); // Check for warnings and remove/raise them as needed
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


  m_msgs << "\n\n";

  m_msgs << " -------- Configuration Settings -----------" << std::endl;
  m_msgs << "ArduPilot URL: " << m_cli_arg.get_path() << std::endl;
  m_msgs << "ArduPilot Port: " << m_cli_arg.get_port() << std::endl;
  m_msgs << "ArduPilot Protocol: " << protocol2str.at(m_cli_arg.get_protocol()) << std::endl;

  
  m_msgs << "-------------------------------------------" << std::endl;
  
  m_msgs << "\n\n";


  m_msgs << " -------- UAV States -----------" << std::endl;

  // ACTable actab1(2);
  // actab1 << " x  | Value ";


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
  m_msgs << "           Altitude (AGL): " << m_uav_model.getAltitudeAGL() << std::endl;
  m_msgs << "                 AirSpeed: " << m_uav_model.getSpeed() << std::endl;
  m_msgs << " AirSpeed (xy projection): " << m_uav_model.getSpeedXY() << std::endl;

  m_msgs << "\n\n";
  m_msgs << "-------------------------------------------" << std::endl;
  m_msgs << "\n\n";


  ACTable actab(2);
  actab << "Setting | Value ";
  actab.addHeaderLines();
  actab << "Do set fly waypoint:" << boolToString(m_do_fly_to_waypoint);
  actab << "Do takeoff:" << boolToString(m_do_takeoff);
  m_msgs << actab.getFormattedString();
  m_msgs << "\n\n" << std::endl;

  m_msgs << "Home Coord (lat, lon): (" << m_uav_model.getHomeCoord().x() << " , " << m_uav_model.getHomeCoord().y() << ")\n";

  return(true);
}


void ArduBridge::postTelemetryUpdate(const std::string& prefix){

  
  double lat = m_uav_model.getLatitude();
  double lon = m_uav_model.getLongitude();

  if(!lat || !lon) {
    m_warning_system.monitorWarningForXseconds("NAN Values at lat or long", 5);
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

  Notify(prefix+"_SPEED", m_uav_model.getSpeed(), m_curr_time);
  // Notify(prefix+"_DEPTH", m_uav_model.getDepth(), m_curr_time);
  Notify(prefix+"_ALTITUDE", m_uav_model.getAltitudeAGL(), m_curr_time);

  // Notify(prefix+"_Z", -m_uav_model.getDepth(), m_curr_time);


  // Notify(prefix+"_ROLL", m_uav_model.getPitch(), m_curr_time);
  // Notify(prefix+"_PITCH", m_uav_model.getPitch(), m_curr_time);
  Notify(prefix+"_HEADING", m_uav_model.getHeading(), m_curr_time);




  // Notify(prefix+"_HEADING_OVER_GROUND", hog, m_curr_time);
  Notify(prefix+"_SPEED_OVER_GROUND", m_uav_model.getSpeedXY(), m_curr_time);
  

}