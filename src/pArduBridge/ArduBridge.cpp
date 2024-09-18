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



//---------------------------------------------------------
// Constructor()

ArduBridge::ArduBridge()
: m_do_fly_to_waypoint{false},
  m_do_takeoff{false},
  m_cli_arg{}
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

  m_uav_model.sendArmCommandIfHealthyAndNotArmed();
 

  if(m_do_takeoff ){
    // send the takeoff command
    auto start_result = m_uav_model.startMission([this](std::string msg){this->reportRunWarning(msg);});
    // if (start_result != mavsdk::MissionRaw::Result::Success) {
    //     std::cout << "start failed" << std::endl;
    //     reportRunWarning("Failed to start mission");
    // }else{
    //     std::cout << "Mission started" << std::endl;
    //     MOOSTrace("Mission started\n");
    // }
    m_do_takeoff = false;
  }

  if(m_do_fly_to_waypoint){
    // send the fly to waypoint command

    
    auto res = m_uav_model.goToLocation(m_lat_deg_home+0.0011,
                         m_lon_deg_home+0.0011,
                         564 + 60,
                         0.0,
                         [this](std::string msg){this->reportRunWarning(msg);});

    // if(res != mavsdk::Action::Result::Success){
    //     std::cerr << "goto_location failed: " << res << '\n';
    //     reportRunWarning("goto_location failed");
    // }else{
    //     std::cout << "goto_location succeeded" << '\n';
    //     MOOSTrace("goto_location succeeded\n");
    // }
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
    else if(param == "bar") {
      handled = true;
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);

  }

  if (!m_cli_arg.parse(ardupilot_url)) {
      reportConfigWarning("Invalid ArduPilot URL specified - Need to restart with a valid URL");
  }
  else{
    // Connect to autopilot
    m_uav_model.connectToUAV(m_cli_arg.get_path(), [this](std::string msg){this->reportRunWarning(msg);});
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


  m_msgs << " -------- Configuration Settings -----------" << std::endl;
  m_msgs << "ArduPilot URL: " << m_cli_arg.get_path() << std::endl;
  
  
  m_msgs << "-------------------------------------------" << std::endl;
  

  ACTable actab(2);
  actab << "Setting | Value ";
  actab.addHeaderLines();
  actab << "Do set fly waypoint:" << boolToString(m_do_fly_to_waypoint);
  actab << "Do takeoff:" << boolToString(m_do_takeoff);
  m_msgs << actab.getFormattedString();
  m_msgs << "\n\n" << std::endl;


  return(true);
}

