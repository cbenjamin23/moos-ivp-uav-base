/************************************************************/
/*    NAME: Charlie Benjamin                                */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: RefuelReplace.cpp                               */
/*    DATE: January 29th, 2026                              */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "RefuelReplace.h"
#include <math.h>
#include <iostream>

using namespace std;

//---------------------------------------------------------
// Constructor()

RefuelReplace::RefuelReplace()
{
  // NOTE: These are member variables (declared in Odometry.h).
  // Initialize them here (do NOT redeclare as locals).
  m_first_reading        = true;
  m_current_x            = 0.0;
  m_current_y            = 0.0;
  m_previous_x           = 0.0;
  m_previous_y           = 0.0;
  m_total_distance       = 0.0;
  m_lastMailTime         = 0.0;
  m_warning_issued       = false;
  m_staleness_threshold  = 0.0;

  // Distance (meters) at which we trigger a fuel-replacement task.
  // Disabled if <= 0.
  m_replacement_fuel_level = 0.0;
  m_task_refuel_replace    = false;
}

//---------------------------------------------------------
// Destructor

RefuelReplace::~RefuelReplace()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool RefuelReplace::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();
    
    if(key == "NAV_X"){
      m_current_x = msg.GetDouble();
      m_lastMailTime = MOOSTime();
      if(m_first_reading) {
        m_previous_x = m_current_x;
      }
    }
    else if(key == "NAV_Y"){
      m_current_y = msg.GetDouble();
      m_lastMailTime = MOOSTime();
      if(m_first_reading) {
        m_previous_y = m_current_y;
      }
    }
    else if(key == "STALEVAR") {
      m_staleness_threshold = msg.GetDouble();
    }
    else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);

    // Set false after first reading
    m_first_reading = false;

#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif
  }
  
  return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool RefuelReplace::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool RefuelReplace::Iterate()
{
  AppCastingMOOSApp::Iterate();
  // --- Integrate distance ---
  const double added_distance = hypot(m_current_x - m_previous_x,
                                      m_current_y - m_previous_y);
  m_total_distance += added_distance;

  //----------------------------------------------------------
  // --- Staleness watchdog ---
  double timeSinceLastMail = 0.0;
  if(m_lastMailTime > 0)
    timeSinceLastMail = MOOSTime() - m_lastMailTime;

  // If staleness threshold is <= 0, treat the watchdog as disabled.
  if(m_staleness_threshold > 0.0 && timeSinceLastMail > m_staleness_threshold) {
    if(!m_warning_issued) {
      reportRunWarning("No NAV_X/NAV_Y mail for > " + doubleToStringX(m_staleness_threshold, 2) + "s");
      m_warning_issued = true;
    }
  } else if(m_warning_issued) {
    retractRunWarning("No NAV_X/NAV_Y mail for > " + doubleToStringX(m_staleness_threshold, 2) + "s");
    m_warning_issued = false;
  }
  //----------------------------------------------------------

  // --- Trigger TASK_REFUEL_REPLACE by distance ---
  // replacement_fuel_level is configured in OnStartUp.
  if(!m_task_refuel_replace && (m_replacement_fuel_level > 0.0) &&
     (m_total_distance >= m_replacement_fuel_level)) {
    m_task_refuel_replace = true;
  }

  // --- Publish outputs ---
  m_previous_x = m_current_x;
  m_previous_y = m_current_y;
  Notify("ODOMETRY_DIST", m_total_distance);

  // Publish as a boolean-like MOOS variable: "true" / "false".
  Notify("TASK_REFUEL_REPLACE", m_task_refuel_replace ? "true" : "false");

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool RefuelReplace::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    if(param == "stalenessthreshold") {
      handled = setDoubleOnString(m_staleness_threshold, value);
    }
    else if(param == "replacement_fuel_level") {
      handled = setDoubleOnString(m_replacement_fuel_level, value);
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);
	

  }
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void RefuelReplace::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
  Register("STALEVAR", 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool RefuelReplace::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "dist: "  <<  m_total_distance  << endl;
  m_msgs << "============================================" << endl;
  m_msgs << "replacement_fuel_level: " << m_replacement_fuel_level << endl;
  m_msgs << "TASK_REFUEL_REPLACE: " << (m_task_refuel_replace ? "true" : "false") << endl;
  m_msgs << "============================================" << endl;

  return(true);
}




