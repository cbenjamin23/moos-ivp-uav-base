/************************************************************/
/*    NAME: Charlie Benjamin                                */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: Odometry.cpp                                    */
/*    DATE: January 30th, 2026                              */
/************************************************************/

#include "Odometry.h"

#include <cmath>
#include "MBUtils.h"
#include "ACTable.h"

using namespace std;

//---------------------------------------------------------
// Constructor

Odometry::Odometry()
{
  // State
  m_first_reading       = true;
  m_current_x           = 0.0;
  m_current_y           = 0.0;
  m_previous_x          = 0.0;
  m_previous_y          = 0.0;
  m_total_distance      = 0.0;

  // Mail / staleness
  m_lastMailTime        = 0.0;
  m_warning_issued      = false;
  m_staleness_threshold = 0.0;

  // Gating + reset
  m_got_nav_x     = false;
  m_got_nav_y     = false;
  m_reset_pending = false;
}

//---------------------------------------------------------
// Destructor

Odometry::~Odometry() {}

//---------------------------------------------------------
// Procedure: OnNewMail

bool Odometry::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);
  
  for (auto &msg : NewMail) {
    const string &key = msg.GetKey();

    //Helpers
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();

    if (key == "NAV_X") {
      m_current_x = dval;
      m_lastMailTime = MOOSTime();
      m_got_nav_x = true;

      // Initialize previous on first readings so first step isn't a big jump
      if (m_first_reading)
        m_previous_x = m_current_x;
    }
    else if (key == "NAV_Y") {
      m_current_y = dval;
      m_lastMailTime = MOOSTime();
      m_got_nav_y = true;
      
      if (m_first_reading)
        m_previous_y = m_current_y;
    }
    else if (key == "STALEVAR") {
      m_staleness_threshold = dval;
    }
    else if (key == "ODOMETRY_RESET") {
      bool do_reset = false;

      if (msg.IsDouble()) {
	do_reset = (dval != 0.0);
      } else {
	std::string s = tolower(stripBlankEnds(sval));
	do_reset = (s == "true" || s == "1" || s == "reset");
      }

      if (do_reset)
	m_reset_pending = true;
    }
    else if (key != "APPCAST_REQ") {
      reportRunWarning("Unhandled Mail: " + key);
    }
  }

  // Once we've seen both X and Y NAV mail, we are no longer in "first reading" mode.
  if (m_got_nav_x && m_got_nav_y)
    m_first_reading = false;
  
  return true;
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool Odometry::OnConnectToServer()
{
  registerVariables();
  return true;
}

//---------------------------------------------------------
// Procedure: Iterate

bool Odometry::Iterate()
{
  AppCastingMOOSApp::Iterate();

  // ----- Apply reset command (if pending) -----
  if (m_reset_pending) {
    m_total_distance = 0.0;

    // Set previous to current so we don't add a jump right after reset
    m_previous_x = m_current_x;
    m_previous_y = m_current_y;

    // Clear gating flags so we require a fresh pair after reset
    m_got_nav_x = false;
    m_got_nav_y = false;

    m_reset_pending = false;

    Notify("ODOMETRY_DIST", m_total_distance);
    AppCastingMOOSApp::PostReport();
    return true;
  }
  
  // ----- Gating: integrate only when we have a fresh NAV_X and NAV_Y -----
  if (m_got_nav_x && m_got_nav_y && !m_first_reading) {
    const double dx = m_current_x - m_previous_x;
    const double dy = m_current_y - m_previous_y;
    const double step_dist = std::hypot(dx, dy);

    m_total_distance += step_dist;

    // Update previous to current after successful integration
    m_previous_x = m_current_x;
    m_previous_y = m_current_y;

    // Consume the pair
    m_got_nav_x = false;
    m_got_nav_y = false;
  }

  // ----- Staleness check -----
  double time_since_mail = 0.0;
  if (m_lastMailTime > 0.0)
    time_since_mail = MOOSTime() - m_lastMailTime;

  if (m_staleness_threshold > 0.0) {
    if (time_since_mail > m_staleness_threshold && !m_warning_issued) {
      reportRunWarning("NAV data stale");
      m_warning_issued = true;
    }
    else if (time_since_mail <= m_staleness_threshold && m_warning_issued) {
      retractRunWarning("NAV data stale");
      m_warning_issued = false;
    }
  }

  // Publish odometry
  Notify("ODOMETRY_DIST", m_total_distance);

  AppCastingMOOSApp::PostReport();
  return true;
}

//---------------------------------------------------------
// Procedure: OnStartUp

bool Odometry::OnStartUp()
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

    if(!handled)
      reportUnhandledConfigWarning(orig);
	

  }
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables

void Odometry::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
  Register("STALEVAR", 0);
  Register("ODOMETRY_RESET", 0);
}

//---------------------------------------------------------
// Procedure: buildReport

bool Odometry::buildReport()
{
  ACTable table(2);
  table << "Field" << "Value";
  table.addHeaderLines();

  table << "ODOMETRY_DIST" << doubleToStringX(m_total_distance, 3);
  table << "NAV_X"         << doubleToStringX(m_current_x, 3);
  table << "NAV_Y"         << doubleToStringX(m_current_y, 3);
  
  if (m_lastMailTime > 0.0)
    table << "last_mail_age(s)" << doubleToStringX(MOOSTime() - m_lastMailTime, 2);
  else
    table << "last_mail_age(s)" << "n/a";

  table << "staleness_threshold(s)" << doubleToStringX(m_staleness_threshold, 2);
  table << "warning_issued"         << (m_warning_issued ? "true" : "false");

  m_msgs << table.getFormattedString();
  return true;
}
