/************************************************************/
/*    NAME: Charlie Benjamin                                */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: RefuelReplace.cpp                               */
/*    DATE: January 31st, 2026                              */
/************************************************************/

#include "RefuelReplace.h"

#include <sstream>
#include "MBUtils.h"
#include "ACTable.h"

using namespace std;

// For improvment:
// 1. The x and y waypoint pulished to MISSION_TASK should probably be
// the loiter point not the current x and y,
// 2. Return to home logic when the next plane arrives. Handoff logic

//---------------------------------------------------------
// Constructor()

RefuelReplace::RefuelReplace()
{
  // Inputs
  m_nav_x = 0.0;
  m_nav_y = 0.0;
  m_odometry_dist = 0.0;

  m_got_nav_x = false;
  m_got_nav_y = false;
  m_got_odom  = false;

  // Config
  m_refuel_threshold = 0.0;   // meters; disabled if <= 0

  // State
  m_task_sent = false;
  m_task_id_counter = 0;

  // Task Helper
  m_host_community = "vehicle";
}

//---------------------------------------------------------
// Destructor

RefuelReplace::~RefuelReplace() {}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool RefuelReplace::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  for (auto &msg : NewMail) {
    const string &key = msg.GetKey();

    if (key == "NAV_X") {
      m_nav_x = msg.GetDouble();
      m_got_nav_x = true;
    }
    else if (key == "NAV_Y") {
      m_nav_y = msg.GetDouble();
      m_got_nav_y = true;
    }
    else if (key == "ODOMETRY_DIST") {
      m_odometry_dist = msg.GetDouble();
      m_got_odom = true;
    }
    else if (key != "APPCAST_REQ") {
      reportRunWarning("Unhandled Mail: " + key);
    }
  }

  return true;
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool RefuelReplace::OnConnectToServer()
{
  registerVariables();
  return true;
}

//---------------------------------------------------------
// Procedure: Iterate()

bool RefuelReplace::Iterate()
{
  AppCastingMOOSApp::Iterate();

  // Only act once we have the needed inputs at least once.
  const bool have_nav  = (m_got_nav_x && m_got_nav_y);
  const bool have_odom = m_got_odom;

  // Trigger once when odometry reaches threshold.
  if (!m_task_sent &&
      (m_refuel_threshold > 0.0) &&
      have_nav && have_odom &&
      (m_odometry_dist >= m_refuel_threshold))
  {
    // Build an incrementing id like rr0, rr1, ...
    string id = m_host_community + "_rr" + intToString(m_task_id_counter++);

    // Use current time as utc (MOOSTime is epoch seconds in typical MOOS-IvP builds)
    double utc = MOOSTime();

    // Simple unique hash
    long utc_int  = (long)(MOOSTime() * 100); // centiseconds
    long utc_tail = utc_int % 100000; // wraps every 16 minutes
    string hash = "rr_" + id + "_" + intToString((int)utc_tail);

    // Build MISSION_TASK string (modeled after TaskManager docs)
    // type, id, utc, hash, waypt_x, waypt_y
    ostringstream os;
    os << "type=refuelreplace,"
       << "id=" << id << ","
       << "utc=" << doubleToStringX(utc, 2) << ","
       << "hash=" << hash << ","
       << "waypt_x=" << doubleToStringX(m_nav_x, 2) << ","
       << "waypt_y=" << doubleToStringX(m_nav_y, 2);

    Notify("MISSION_TASK", os.str());

    // Optional: also publish a simple boolean flag for debugging/compatibility
    Notify("TASK_REFUEL_REPLACE", "true");

    m_task_sent = true;
  }
 
  AppCastingMOOSApp::PostReport();
  return true;
}

//---------------------------------------------------------
// Procedure: OnStartUp()

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
    string value = stripBlankEnds(line);

    bool handled = false;

    // Example:
    // refuel_threshold = 150.0
    if (param == "refuel_threshold") {
      handled = setDoubleOnString(m_refuel_threshold, value);
    }
    else if (param == "vname") {
      handled = setNonWhiteVarOnString(m_host_community, value);
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
  Register("ODOMETRY_DIST", 0);
}

//---------------------------------------------------------
// Procedure: buildReport()

bool RefuelReplace::buildReport()
{
  ACTable table(2);
  table << "Field" << "Value";
  table.addHeaderLines();

  table << "refuel_threshold"     << doubleToStringX(m_refuel_threshold, 2);
  table << "ODOMETRY_DIST"        << doubleToStringX(m_odometry_dist, 2);
  table << "NAV_X"                << doubleToStringX(m_nav_x, 2);
  table << "NAV_Y"                << doubleToStringX(m_nav_y, 2);
  table << "got_odom"             << (m_got_odom  ? "true" : "false");
  table << "got_nav_x"            << (m_got_nav_x ? "true" : "false");
  table << "got_nav_y"            << (m_got_nav_y ? "true" : "false");
  table << "task_sent"            << (m_task_sent ? "true" : "false");
  table << "next_task_id_counter" << intToString(m_task_id_counter);

  m_msgs << table.getFormattedString();
  return true;
}
