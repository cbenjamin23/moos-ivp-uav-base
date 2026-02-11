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

  // Region this vehicle covers
  m_region_x = 0;
  m_region_y = 0;
  m_region_set = false;
  m_priority_weight = 1.0;

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
    else if (key == OWN_REGION_WEIGHT) {
      m_priority_weight = msg.GetDouble();
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
//
// When odometry reaches the refuel threshold, post a
// MISSION_TASK of type=refuelreplace.  The task carries:
//   - region_x, region_y : the loiter point that needs coverage
//   - priority_weight    : how important this region is
//   - exempt             : this vehicle's name (don't bid on own task)

bool RefuelReplace::Iterate()
{
  AppCastingMOOSApp::Iterate();

  const bool have_nav  = (m_got_nav_x && m_got_nav_y);
  const bool have_odom = m_got_odom;

  if (!m_task_sent &&
      (m_refuel_threshold > 0.0) &&
      have_nav && have_odom &&
      (m_odometry_dist >= m_refuel_threshold))
  {
    string id = m_host_community + "_rr" + intToString(m_task_id_counter++);

    double utc = MOOSTime();

    long utc_int  = (long)(MOOSTime() * 100);
    long utc_tail = utc_int % 100000;
    string hash = "rr_" + id + "_" + intToString((int)utc_tail);

    // Use configured region point; fall back to current position
    // if no region was configured (matches your TODO note #1).
    double rx = m_region_set ? m_region_x : m_nav_x;
    double ry = m_region_set ? m_region_y : m_nav_y;

    ostringstream os;
    os << "type=refuelreplace,"
       << "id=" << id << ","
       << "utc=" << doubleToStringX(utc, 2) << ","
       << "hash=" << hash << ","
       << "exempt=" << m_host_community << ","
       << "region_x=" << doubleToStringX(rx, 2) << ","
       << "region_y=" << doubleToStringX(ry, 2) << ","
       << "priority_weight=" << doubleToStringX(m_priority_weight, 2);

    Notify("MISSION_TASK", os.str());

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

    if (param == "refuel_threshold") {
      handled = setDoubleOnString(m_refuel_threshold, value);
    }
    else if (param == "vname") {
      handled = setNonWhiteVarOnString(m_host_community, value);
    }
    else if (param == "region_x") {
      handled = setDoubleOnString(m_region_x, value);
      if(handled) m_region_set = true;
    }
    else if (param == "region_y") {
      handled = setDoubleOnString(m_region_y, value);
      if(handled) m_region_set = true;
    }
    else if (param == "priority_weight") {
      handled = setNonNegDoubleOnString(m_priority_weight, value);
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
  table << "region_x"            << doubleToStringX(m_region_x, 2);
  table << "region_y"            << doubleToStringX(m_region_y, 2);
  table << "priority_weight"     << doubleToStringX(m_priority_weight, 2);
  table << "region_set"          << (m_region_set ? "true" : "false");
  table << "got_odom"             << (m_got_odom  ? "true" : "false");
  table << "got_nav_x"            << (m_got_nav_x ? "true" : "false");
  table << "got_nav_y"            << (m_got_nav_y ? "true" : "false");
  table << "task_sent"            << (m_task_sent ? "true" : "false");
  table << "next_task_id_counter" << intToString(m_task_id_counter);

  m_msgs << table.getFormattedString();
  return true;
}