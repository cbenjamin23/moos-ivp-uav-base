/************************************************************/
/*    NAME: Charlie Benjamin                                */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: RefuelReplace.cpp                               */
/*    DATE: January 31st, 2026                              */
/************************************************************/

#include "RefuelReplace.h"

#include <cstdlib>
#include <cmath>
#include <sstream>
#include "MBUtils.h"
#include "ACTable.h"
#include "NodeMessage.h"

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
  m_total_range      = 0.0;   // meters; total range on full fuel
  m_handoff_radius   = 50.0;  // meters

  // Region this vehicle covers
  m_region_x = 0;
  m_region_y = 0;
  m_region_set = false;
  m_priority_weight = 1.0;

  // State
  m_task_sent = false;
  m_task_id_counter = 0;

  // Task Helper
  // This should be set to vname by param
  m_host_community = "vehicle";

  // fuel dist
  m_fuel_distance_remaining = 0.0;
}

//---------------------------------------------------------
// Destructor

RefuelReplace::~RefuelReplace() {}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool RefuelReplace::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  auto readNumeric = [&](const CMOOSMsg& msg, double& out) -> bool {
    if(msg.IsDouble()) {
      out = msg.GetDouble();
      return(true);
    }
    if(msg.IsString()) {
      string sval = stripBlankEnds(msg.GetString());
      if(isNumber(sval)) {
        out = atof(sval.c_str());
        return(true);
      }
    }
    return(false);
  };

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
    else if (key == "ODOMETRY_RESET") {
      // Reset task sent latch if odometry resets (e.g. after refuel)
      m_task_sent = false;
      m_region_set = false;
    }
    else if (key == "OWN_REGION_WEIGHT") {
      double val = 0;
      if(readNumeric(msg, val))
        m_priority_weight = val;
    }
    else if (key == "OWN_REGION_X") {
      double val = 0;
      if(readNumeric(msg, val)) {
        m_region_x = val;
        m_region_set = true;
      }
    }
    else if (key == "OWN_REGION_Y") {
      double val = 0;
      if(readNumeric(msg, val)) {
        m_region_y = val;
        m_region_set = true;
      }
    }
    else if (key == "TASK_REFUEL") {
      processTaskRefuel(msg.GetString());
    }
    else if (key == "TASK_STATE") {
      processTaskState(msg.GetString());
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
// Every tick: publish local FUEL_DISTANCE_REMAINING so this
// vehicle's task behavior can evaluate its own feasibility.
//
// When odometry hits the refuel threshold: post a MISSION_TASK
// of type=refuelreplace carrying the region that needs coverage.

bool RefuelReplace::Iterate()
{
  AppCastingMOOSApp::Iterate();

  const bool have_nav  = (m_got_nav_x && m_got_nav_y);
  const bool have_odom = m_got_odom;

  // Publish fuel distance remaining every tick
  if(have_odom && (m_total_range > 0)) {
    m_fuel_distance_remaining = m_total_range - m_odometry_dist;
    if(m_fuel_distance_remaining < 0) m_fuel_distance_remaining = 0;
    Notify("FUEL_DISTANCE_REMAINING", m_fuel_distance_remaining);
  }

  // Trigger task once when odometry reaches threshold
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
    double rx = m_region_set ? m_region_x : m_nav_x;
    double ry = m_region_set ? m_region_y : m_nav_y;
    // add a warning flag if m_region_set is false and we're using current position as region center?
    if(!m_region_set) {
      reportRunWarning("Region not set, using current position as region center for task: " + id);
    }

    ostringstream os;
    os << "type=refuelreplace,"
       << "id=" << id << ","
       << "utc=" << doubleToStringX(utc, 2) << ","
       << "hash=" << hash << ","
       << "exempt=" << m_host_community << ","
       << "requester=" << m_host_community << ","
       << "region_x=" << doubleToStringX(rx, 2) << ","
       << "region_y=" << doubleToStringX(ry, 2) << ","
       << "priority_weight=" << doubleToStringX(m_priority_weight, 2);

    string task_spec = os.str();

    // 1) Local consume by this vehicle's task manager.
    Notify("MISSION_TASK", task_spec);

    // 2) Keep shore qbridge path for compatibility.
    //Notify("MISSION_TASK_ALL", task_spec);

    // 3) Explicit team broadcast via NodeComms -> MessageHandler path.
    //    This delivers MISSION_TASK directly into each teammate MOOSDB.
    // May need to improve on this message sending approach to use pShare soon 
    NodeMessage nmsg;
    nmsg.setSourceNode(m_host_community);
    nmsg.setSourceApp("pRefuelReplace");
    nmsg.setDestNode("all");
    nmsg.setVarName("MISSION_TASK");
    nmsg.setStringVal(task_spec);
    Notify("NODE_MESSAGE_LOCAL", nmsg.getSpec());

    // Seed local task cache from the exact task payload we posted.
    processTaskRefuel(task_spec);

    m_task_sent = true;
  }

  // Check if we need to send return handoff messages for any tasks we've won but not yet sent a handoff for.
  if(have_nav) {
    for(auto& entry : m_task_records) {
      const string task_hash = entry.first;
      TaskRecord& task = entry.second;
      if(!task.bidwon_by_me || task.handoff_sent || !task.region_set)
        continue;
      if((task.requester == "") || (task.requester == m_host_community))
        continue;

      double dist = hypot(m_nav_x - task.region_x, m_nav_y - task.region_y);
      if(dist <= m_handoff_radius) {
        notifyRequesterReturn(task.requester, task_hash);
        task.handoff_sent = true;
      }
    }
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
    else if (param == "total_range") {
      handled = setDoubleOnString(m_total_range, value);
    }
    else if (param == "vname") {
      handled = setNonWhiteVarOnString(m_host_community, value);
    }
    // The region x/y and priority will probably be set in some other logic about
    // the region this vehicle covers, but can set in config for now for testing
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
    else if (param == "handoff_radius") {
      handled = setNonNegDoubleOnString(m_handoff_radius, value);
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
  Register("OWN_REGION_X", 0);      
  Register("OWN_REGION_Y", 0);      
  Register("OWN_REGION_WEIGHT", 0); 
  Register("TASK_REFUEL", 0);
  Register("TASK_STATE", 0);
  Register("ODOMETRY_RESET", 0);
}

//---------------------------------------------------------
// Procedure: normalizeTaskSpec()
//   Task strings may use commas or '#' separators.

string RefuelReplace::normalizeTaskSpec(const string& msg) const
{
  string spec = msg;
  spec = findReplace(spec, " # ", ",");
  spec = findReplace(spec, "# ", ",");
  spec = findReplace(spec, " #", ",");
  spec = findReplace(spec, "#", ",");
  return(spec);
}

//---------------------------------------------------------
// Procedure: inferRequesterFromId()

// Infer requester plane from task id by stripping off "_rr" suffix and anything after it.
// We do this if requester field is missing but id is present in processTaskRefuel()
string RefuelReplace::inferRequesterFromId(const string& id) const
{
  if(id == "")
    return("");

  string requester = id;
  string delim = "_rr";
  string::size_type pos = id.find(delim);
  if(pos != string::npos)
    requester = id.substr(0, pos);
  return(stripBlankEnds(requester));
}

//---------------------------------------------------------
// Procedure: processTaskRefuel()

// Parse a TASK_REFUEL message and update the corresponding task record in m_task_records.
void RefuelReplace::processTaskRefuel(const string& task_msg)
{
  string spec = normalizeTaskSpec(task_msg);

  string hash = stripBlankEnds(tokStringParse(spec, "hash"));
  string id   = stripBlankEnds(tokStringParse(spec, "id"));
  if(id == "")
    id = stripBlankEnds(tokStringParse(spec, "name"));
  if((hash == "") && (id != ""))
    hash = id;
  if(hash == "")
    return;

  TaskRecord& rec = m_task_records[hash];
  if(id != "")
    rec.id = id;

  string requester = stripBlankEnds(tokStringParse(spec, "requester"));
  if((requester == "") && (rec.id != ""))
    requester = inferRequesterFromId(rec.id);
  if(requester != "")
    rec.requester = requester;

  string sx = stripBlankEnds(tokStringParse(spec, "region_x"));
  string sy = stripBlankEnds(tokStringParse(spec, "region_y"));
  if(isNumber(sx) && isNumber(sy)) {
    rec.region_x = atof(sx.c_str());
    rec.region_y = atof(sy.c_str());
    rec.region_set = true;
  }
}

//---------------------------------------------------------
// Procedure: processTaskState()

void RefuelReplace::processTaskState(const string& state_msg)
{
  string spec  = normalizeTaskSpec(state_msg);
  string hash  = stripBlankEnds(tokStringParse(spec, "hash"));
  string id    = stripBlankEnds(tokStringParse(spec, "id"));
  string state = tolower(stripBlankEnds(tokStringParse(spec, "state")));
  if(hash == "")
    return;

  TaskRecord& rec = m_task_records[hash];
  if(id != "")
    rec.id = id;

  if(rec.requester == "")
    rec.requester = inferRequesterFromId(rec.id);

  if(state == "bidwon") {
    if((rec.requester != "") && (rec.requester != m_host_community))
      rec.bidwon_by_me = true;
  }
  else if((state == "bidlost") || (state == "abstain")) {
    rec.bidwon_by_me = false;
  }
}

//---------------------------------------------------------
// Procedure: sendNodeMessage()

void RefuelReplace::sendNodeMessage(const string& dest_node,
                                    const string& var_name,
                                    const string& value)
{
  NodeMessage nmsg;
  nmsg.setSourceNode(m_host_community);
  nmsg.setSourceApp("pRefuelReplace");
  nmsg.setDestNode(dest_node);
  nmsg.setVarName(var_name);
  nmsg.setStringVal(value);
  Notify("NODE_MESSAGE_LOCAL", nmsg.getSpec());
}

//---------------------------------------------------------
// Procedure: notifyRequesterReturn()

void RefuelReplace::notifyRequesterReturn(const string& requester,
                                          const string& task_hash)
{
  sendNodeMessage(requester, "DEPLOY", "false");
  sendNodeMessage(requester, "DO_SURVEY", "false");
  sendNodeMessage(requester, "LOITER", "false");
  sendNodeMessage(requester, "RETURN", "true");

  string info = "requester=" + requester + ",hash=" + task_hash;
  Notify("REFUEL_HANDOFF", info);
  reportEvent("Return handoff sent: " + info);
}

//---------------------------------------------------------
// Procedure: buildReport()

bool RefuelReplace::buildReport()
{
  ACTable table(2);
  table << "Field" << "Value";
  table.addHeaderLines();

  table << "refuel_threshold"       << doubleToStringX(m_refuel_threshold, 2);
  table << "total_range"            << doubleToStringX(m_total_range, 2);
  table << "handoff_radius"         << doubleToStringX(m_handoff_radius, 2);
  table << "ODOMETRY_DIST"          << doubleToStringX(m_odometry_dist, 2);

  table << "FUEL_DISTANCE_REMAINING" << doubleToStringX(m_fuel_distance_remaining, 2);

  table << "NAV_X"                << doubleToStringX(m_nav_x, 2);
  table << "NAV_Y"                << doubleToStringX(m_nav_y, 2);
  table << "region_x"            << doubleToStringX(m_region_x, 2);
  table << "region_y"            << doubleToStringX(m_region_y, 2);
  table << "priority_weight"     << doubleToStringX(m_priority_weight, 2);
  table << "region_set"          << (m_region_set ? "true" : "false");
  table << "got_odom"             << (m_got_odom  ? "true" : "false");
  table << "task_sent"            << (m_task_sent ? "true" : "false");
  table << "next_task_id_counter" << intToString(m_task_id_counter);
  table << "tracked_tasks"         << uintToString((unsigned int)m_task_records.size());

  m_msgs << table.getFormattedString();
  return true;
}
