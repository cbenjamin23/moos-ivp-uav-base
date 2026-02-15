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

// pRefuelReplace overview:
// 1) Publishes FUEL_DISTANCE_REMAINING each iterate from total_range - odometry.
// 2) Posts replacement tasks when either:
//    - refuel threshold is crossed (one-shot per reset cycle), or
//    - a REFUEL_DISCOVERY_REQUEST arrives (fire-id deduped + age-limited).
//    Posting checks require nav/odom readiness and no active replacement lock.
// 3) Tracks TASK_STATE for spawned tasks and infers winner identity from payload
//    (or source metadata fallback), then enforces a single active replacement lock.
// 4) For target tasks, sends return handoff when within handoff_radius; lock is
//    released on handoff completion, basic-return completion, or timeout fallback.

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
  m_returning_mode = false;

  // Config
  m_refuel_threshold = 0.0;   // meters; disabled if <= 0
  m_total_range      = 0.0;   // meters; total range on full fuel
  m_handoff_radius   = 50.0;  // meters
  // Lock timeout protects against wedged transit-busy state.
  m_replacement_lock_timeout = 600.0; // seconds
  // Discovery queue guards: stale-request drop + fire-id cooldown dedupe.
  m_discovery_request_timeout = 30.0; // seconds
  m_discovery_repost_cooldown = 60.0; // seconds

  // Target this vehicle covers
  m_target_x = 0;
  m_target_y = 0;
  m_target_set = false;
  m_priority_weight = 1.0;

  // State
  m_waiting_for_odom_reset = false;
  m_task_sent = false;
  m_task_id_counter = 0;
  // Empty hash means this vehicle is not committed to any replacement task.
  m_active_replacement_hash = "";
  m_active_replacement_type = "";
  m_active_replacement_time = 0.0;
  m_active_replacement_return_started = false;
  // Discovery-triggered posting is queued to keep mail handling lightweight.
  m_pending_discovery_fire_id = "";
  m_pending_discovery_utc = 0.0;

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
    else if (key == "RETURN") {
      // Used by lock-release logic for basic replacement tasks.
      if(msg.IsDouble()) {
        m_returning_mode = (msg.GetDouble() != 0.0);
      }
      else if(msg.IsString()) {
        string sval = tolower(stripBlankEnds(msg.GetString()));
        if((sval == "true") || (sval == "1") || (sval == "on"))
          m_returning_mode = true;
        else if((sval == "false") || (sval == "0") || (sval == "off"))
          m_returning_mode = false;
      }
    }
    else if (key == "ODOMETRY_RESET") {
      bool do_reset = false;
      if(msg.IsDouble()) {
        do_reset = (msg.GetDouble() != 0.0);
      }
      else if(msg.IsString()) {
        string sval = tolower(stripBlankEnds(msg.GetString()));
        do_reset = (sval == "true") || (sval == "1") || (sval == "reset");
      }

      // Arm re-latching logic; clear m_task_sent only after odometry
      // has actually dropped to a low value on a subsequent update.
      if(do_reset)
        m_waiting_for_odom_reset = true;
    }
    else if (key == "TASK_RESET") {
      // Legacy reset path (still supported): explicitly unlatch threshold posting.
      bool do_reset = false;
      if(msg.IsDouble()) {
        do_reset = (msg.GetDouble() != 0.0);
      }
      else if(msg.IsString()) {
        string sval = tolower(stripBlankEnds(msg.GetString()));
        do_reset = (sval == "true") || (sval == "1") || (sval == "reset");
      }

      if(do_reset) {
        m_task_sent = false;
      }
    }
    else if (key == "TARGET_RESET") {
      bool do_reset = false;
      if(msg.IsDouble()) {
        do_reset = (msg.GetDouble() != 0.0);
      }
      else if(msg.IsString()) {
        string sval = tolower(stripBlankEnds(msg.GetString()));
        do_reset = (sval == "true") || (sval == "1") || (sval == "reset");
      }

      if(do_reset) {
        m_target_set = false;
      }
    }
    else if (key == "REFUEL_DISCOVERY_REQUEST") {
      // Discovery-driven replacement requests are queued and processed in Iterate()
      // so posting decisions can use up-to-date nav/odom/lock state together.
      string spec = normalizeTaskSpec(msg.GetString());
      string fire_id = stripBlankEnds(tokStringParse(spec, "fire_id"));
      if(fire_id == "")
        fire_id = stripBlankEnds(msg.GetString());
      if(fire_id == "")
        fire_id = "unknown";

      m_pending_discovery_fire_id = fire_id;
      m_pending_discovery_utc = MOOSTime();
      reportEvent("Queued discovery replacement request: fire_id=" + fire_id);
    }
    else if (key == "OWN_TARGET_WEIGHT") {
      double val = 0;
      if(readNumeric(msg, val))
        m_priority_weight = val;
    }
    else if (key == "OWN_TARGET_X") {
      double val = 0;
      if(readNumeric(msg, val)) {
        m_target_x = val;
        m_target_set = true;
      }
    }
    else if (key == "OWN_TARGET_Y") {
      double val = 0;
      if(readNumeric(msg, val)) {
        m_target_y = val;
        m_target_set = true;
      }
    }
    else if (key == "TASK_REFUEL_TARGET") {
      processTaskRefuelTarget(msg.GetString());
    }
    else if (key == "TASK_REFUEL_BASIC") {
      processTaskRefuelTarget(msg.GetString());
    }
    else if (key == "TASK_STATE") {
      // Include source metadata; older TASK_STATE payloads may omit explicit winner.
      processTaskState(msg.GetString(), msg.GetSource(), msg.GetCommunity());
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
// When odometry hits the refuel threshold: post a MISSION_TASK.
// If target is set, request target-based replacement. Otherwise request
// basic replacement.

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

  // After a reset command, wait for odometry to actually drop low
  // before allowing the next threshold-triggered task.
  if(m_waiting_for_odom_reset && have_odom) {
    const double odom_reset_clear_thresh = 50.0;
    if(m_odometry_dist <= odom_reset_clear_thresh) {
      m_task_sent = false;
      m_waiting_for_odom_reset = false;
    }
  }

  // Handle fire-discovery-triggered replacement request. This path is
  // independent of the threshold latch and intentionally one-shot per fire
  // within a cooldown window.
  if(m_pending_discovery_fire_id != "") {
    double now = MOOSTime();
    double age = now - m_pending_discovery_utc;
    if((m_discovery_request_timeout > 0.0) && (age > m_discovery_request_timeout)) {
      reportRunWarning("Dropping stale discovery request: fire_id=" +
                       m_pending_discovery_fire_id);
      m_pending_discovery_fire_id = "";
      m_pending_discovery_utc = 0.0;
    }
    else {
      const string fire_id = m_pending_discovery_fire_id;
      bool recently_posted = false;
      if(m_last_discovery_post_utc.count(fire_id) > 0) {
        double dt = now - m_last_discovery_post_utc[fire_id];
        recently_posted =
          (m_discovery_repost_cooldown > 0.0) && (dt < m_discovery_repost_cooldown);
      }

      if(recently_posted) {
        // Same fire can re-notify rapidly; suppress duplicate team auctions.
        reportEvent("Skipping duplicate discovery-triggered replacement: fire_id=" +
                    fire_id);
        m_pending_discovery_fire_id = "";
        m_pending_discovery_utc = 0.0;
      }
      else if(have_odom &&
              ((m_refuel_threshold <= 0.0) ||
               (m_odometry_dist < m_refuel_threshold))) {
        // Discovery-triggered replacement is only intended once this vehicle
        // is in replacement-needed territory (same threshold semantics).
        reportEvent("Ignoring discovery-triggered replacement below threshold: fire_id=" +
                    fire_id);
        m_pending_discovery_fire_id = "";
        m_pending_discovery_utc = 0.0;
      }
      else if((m_active_replacement_hash == "") && have_nav && have_odom) {
        // Post only if this vehicle is not already committed to another replacement.
        if(postReplacementTask("discovery_fire_" + fire_id, true))
          m_last_discovery_post_utc[fire_id] = now;
        m_pending_discovery_fire_id = "";
        m_pending_discovery_utc = 0.0;
      }
    }
  }

  // Trigger task once when odometry reaches threshold
  if (!m_task_sent &&
      !m_waiting_for_odom_reset &&
      (m_refuel_threshold > 0.0) &&
      have_nav && have_odom &&
      (m_odometry_dist >= m_refuel_threshold))
  {
    // Standard threshold path remains one-shot per odometry-reset cycle.
    postReplacementTask("threshold_refuel", false);
  }

  // Check if we need to send return handoff messages for any tasks we've won but not yet sent a handoff for.
  if(have_nav) {
    for(auto& entry : m_task_records) {
      const string task_hash = entry.first;
      TaskRecord& task = entry.second;

      // Enforce single active replacement: only service the currently locked task.
      if((m_active_replacement_hash != "") && (task_hash != m_active_replacement_hash))
        continue;

      if(!task.bidwon_by_me || task.handoff_sent || !task.target_set)
        continue;
      if((task.requester == "") || (task.requester == m_host_community))
        continue;

      double dist = hypot(m_nav_x - task.target_x, m_nav_y - task.target_y);
      if(dist <= m_handoff_radius) {
        // Handoff completion signal for target tasks: requester is told to return.
        notifyRequesterReturn(task.requester, task_hash);
        task.handoff_sent = true;
      }
    }
  }

  // Maintain and release explicit active replacement lock.
  if(m_active_replacement_hash != "") {
    // Optional timeout guard against stale lock.
    if((m_replacement_lock_timeout > 0.0) &&
       ((MOOSTime() - m_active_replacement_time) > m_replacement_lock_timeout)) {
      clearActiveReplacementLock("timeout");
    }
    // Active task disappeared from cache.
    else if(m_task_records.count(m_active_replacement_hash) == 0) {
      clearActiveReplacementLock("task_record_missing");
    }
    else {
      TaskRecord& active = m_task_records[m_active_replacement_hash];
      if((m_active_replacement_type == "") && (active.task_type != ""))
        // Task type may arrive after bidwon in asynchronous mail order.
        m_active_replacement_type = active.task_type;

      // Identify "basic-like" tasks that have no handoff target. For these tasks
      // we hold lock for the entire return leg rather than clearing on RETURN=true.
      const bool basic_like_task =
        (m_active_replacement_type == "refuelreplace_basic") ||
        ((m_active_replacement_type == "") && !active.target_set);

      // Mark that the active basic-like winner has started its return leg.
      if(basic_like_task && m_returning_mode)
        m_active_replacement_return_started = true;

      // Any task with a completed handoff can release immediately.
      if(active.handoff_sent) {
        clearActiveReplacementLock("handoff_complete");
      }
      // Basic-like tasks only release once return completes (ODOMETRY_RESET path).
      // This keeps REFUEL_TRANSIT_BUSY asserted while actively returning.
      else if(basic_like_task &&
              m_active_replacement_return_started &&
              m_waiting_for_odom_reset) {
        if(m_active_replacement_type == "refuelreplace_basic")
          clearActiveReplacementLock("basic_return_complete");
        else
          clearActiveReplacementLock("unknown_basic_return_complete");
      }
    }
  }
  bool transit_busy = (m_active_replacement_hash != "");
  Notify("REFUEL_TRANSIT_BUSY", transit_busy ? "true" : "false");
 
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
    // The target x/y and priority will probably be set in some other logic about
    // the target this vehicle covers, but can set in config for now for testing
    else if (param == "target_x") {
      handled = setDoubleOnString(m_target_x, value);
      if(handled) m_target_set = true;
    }
    else if (param == "target_y") {
      handled = setDoubleOnString(m_target_y, value);
      if(handled) m_target_set = true;
    }
    else if (param == "priority_weight") {
      handled = setNonNegDoubleOnString(m_priority_weight, value);
    }
    else if (param == "handoff_radius") {
      handled = setNonNegDoubleOnString(m_handoff_radius, value);
    }
    else if (param == "replacement_lock_timeout") {
      handled = setNonNegDoubleOnString(m_replacement_lock_timeout, value);
    }
    else if (param == "discovery_request_timeout") {
      handled = setNonNegDoubleOnString(m_discovery_request_timeout, value);
    }
    else if (param == "discovery_repost_cooldown") {
      handled = setNonNegDoubleOnString(m_discovery_repost_cooldown, value);
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
  Register("OWN_TARGET_X", 0);      
  Register("OWN_TARGET_Y", 0);      
  Register("OWN_TARGET_WEIGHT", 0); 
  Register("TASK_REFUEL_TARGET", 0);
  Register("TASK_REFUEL_BASIC", 0);
  // TASK_STATE arrives from spawned helm task behaviors on this platform.
  Register("TASK_STATE", 0);
  Register("ODOMETRY_RESET", 0);
  Register("TASK_RESET", 0);
  Register("TARGET_RESET", 0);
  Register("REFUEL_DISCOVERY_REQUEST", 0);
  Register("RETURN", 0);
}

//---------------------------------------------------------
// Procedure: postReplacementTask()

bool RefuelReplace::postReplacementTask(const string& trigger_reason,
                                        bool bypass_task_latch)
{
  if(!bypass_task_latch && m_task_sent)
    return(false);
  if(!m_got_odom || !m_got_nav_x || !m_got_nav_y)
    return(false);
  if((!bypass_task_latch) && (m_odometry_dist < m_refuel_threshold))
    return(false);

  string id = m_host_community + "_rr" + intToString(m_task_id_counter++);
  bool basic_task = false;

  double utc = MOOSTime();

  long utc_int  = (long)(utc * 100);
  long utc_tail = utc_int % 100000;
  string hash = "rr_" + id + "_" + intToString((int)utc_tail);

  ostringstream os;
  if(m_target_set) {
    os << "type=refuelreplace_target,"
       << "id=" << id << ","
       << "utc=" << doubleToStringX(utc, 2) << ","
       << "hash=" << hash << ","
       << "exempt=" << m_host_community << ","
       << "requester=" << m_host_community << ","
       << "requester_x=" << doubleToStringX(m_nav_x, 2) << ","
       << "requester_y=" << doubleToStringX(m_nav_y, 2) << ","
       << "target_x=" << doubleToStringX(m_target_x, 2) << ","
       << "target_y=" << doubleToStringX(m_target_y, 2) << ","
       << "priority_weight=" << doubleToStringX(m_priority_weight, 2) << ","
       << "fuel_abstain_threshold=" << doubleToStringX(m_refuel_threshold, 2);
  }
  else {
    basic_task = true;
    os << "type=refuelreplace_basic,"
       << "id=" << id << ","
       << "utc=" << doubleToStringX(utc, 2) << ","
       << "hash=" << hash << ","
       << "exempt=" << m_host_community << ","
       //<< "requester=" << m_host_community << ","
       << "requester_x=" << doubleToStringX(m_nav_x, 2) << ","
       << "requester_y=" << doubleToStringX(m_nav_y, 2) << ","
       << "fuel_abstain_threshold=" << doubleToStringX(m_refuel_threshold, 2);
    reportEvent("Target not set, posting basic refuel task: " + id);
  }

  string task_spec = os.str();
  // Publish locally and through NODE_MESSAGE so all teammates receive
  // identical task payloads.
  Notify("MISSION_TASK", task_spec);

  NodeMessage nmsg;
  nmsg.setSourceNode(m_host_community);
  nmsg.setSourceApp("pRefuelReplace");
  nmsg.setDestNode("all");
  nmsg.setVarName("MISSION_TASK");
  nmsg.setStringVal(task_spec);
  Notify("NODE_MESSAGE_LOCAL", nmsg.getSpec());

  processTaskRefuelTarget(task_spec);

  if(basic_task) {
    // Basic replacement requests immediately put requester on return leg.
    Notify("DEPLOY", "false");
    Notify("DO_SURVEY", "false");
    Notify("LOITER", "false");
    Notify("RETURN", "true");
    reportEvent("Basic replacement posted, commanding return home: " + id);
  }

  m_task_sent = true;
  reportEvent("Posted replacement task id=" + id + ", reason=" + trigger_reason);
  return(true);
}

//---------------------------------------------------------
// Procedure: parseTaskStateWinner()

string RefuelReplace::parseTaskStateWinner(const string& spec) const
{
  // Support multiple field spellings across task-manager variants.
  static const char* winner_keys[] = {
    "winner",
    "bid_winner",
    "bidwinner",
    "winner_vname",
    "winning_vname",
    "awarded_to",
    "assigned_to"
  };

  for(unsigned int i=0; i<sizeof(winner_keys)/sizeof(winner_keys[0]); i++) {
    string val = stripBlankEnds(tokStringParse(spec, winner_keys[i]));
    if(val != "")
      return(val);
  }

  return("");
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
// We do this if requester field is missing but id is present in processTaskRefuelTarget()
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
// Procedure: processTaskRefuelTarget()

// Parse a TASK_REFUEL_TARGET or TASK_REFUEL_BASIC message and update
// the corresponding task record in m_task_records.
void RefuelReplace::processTaskRefuelTarget(const string& task_msg)
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

  string task_type = stripBlankEnds(tokStringParse(spec, "type"));
  if(task_type != "")
    rec.task_type = task_type;

  string requester = stripBlankEnds(tokStringParse(spec, "requester"));
  if((requester == "") && (rec.id != ""))
    requester = inferRequesterFromId(rec.id);
  if(requester != "")
    rec.requester = requester;

  string sx = stripBlankEnds(tokStringParse(spec, "target_x"));
  string sy = stripBlankEnds(tokStringParse(spec, "target_y"));
  if(isNumber(sx) && isNumber(sy)) {
    rec.target_x = atof(sx.c_str());
    rec.target_y = atof(sy.c_str());
    rec.target_set = true;
  }
}

//---------------------------------------------------------
// Procedure: processTaskState()

void RefuelReplace::processTaskState(const string& state_msg,
                                     const string& msg_source_app,
                                     const string& msg_source_community)
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

  string winner = parseTaskStateWinner(spec);
  // Fallback path for TASK_STATE payloads that only include id/hash/state.
  // In these cases source community corresponds to the local helm node.
  if((winner == "") && (msg_source_community != ""))
    winner = msg_source_community;
  if((winner == "") && (tolower(stripBlankEnds(msg_source_app)) == "ptaskmanager"))
    winner = m_host_community;

  if(state == "bidwon") {
    if(winner == "") {
      // Without a winner identity we cannot safely claim this bid as ours.
      rec.bidwon_by_me = false;
      return;
    }

    string winner_lc = tolower(stripBlankEnds(winner));
    string host_lc = tolower(stripBlankEnds(m_host_community));
    if((winner != "") && (winner_lc == host_lc)) {
      if(m_active_replacement_hash == "") {
        // First accepted win acquires the replacement lock.
        rec.bidwon_by_me = true;
        m_active_replacement_hash = hash;
        m_active_replacement_type = rec.task_type;
        // Fallback inference if task type arrives late.
        if((m_active_replacement_type == "") && rec.target_set)
          m_active_replacement_type = "refuelreplace_target";
        m_active_replacement_time = MOOSTime();
        m_active_replacement_return_started = false;
      }
      else if(m_active_replacement_hash == hash) {
        // Repeated status update for same task: refresh lock freshness.
        rec.bidwon_by_me = true;
        m_active_replacement_type = rec.task_type;
        if((m_active_replacement_type == "") && rec.target_set)
          m_active_replacement_type = "refuelreplace_target";
        m_active_replacement_time = MOOSTime();
      }
      else {
        // Ignore additional wins while already committed to another task.
        rec.bidwon_by_me = false;
        reportRunWarning("Concurrent bidwon while lock held. held_hash=" +
                         m_active_replacement_hash + ", new_hash=" + hash);
      }
    }
    else {
      rec.bidwon_by_me = false;
      if(m_active_replacement_hash == hash)
        clearActiveReplacementLock("task_state_bidwon_other_winner");
    }
  }
  else if((state == "bidlost") || (state == "abstain")) {
    // If the active task transitions out of bidwon, release commitment.
    rec.bidwon_by_me = false;
    if(m_active_replacement_hash == hash)
      clearActiveReplacementLock("task_state_" + state);
  }
}

//---------------------------------------------------------
// Procedure: clearActiveReplacementLock()

void RefuelReplace::clearActiveReplacementLock(const string& reason)
{
  if(m_active_replacement_hash == "")
    return;

  // Keep reason in event log for post-run debugging of lock lifecycle.
  reportEvent("Clearing replacement lock hash=" + m_active_replacement_hash +
              ", reason=" + reason);
  m_active_replacement_hash = "";
  m_active_replacement_type = "";
  m_active_replacement_time = 0.0;
  m_active_replacement_return_started = false;
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
  sendNodeMessage(requester, "TARGET_RESET", "true");

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
  table << "target_x"            << doubleToStringX(m_target_x, 2);
  table << "target_y"            << doubleToStringX(m_target_y, 2);
  table << "priority_weight"     << doubleToStringX(m_priority_weight, 2);
  table << "target_set"          << (m_target_set ? "true" : "false");
  table << "got_odom"             << (m_got_odom  ? "true" : "false");
  table << "waiting_odom_reset"   << (m_waiting_for_odom_reset ? "true" : "false");
  table << "task_sent"            << (m_task_sent ? "true" : "false");
  table << "next_task_id_counter" << intToString(m_task_id_counter);
  table << "tracked_tasks"         << uintToString((unsigned int)m_task_records.size());
  bool transit_busy = (m_active_replacement_hash != "");
  table << "refuel_transit_busy"   << (transit_busy ? "true" : "false");
  table << "returning_mode"        << (m_returning_mode ? "true" : "false");
  table << "lock_hash"             << (m_active_replacement_hash == "" ?
                                        "(none)" : m_active_replacement_hash);
  table << "lock_type"             << (m_active_replacement_type == "" ?
                                        "(none)" : m_active_replacement_type);
  table << "lock_return_started"   <<
             (m_active_replacement_return_started ? "true" : "false");
  table << "lock_timeout_s"        << doubleToStringX(m_replacement_lock_timeout, 2);
  table << "pending_discovery_fire" << (m_pending_discovery_fire_id == "" ?
                                          "(none)" : m_pending_discovery_fire_id);
  table << "discovery_timeout_s"    << doubleToStringX(m_discovery_request_timeout, 2);
  table << "discovery_cooldown_s"   << doubleToStringX(m_discovery_repost_cooldown, 2);

  m_msgs << table.getFormattedString();
  return true;
}
