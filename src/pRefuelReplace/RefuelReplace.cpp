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
// 2) Posts replacement tasks from two trigger paths:
//    - threshold path (one-shot per odometry-reset cycle via m_task_sent), and
//    - discovery override path (queued; fire-id deduped; only posts
//      immediately if the vehicle is already at/above threshold).
// 3) Tracks thin pre-win task metadata keyed by hash and uses TASK_STATE
//    to infer local winner identity robustly across payload variants.
// 4) Maintains a single-active replacement lock:
//    - lock acquisition on accepted bidwon,
//    - lock release on handoff completion, basic return completion, or timeout.
// 5) Publishes REFUEL_TRANSIT_BUSY while lock is held so task behaviors abstain
//    from concurrent replacement bids on the same vehicle.

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
  m_unawarded_retry_delay = 1.0; // seconds
  m_unawarded_retry_limit = 2;

  // Target this vehicle covers
  m_target_x = 0;
  m_target_y = 0;
  m_target_set = false;
  m_priority_weight = 1.0;

  // State
  m_waiting_for_odom_reset = false;
  m_task_sent = false;
  m_task_id_counter = 0;
  m_active_lock.clear();
  // Discovery override requests are queued to keep mail handling lightweight.
  m_pending_discovery_fire_id = "";
  m_pending_discovery_utc = 0.0;
  m_pending_retry_utc = 0.0;
  m_retry_attempts_used = 0;

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

      // Arm threshold re-latching logic; clear m_task_sent only after
      // odometry has actually dropped to a low value on a subsequent update.
      if(do_reset) {
        m_waiting_for_odom_reset = true;
        // Track reset separately for active-lock completion logic.
        if(activeLockEngaged())
          m_active_lock.odom_reset_seen = true;
      }
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
      // Discovery override requests are queued and processed in Iterate() so
      // posting decisions can use up-to-date nav/odom/lock state together.
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
      upsertTaskInfo(msg.GetString());
    }
    else if (key == "TASK_REFUEL_BASIC") {
      upsertTaskInfo(msg.GetString());
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
// Trigger processing order:
// 1) threshold relatch handling (ODOMETRY_RESET + low-odom clear),
// 2) queued discovery-override handling (age + cooldown + threshold),
// 3) threshold-triggered posting,
// 4) target handoff detection and lock release.

bool RefuelReplace::Iterate()
{
  AppCastingMOOSApp::Iterate();

  const bool have_nav  = (m_got_nav_x && m_got_nav_y);
  const bool have_odom = m_got_odom;

  updateFuelDistanceRemaining();
  maybeRelatchAfterOdometryReset();
  maybeHandleImmediateDiscoveryReplacement(have_nav, have_odom);
  maybeHandlePendingUnawardedRetry(have_nav, have_odom);
  maybePostThresholdReplacement(have_nav, have_odom);
  maybeSendActiveHandoff(have_nav);
  maybeMaintainActiveLock();
  prunePendingTasks(MOOSTime());

  bool transit_busy = activeLockEngaged();
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
    else if (param == "unawarded_retry_delay") {
      handled = setNonNegDoubleOnString(m_unawarded_retry_delay, value);
    }
    else if (param == "unawarded_retry_limit") {
      handled = setUIntOnString(m_unawarded_retry_limit, value);
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
//   bypass_task_latch semantics:
//   - false: enforce one-shot threshold latch and threshold gate.
//   - true: skip latch/threshold gate for discovery path; caller is responsible
//           for policy checks (freshness/cooldown/threshold/lock readiness).

bool RefuelReplace::postReplacementTask(const string& trigger_reason,
                                        bool bypass_task_latch)
{
  PendingTaskInfo task_info;
  task_info.requester = m_host_community;
  task_info.priority_weight = m_priority_weight;
  if(m_target_set) {
    task_info.task_type = "refuelreplace_target";
    task_info.target_x = m_target_x;
    task_info.target_y = m_target_y;
    task_info.target_set = true;
  }
  else {
    task_info.task_type = "refuelreplace_basic";
  }

  return(postReplacementTaskFromInfo(task_info, trigger_reason,
                                     bypass_task_latch, true));
}

//---------------------------------------------------------
// Procedure: postReplacementTaskFromInfo()
//   Reposts a replacement task using supplied task metadata.

bool RefuelReplace::postReplacementTaskFromInfo(const PendingTaskInfo& task_info,
                                                const string& trigger_reason,
                                                bool bypass_task_latch,
                                                bool reset_retry_sequence)
{
  if(!bypass_task_latch && m_task_sent)
    return(false);
  if(!m_got_odom || !m_got_nav_x || !m_got_nav_y)
    return(false);
  if((!bypass_task_latch) && (m_odometry_dist < m_refuel_threshold))
    return(false);

  string id = m_host_community + "_rr" + intToString(m_task_id_counter++);
  const string task_type = stripBlankEnds(task_info.task_type);
  const bool target_task =
    (task_type == "refuelreplace_target") ||
    ((task_type == "") && task_info.target_set);
  const bool basic_task = !target_task;
  const string requester =
    (task_info.requester == "") ? m_host_community : task_info.requester;

  double utc = MOOSTime();

  long utc_int  = (long)(utc * 100);
  long utc_tail = utc_int % 100000;
  string hash = "rr_" + id + "_" + intToString((int)utc_tail);

  ostringstream os;
  if(target_task) {
    os << "type=refuelreplace_target,"
       << "id=" << id << ","
       << "utc=" << doubleToStringX(utc, 2) << ","
       << "hash=" << hash << ","
       << "requester=" << requester << ","
       << "requester_x=" << doubleToStringX(m_nav_x, 2) << ","
       << "requester_y=" << doubleToStringX(m_nav_y, 2) << ","
       << "target_x=" << doubleToStringX(task_info.target_x, 2) << ","
       << "target_y=" << doubleToStringX(task_info.target_y, 2) << ","
       << "priority_weight=" << doubleToStringX(task_info.priority_weight, 2) << ","
       << "fuel_abstain_threshold=" << doubleToStringX(m_refuel_threshold, 2);
  }
  else {
    os << "type=refuelreplace_basic,"
       << "id=" << id << ","
       << "utc=" << doubleToStringX(utc, 2) << ","
       << "hash=" << hash << ","
       << "requester=" << requester << ","
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

  upsertTaskInfo(task_spec);

  if(basic_task) {
    // Basic replacement requests immediately put requester on return leg.
    Notify("DEPLOY", "false");
    Notify("DO_SURVEY", "false");
    Notify("LOITER", "false");
    Notify("RETURN", "true");
    reportEvent("Basic replacement posted, commanding return home: " + id);
  }

  m_task_sent = true;
  if(reset_retry_sequence) {
    m_pending_retry_task = PendingTaskInfo();
    m_pending_retry_utc = 0.0;
    m_retry_attempts_used = 0;
  }
  reportEvent("Posted replacement task id=" + id + ", reason=" + trigger_reason);
  return(true);
}

//---------------------------------------------------------
// Procedure: parseTaskStateWinner()
//   Extracts the winner field from variant TASK_STATE payload formats.

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
//   This keeps parsing stable across slightly different publishers.

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
//   Falls back to requester identity encoded in the generated task id.

// Infer requester plane from task id by stripping off "_rr" suffix and anything after it.
// We do this if requester field is missing but id is present in upsertTaskInfo()
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
// Procedure: activeLockEngaged()
//   Returns true when this vehicle currently holds an active replacement lock.

bool RefuelReplace::activeLockEngaged() const
{
  return(m_active_lock.engaged());
}

//---------------------------------------------------------
// Procedure: activeTaskUsesReturnReset()
//   Classifies tasks that should release only after RETURN + ODOMETRY_RESET.

bool RefuelReplace::activeTaskUsesReturnReset(const string& task_type,
                                              bool target_set) const
{
  return((task_type == "refuelreplace_basic") ||
         ((task_type == "") && !target_set));
}

//---------------------------------------------------------
// Procedure: updateFuelDistanceRemaining()
//   Publishes local remaining range from total range minus traveled odometry.

void RefuelReplace::updateFuelDistanceRemaining()
{
  if(!m_got_odom || (m_total_range <= 0))
    return;

  m_fuel_distance_remaining = m_total_range - m_odometry_dist;
  if(m_fuel_distance_remaining < 0)
    m_fuel_distance_remaining = 0;
  Notify("FUEL_DISTANCE_REMAINING", m_fuel_distance_remaining);
}

//---------------------------------------------------------
// Procedure: maybeRelatchAfterOdometryReset()
//   Re-arms threshold-triggered posting once odometry has dropped low again.

void RefuelReplace::maybeRelatchAfterOdometryReset()
{
  if(!m_waiting_for_odom_reset || !m_got_odom)
    return;

  const double odom_reset_clear_thresh = 50.0;
  if(m_odometry_dist <= odom_reset_clear_thresh) {
    m_task_sent = false;
    m_waiting_for_odom_reset = false;
  }
}

//---------------------------------------------------------
// Procedure: maybeHandleImmediateDiscoveryReplacement()
//   Services the queued discovery override that can post immediate relief only
//   if the vehicle is already at/above threshold.

void RefuelReplace::maybeHandleImmediateDiscoveryReplacement(bool have_nav, bool have_odom)
{
  if(m_pending_discovery_fire_id == "")
    return;

  double now = MOOSTime();
  double age = now - m_pending_discovery_utc;
  if((m_discovery_request_timeout > 0.0) && (age > m_discovery_request_timeout)) {
    reportRunWarning("Dropping stale discovery request: fire_id=" +
                     m_pending_discovery_fire_id);
    m_pending_discovery_fire_id = "";
    m_pending_discovery_utc = 0.0;
    return;
  }

  const string fire_id = m_pending_discovery_fire_id;
  bool recently_posted = false;
  if(m_last_discovery_post_utc.count(fire_id) > 0) {
    double dt = now - m_last_discovery_post_utc[fire_id];
    recently_posted =
      (m_discovery_repost_cooldown > 0.0) && (dt < m_discovery_repost_cooldown);
  }

  if(recently_posted) {
    // Same fire can re-notify rapidly; suppress duplicate team auctions.
    reportEvent("Skipping duplicate discovery override replacement: fire_id=" +
                fire_id);
    m_pending_discovery_fire_id = "";
    m_pending_discovery_utc = 0.0;
  }
  else if(have_odom &&
          ((m_refuel_threshold <= 0.0) ||
           (m_odometry_dist < m_refuel_threshold))) {
    // Discovery override is only intended when this vehicle is already
    // in replacement-needed territory (same threshold semantics).
    // postReplacementTask(..., bypass=true) skips the internal threshold
    // gate, so this explicit check remains the policy gate.
    reportEvent("Ignoring discovery override below threshold: fire_id=" +
                fire_id);
    m_pending_discovery_fire_id = "";
    m_pending_discovery_utc = 0.0;
  }
  else if(!activeLockEngaged() && have_nav && have_odom) {
    // Post only if this vehicle is not already committed to another replacement.
    if(postReplacementTask("discovery_fire_" + fire_id, true))
      m_last_discovery_post_utc[fire_id] = now;
    m_pending_discovery_fire_id = "";
    m_pending_discovery_utc = 0.0;
  }
}

//---------------------------------------------------------
// Procedure: maybeHandlePendingUnawardedRetry()
//   Posts a queued retry for a locally requested replacement after a fixed delay.

void RefuelReplace::maybeHandlePendingUnawardedRetry(bool have_nav, bool have_odom)
{
  if(m_pending_retry_utc <= 0.0)
    return;
  if(!have_nav || !have_odom)
    return;
  if(activeLockEngaged())
    return;

  const double now = MOOSTime();
  if(now < m_pending_retry_utc)
    return;

  string reason = "unawarded_retry_" + uintToString(m_retry_attempts_used);
  if(postReplacementTaskFromInfo(m_pending_retry_task, reason, true, false)) {
    reportEvent("Posted unawarded retry attempt " +
                uintToString(m_retry_attempts_used) + "/" +
                uintToString(m_unawarded_retry_limit));
    m_pending_retry_task = PendingTaskInfo();
    m_pending_retry_utc = 0.0;
  }
}

//---------------------------------------------------------
// Procedure: maybePostThresholdReplacement()
//   Posts the standard threshold-triggered replacement task once per reset cycle.

void RefuelReplace::maybePostThresholdReplacement(bool have_nav, bool have_odom)
{
  if (!m_task_sent &&
      !m_waiting_for_odom_reset &&
      (m_refuel_threshold > 0.0) &&
      have_nav && have_odom &&
      (m_odometry_dist >= m_refuel_threshold))
  {
    // Standard threshold path remains one-shot per odometry-reset cycle.
    postReplacementTask("threshold_refuel", false);
  }
}

//---------------------------------------------------------
// Procedure: maybeSendActiveHandoff()
//   Tells the requester to return once the active target winner reaches handoff range.

void RefuelReplace::maybeSendActiveHandoff(bool have_nav)
{
  if(!have_nav || !activeLockEngaged())
    return;

  if(m_active_lock.handoff_sent ||
     !m_active_lock.target_set ||
     (m_active_lock.requester == "") ||
     (m_active_lock.requester == m_host_community)) {
    return;
  }

  double dist = hypot(m_nav_x - m_active_lock.target_x,
                      m_nav_y - m_active_lock.target_y);
  if(dist <= m_handoff_radius) {
    // Handoff completion signal for target tasks: requester is told to return.
    notifyRequesterReturn(m_active_lock.requester, m_active_lock.task_hash);
    m_active_lock.handoff_sent = true;
  }
}

//---------------------------------------------------------
// Procedure: maybeMaintainActiveLock()
//   Refreshes, completes, or times out the one active replacement commitment.

void RefuelReplace::maybeMaintainActiveLock()
{
  if(!activeLockEngaged())
    return;

  // Optional timeout guard against stale lock.
  if((m_replacement_lock_timeout > 0.0) &&
     ((MOOSTime() - m_active_lock.last_refresh_utc) > m_replacement_lock_timeout)) {
    clearActiveReplacementLock("timeout");
  }
  else {
    // Identify "basic-like" tasks that have no handoff target. For these tasks
    // we hold lock for the entire return leg rather than clearing on RETURN=true.
    const bool basic_like_task =
      activeTaskUsesReturnReset(m_active_lock.task_type, m_active_lock.target_set);

    // Mark that the active basic-like winner has started its return leg.
    if(basic_like_task && m_returning_mode)
      m_active_lock.return_started = true;

    // Any task with a completed handoff can release immediately.
    if(m_active_lock.handoff_sent) {
      clearActiveReplacementLock("handoff_complete");
    }
    // Basic-like tasks only release once return completes (ODOMETRY_RESET path).
    // This keeps REFUEL_TRANSIT_BUSY asserted while actively returning.
    else if(basic_like_task &&
            m_active_lock.return_started &&
            m_active_lock.odom_reset_seen) {
      if(m_active_lock.task_type == "refuelreplace_basic")
        clearActiveReplacementLock("basic_return_complete");
      else
        clearActiveReplacementLock("unknown_basic_return_complete");
    }
  }
}

//---------------------------------------------------------
// Procedure: prunePendingTasks()
//   Drops old non-active pending-task metadata so the passive map stays small.

void RefuelReplace::prunePendingTasks(double now)
{
  // Normal cleanup is event-driven on bid terminal states. This short TTL is
  // only a fallback for orphaned pending entries if expected closeout mail
  // never arrives.
  const double prune_age = 120.0; // seconds

  map<string, PendingTaskInfo>::iterator p = m_pending_tasks.begin();
  while(p != m_pending_tasks.end()) {
    if(activeLockEngaged() && (p->first == m_active_lock.task_hash)) {
      ++p;
      continue;
    }

    const double age = now - p->second.last_seen_utc;
    if((p->second.last_seen_utc > 0.0) && (age > prune_age))
      p = m_pending_tasks.erase(p);
    else
      ++p;
  }
}

//---------------------------------------------------------
// Procedure: upsertTaskInfo()
//   Parses task-spec mail into the passive pending-task metadata map.

// Parse a TASK_REFUEL_TARGET or TASK_REFUEL_BASIC message and update the
// corresponding pending-task entry. If the same hash is already active,
// copy any late-arriving metadata into the active lock as well.
void RefuelReplace::upsertTaskInfo(const string& task_msg)
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

  PendingTaskInfo& rec = m_pending_tasks[hash];
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

  string pweight = stripBlankEnds(tokStringParse(spec, "priority_weight"));
  if(isNumber(pweight))
    rec.priority_weight = atof(pweight.c_str());

  rec.last_seen_utc = MOOSTime();

  if(activeLockEngaged() && (m_active_lock.task_hash == hash)) {
    if(rec.requester != "")
      m_active_lock.requester = rec.requester;
    if(rec.task_type != "")
      m_active_lock.task_type = rec.task_type;
    if(rec.target_set) {
      m_active_lock.target_x = rec.target_x;
      m_active_lock.target_y = rec.target_y;
      m_active_lock.target_set = true;
      if(m_active_lock.task_type == "")
        m_active_lock.task_type = "refuelreplace_target";
    }
  }
}

//---------------------------------------------------------
// Procedure: processTaskState()
//   Interprets bid outcomes for local commitment tracking.
//   Winner identity is inferred from payload keys first, then mail metadata.
//   This method is the sole owner of active-lock acquisition/release transitions.

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

  PendingTaskInfo& rec = m_pending_tasks[hash];
  if(id != "")
    rec.id = id;

  if(rec.requester == "")
    rec.requester = inferRequesterFromId(rec.id);

  const string host_lc = tolower(stripBlankEnds(m_host_community));
  const bool own_request =
    (tolower(stripBlankEnds(rec.requester)) == host_lc);

  string winner = parseTaskStateWinner(spec);
  // Fallback path for TASK_STATE payloads that only include id/hash/state.
  // In these cases source community corresponds to the local helm node.
  if((winner == "") && (msg_source_community != ""))
    winner = msg_source_community;
  if((winner == "") && (tolower(stripBlankEnds(msg_source_app)) == "ptaskmanager"))
    winner = m_host_community;

  rec.last_seen_utc = MOOSTime();

  if(state == "bidwon") {
    if(winner == "") {
      // Without a winner identity we cannot safely claim this bid as ours.
      return;
    }

    string winner_lc = tolower(stripBlankEnds(winner));
    if((winner != "") && (winner_lc == host_lc)) {
      if(!activeLockEngaged()) {
        // First accepted win acquires the replacement lock.
        m_active_lock.task_hash = hash;
        m_active_lock.requester = rec.requester;
        m_active_lock.task_type = rec.task_type;
        m_active_lock.target_x = rec.target_x;
        m_active_lock.target_y = rec.target_y;
        m_active_lock.target_set = rec.target_set;
        // Fallback inference if task type arrives late.
        if((m_active_lock.task_type == "") && rec.target_set)
          m_active_lock.task_type = "refuelreplace_target";
        m_active_lock.acquired_utc = MOOSTime();
        m_active_lock.last_refresh_utc = MOOSTime();
        m_active_lock.handoff_sent = false;
        m_active_lock.return_started = false;
        m_active_lock.odom_reset_seen = false;
        m_pending_tasks.erase(hash);
      }
      else if(m_active_lock.task_hash == hash) {
        // Repeated status update for same task: refresh lock freshness.
        if((m_active_lock.requester == "") && (rec.requester != ""))
          m_active_lock.requester = rec.requester;
        if((m_active_lock.task_type == "") && (rec.task_type != ""))
          m_active_lock.task_type = rec.task_type;
        if(rec.target_set) {
          m_active_lock.target_x = rec.target_x;
          m_active_lock.target_y = rec.target_y;
          m_active_lock.target_set = true;
        }
        if((m_active_lock.task_type == "") && m_active_lock.target_set)
          m_active_lock.task_type = "refuelreplace_target";
        m_active_lock.last_refresh_utc = MOOSTime();
      }
      else {
        // Ignore additional wins while already committed to another task.
        reportRunWarning("Concurrent bidwon while lock held. held_hash=" +
                         m_active_lock.task_hash + ", new_hash=" + hash);
        m_pending_tasks.erase(hash);
      }
    }
    else {
      if(m_active_lock.task_hash == hash)
        clearActiveReplacementLock("task_state_bidwon_other_winner");
      m_pending_tasks.erase(hash);
    }

    if(own_request) {
      m_pending_retry_task = PendingTaskInfo();
      m_pending_retry_utc = 0.0;
      m_retry_attempts_used = 0;
    }
  }
  else if(state == "unawarded") {
    if(m_active_lock.task_hash == hash)
      clearActiveReplacementLock("task_state_unawarded");

    if(own_request && (m_retry_attempts_used < m_unawarded_retry_limit)) {
      m_pending_retry_task = rec;
      if(m_pending_retry_task.requester == "")
        m_pending_retry_task.requester = m_host_community;
      m_retry_attempts_used++;
      m_pending_retry_utc = MOOSTime() + m_unawarded_retry_delay;
      reportEvent("Queued unawarded retry attempt " +
                  uintToString(m_retry_attempts_used) + "/" +
                  uintToString(m_unawarded_retry_limit) +
                  " for hash=" + hash);
    }
    else if(own_request) {
      m_pending_retry_task = PendingTaskInfo();
      m_pending_retry_utc = 0.0;
      reportRunWarning("Unawarded retry limit reached for hash=" + hash);
    }

    m_pending_tasks.erase(hash);
  }
  else if((state == "bidlost") || (state == "abstain")) {
    // If the active task transitions out of bidwon, release commitment.
    if(m_active_lock.task_hash == hash)
      clearActiveReplacementLock("task_state_" + state);
    if(own_request) {
      m_pending_retry_task = PendingTaskInfo();
      m_pending_retry_utc = 0.0;
      m_retry_attempts_used = 0;
    }
    m_pending_tasks.erase(hash);
  }
}

//---------------------------------------------------------
// Procedure: clearActiveReplacementLock()
//   Releases the active commitment state and logs the reason.

void RefuelReplace::clearActiveReplacementLock(const string& reason)
{
  if(!activeLockEngaged())
    return;

  // Keep reason in event log for post-run debugging of lock lifecycle.
  reportEvent("Clearing replacement lock hash=" + m_active_lock.task_hash +
              ", reason=" + reason);
  m_pending_tasks.erase(m_active_lock.task_hash);
  m_active_lock.clear();
}

//---------------------------------------------------------
// Procedure: sendNodeMessage()
//   Convenience wrapper for sending a string NODE_MESSAGE_LOCAL payload.

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
//   Emits requester-directed control messages that complete a target-task
//   handoff. This is only called for locally won target tasks after winner
//   enters the configured handoff radius.

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
  table << "pending_tasks"         << uintToString((unsigned int)m_pending_tasks.size());
  bool transit_busy = activeLockEngaged();
  table << "refuel_transit_busy"   << (transit_busy ? "true" : "false");
  table << "returning_mode"        << (m_returning_mode ? "true" : "false");
  table << "lock_hash"             << (m_active_lock.task_hash == "" ?
                                        "(none)" : m_active_lock.task_hash);
  table << "lock_requester"        << (m_active_lock.requester == "" ?
                                        "(none)" : m_active_lock.requester);
  table << "lock_type"             << (m_active_lock.task_type == "" ?
                                        "(none)" : m_active_lock.task_type);
  table << "lock_target_set"       << (m_active_lock.target_set ? "true" : "false");
  table << "lock_return_started"   <<
             (m_active_lock.return_started ? "true" : "false");
  table << "lock_odom_reset_seen"  <<
             (m_active_lock.odom_reset_seen ? "true" : "false");
  table << "lock_timeout_s"        << doubleToStringX(m_replacement_lock_timeout, 2);
  table << "pending_discovery_fire" << (m_pending_discovery_fire_id == "" ?
                                          "(none)" : m_pending_discovery_fire_id);
  table << "discovery_timeout_s"    << doubleToStringX(m_discovery_request_timeout, 2);
  table << "discovery_cooldown_s"   << doubleToStringX(m_discovery_repost_cooldown, 2);
  table << "unawarded_retry_delay_s" << doubleToStringX(m_unawarded_retry_delay, 2);
  table << "unawarded_retry_limit"   << uintToString(m_unawarded_retry_limit);
  table << "unawarded_retry_used"    << uintToString(m_retry_attempts_used);
  table << "pending_retry_task"      << (m_pending_retry_task.id == "" ?
                                          "(none)" : m_pending_retry_task.id);

  m_msgs << table.getFormattedString();
  return true;
}
