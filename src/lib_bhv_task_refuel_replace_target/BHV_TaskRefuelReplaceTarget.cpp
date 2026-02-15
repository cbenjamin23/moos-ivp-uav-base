/*****************************************************************/
/*    NAME: Charles Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: BHV_TaskRefuelReplaceTarget.cpp                            */
/*    DATE: Jan 31st 2026                                        */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#include <cstdlib>
#include <cmath>
#include <algorithm>
#include "BHV_TaskRefuelReplaceTarget.h"
#include "MBUtils.h"
#include "MacroUtils.h"

using namespace std;

//-----------------------------------------------------------
// Constructor

BHV_TaskRefuelReplaceTarget::BHV_TaskRefuelReplaceTarget(IvPDomain domain) :
  IvPTaskBehavior(domain)
{
  m_target_x     = 0;
  m_target_y     = 0;
  m_target_x_set = false;
  m_target_y_set = false;

  m_priority_weight   = 1.0;
  m_requester         = "";
  m_requester_x       = 0;
  m_requester_y       = 0;
  m_requester_x_set   = false;
  m_requester_y_set   = false;

  m_planning_horizon  = 600;    // 10 min
  m_opw               = 0.3;    // opportunity cost weight
  m_fuel_abstain_threshold = 0; // default disabled

  m_fuel_dist_remaining   = 0;
  m_got_fuel_input        = false;
  m_returning_mode        = false;
  m_refuel_transit_busy   = false;
  m_own_target_weight     = 0;
  m_got_own_target_weight = false;

  addInfoVars("FUEL_DISTANCE_REMAINING", "no_warning");
  addInfoVars("OWN_TARGET_WEIGHT",       "no_warning");
  addInfoVars("RETURN",                  "no_warning");
  addInfoVars("REFUEL_TRANSIT_BUSY",     "no_warning");
}

//-----------------------------------------------------------
// Procedure: onHelmStart()

void BHV_TaskRefuelReplaceTarget::onHelmStart()
{
  string alert_request = "type=" + m_task_type;
  alert_request += ", var=" + m_update_var;
  postMessage("TM_ALERT_REQUEST", alert_request);
}

//-----------------------------------------------------------
// Procedure: setParam()

bool BHV_TaskRefuelReplaceTarget::setParam(string param, string value) 
{
  if(IvPTaskBehavior::setParam(param, value))
    return(true);
  
  param = tolower(param);

  // Target point (from MISSION_TASK spawn details)
  if((param == "target_x") && isNumber(value)) {
    m_target_x = atof(value.c_str());
    m_target_x_set = true;
    return(true);
  }
  else if((param == "target_y") && isNumber(value)) {
    m_target_y = atof(value.c_str());
    m_target_y_set = true;
    return(true);
  }
  else if(param == "null")
    return(true);

  // Priority weight (from MISSION_TASK spawn details)
  else if(param == "priority_weight")
    return(setNonNegDoubleOnString(m_priority_weight, value));
  else if(param == "requester") {
    m_requester = value;
    return(true);
  }
  else if((param == "requester_x") && isNumber(value)) {
    m_requester_x = atof(value.c_str());
    m_requester_x_set = true;
    return(true);
  }
  else if((param == "requester_y") && isNumber(value)) {
    m_requester_y = atof(value.c_str());
    m_requester_y_set = true;
    return(true);
  }

  // Bid formula tuning (from .bhv config)
  else if(param == "planning_horizon")
    return(setNonNegDoubleOnString(m_planning_horizon, value));
  else if(param == "opw")
    return(setNonNegDoubleOnString(m_opw, value));
  else if(param == "fuel_abstain_threshold")
    return(setNonNegDoubleOnString(m_fuel_abstain_threshold, value));

  return(false);
}

//-----------------------------------------------------------
// Procedure: updatePlatformInfo()

bool BHV_TaskRefuelReplaceTarget::updatePlatformInfo()
{
  bool ok;

  double fuel = getBufferDoubleVal("FUEL_DISTANCE_REMAINING", ok);
  if(ok) {
    m_fuel_dist_remaining = fuel;
    m_got_fuel_input = true;
  }

  double orw = getBufferDoubleVal("OWN_TARGET_WEIGHT", ok);
  if(ok) {
    m_own_target_weight = orw;
    m_got_own_target_weight = true;
  }

  // RETURN is typically posted as a string bool, but handle either
  // string or numeric payload robustly.
  string ret_str = getBufferStringVal("RETURN", ok);
  if(ok) {
    ret_str = tolower(stripBlankEnds(ret_str));
    if((ret_str == "true") || (ret_str == "1") || (ret_str == "on"))
      m_returning_mode = true;
    else if((ret_str == "false") || (ret_str == "0") || (ret_str == "off"))
      m_returning_mode = false;
  }
  else {
    double ret_num = getBufferDoubleVal("RETURN", ok);
    if(ok)
      m_returning_mode = (ret_num != 0.0);
  }

  // When this vehicle has already won a replacement task and is still
  // transiting to the handoff point, skip bidding on additional tasks.
  string transit_str = getBufferStringVal("REFUEL_TRANSIT_BUSY", ok);
  if(ok) {
    transit_str = tolower(stripBlankEnds(transit_str));
    if((transit_str == "true") || (transit_str == "1") || (transit_str == "on"))
      m_refuel_transit_busy = true;
    else if((transit_str == "false") || (transit_str == "0") || (transit_str == "off"))
      m_refuel_transit_busy = false;
  }
  else {
    double transit_num = getBufferDoubleVal("REFUEL_TRANSIT_BUSY", ok);
    if(ok)
      m_refuel_transit_busy = (transit_num != 0.0);
  }

  return(m_got_fuel_input);
}

//-----------------------------------------------------------
// Procedure: onIdleState()

void BHV_TaskRefuelReplaceTarget::onIdleState()
{
  IvPTaskBehavior::onGeneralIdleState();
}

//-----------------------------------------------------------
// Procedure: onRunState()

IvPFunction *BHV_TaskRefuelReplaceTarget::onRunState()
{
  const bool was_bidwon = (m_task_state == "bidwon");
  updatePlatformInfo();
  IvPTaskBehavior::onGeneralRunState();

  // Ensure target ownership vars are posted when this behavior wins.
  // This avoids relying solely on bidwonflag macro handling.
  if(!was_bidwon && (m_task_state == "bidwon")) {
    postRepeatableMessage("OWN_TARGET_X", m_target_x);
    postRepeatableMessage("OWN_TARGET_Y", m_target_y);
    postRepeatableMessage("OWN_TARGET_WEIGHT", m_priority_weight);
  }

  return(0);
}

//-----------------------------------------------------------
// Procedure: isTaskFeasible()
//   Can we reach the target with any fuel left?
//   If not, the superclass sends an abstain to all allies.

bool BHV_TaskRefuelReplaceTarget::isTaskFeasible()
{
  bool feasible = true;

  // If we're already in returning mode, we shouldn't be bidding on targets
  if(m_returning_mode)
    feasible = false;

  // If we already won another task and are still in transit to that handoff,
  // abstain from new auctions (but keep servicing the currently won task).
  if(m_refuel_transit_busy && (m_task_state != "bidwon"))
    feasible = false;

  // If we haven't received fuel info yet, assume not feasible to avoid bidding on targets
  if(!m_got_fuel_input)
    feasible = false;

  // Abstain if we are below the fuel threshold, if configured. This is a tuning
  if(m_fuel_dist_remaining < m_fuel_abstain_threshold)
    feasible = false;

  // Check if we have enough fuel to reach the target at all. This is a sanity
  if(feasible) {
    double dist = hypot(m_osx - m_target_x, m_osy - m_target_y);
    feasible = (m_fuel_dist_remaining > dist);
  }

  return(feasible);
}

//-----------------------------------------------------------
// Procedure: getTaskBid()
//
//  BidScore = w_k * max(0, min(H, T_loiter) - tau_k)
//             - opw * OWN_TARGET_WEIGHT
//
//  Treating distance and time interchangeably (speed = 1 m/s
//  equivalent): tau_k = dist, T_loiter = fuel_remaining - dist.
//  Only called after isTaskFeasible() returned true.

double BHV_TaskRefuelReplaceTarget::getTaskBid()
{
  // TEMPORARY SIMPLIFIED BIDDING:
  // Match the basic behavior style while keeping this behavior path intact.
  const double distance_tiebreak_weight = 0.1;
  double tie_dist = hypot(m_osx, m_osy);
  if(m_requester_x_set && m_requester_y_set)
    tie_dist = hypot(m_osx - m_requester_x, m_osy - m_requester_y);
  double score = m_fuel_dist_remaining - (distance_tiebreak_weight * tie_dist);

  /*
  // Original target-aware bid logic (kept for easy restore):
  double dist = hypot(m_osx - m_target_x, m_osy - m_target_y);

  // tau_k: "time" to reach target (= distance, since dist ≡ time)
  double tau_k = dist;

  // T_loiter: how long we can stay after arriving
  double T_loiter = m_fuel_dist_remaining - dist;

  // Value of serving target k
  double term1 = m_priority_weight * max(0.0, min(m_planning_horizon, T_loiter) - tau_k);

  // Cost of leaving own target
  double term2 = m_opw * m_own_target_weight;

  double score = term1 - term2;
  */

  return(max(0.0, score));
}

//-----------------------------------------------------------
// Procedure: applyFlagMacros()

vector<VarDataPair> BHV_TaskRefuelReplaceTarget::applyFlagMacros(vector<VarDataPair> flags)
{
  string rx_str = doubleToStringX(m_target_x, 2);
  string ry_str = doubleToStringX(m_target_y, 2);

  // This is so our .bhv config can use TARGET_X and TARGET_Y 
  for(unsigned int i=0; i<flags.size(); i++) {
    if(flags[i].is_string()) {
      string sdata = flags[i].get_sdata();
      sdata = macroExpand(sdata, "TARGET_X", rx_str);
      sdata = macroExpand(sdata, "TARGET_Y", ry_str);
      sdata = macroExpand(sdata, "PRIORITY_WEIGHT",
                          doubleToStringX(m_priority_weight, 2));
      flags[i].set_sdata(sdata, true);
    }
  }

  return(flags);
}
