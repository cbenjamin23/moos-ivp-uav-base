/*****************************************************************/
/*    NAME: Charles Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: BHV_TaskRefuelReplaceBasic.cpp                       */
/*    DATE: Feb 15th 2026                                        */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#include <cstdlib>
#include <cmath>
#include <algorithm>
#include "BHV_TaskRefuelReplaceBasic.h"
#include "RefuelBidReservation.h"
#include "MBUtils.h"

using namespace std;

//-----------------------------------------------------------
// Constructor

BHV_TaskRefuelReplaceBasic::BHV_TaskRefuelReplaceBasic(IvPDomain domain) :
  IvPTaskBehavior(domain)
{
  m_distance_tiebreak_weight = 0.001;
  m_fuel_abstain_threshold   = 0;
  m_requester_x              = 0;
  m_requester_y              = 0;
  m_requester_x_set          = false;
  m_requester_y_set          = false;

  m_fuel_dist_remaining = 0;
  m_got_fuel_input      = false;
  m_returning_mode      = false;
  m_refuel_transit_busy = false;

  addInfoVars("FUEL_DISTANCE_REMAINING", "no_warning");
  addInfoVars("RETURN",                  "no_warning");
  addInfoVars("REFUEL_TRANSIT_BUSY",     "no_warning");
}

//-----------------------------------------------------------
// Procedure: onHelmStart()

void BHV_TaskRefuelReplaceBasic::onHelmStart()
{
  string alert_request = "type=" + m_task_type;
  alert_request += ", var=" + m_update_var;
  postMessage("TM_ALERT_REQUEST", alert_request);
}

//-----------------------------------------------------------
// Procedure: setParam()

bool BHV_TaskRefuelReplaceBasic::setParam(string param, string value)
{
  if(IvPTaskBehavior::setParam(param, value))
    return(true);

  param = tolower(param);

  if(param == "distance_tiebreak_weight")
    return(setNonNegDoubleOnString(m_distance_tiebreak_weight, value));
  else if(param == "fuel_abstain_threshold")
    return(setNonNegDoubleOnString(m_fuel_abstain_threshold, value));
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

  return(false);
}

//-----------------------------------------------------------
// Procedure: updatePlatformInfo()

bool BHV_TaskRefuelReplaceBasic::updatePlatformInfo()
{
  bool ok;

  double fuel = getBufferDoubleVal("FUEL_DISTANCE_REMAINING", ok);
  if(ok) {
    m_fuel_dist_remaining = fuel;
    m_got_fuel_input = true;
  }

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

void BHV_TaskRefuelReplaceBasic::onIdleState()
{
  IvPTaskBehavior::onGeneralIdleState();
}

//-----------------------------------------------------------
// Procedure: onRunState()

IvPFunction *BHV_TaskRefuelReplaceBasic::onRunState()
{
  updatePlatformInfo();
  IvPTaskBehavior::onGeneralRunState();

  // Shared in-process reservation lifecycle across basic + target behaviors.
  const double now = getBufferCurrTime();
  RefuelBidReservation::maintainForState(m_task_hash, m_task_state, now);

  return(0);
}

//-----------------------------------------------------------
// Procedure: isTaskFeasible()

bool BHV_TaskRefuelReplaceBasic::isTaskFeasible()
{
  bool feasible = true;
  const double now = getBufferCurrTime();

  // If another replacement task (basic or target) on this same vehicle is
  // already bidding/won, abstain this task to avoid double-award races.
  if(RefuelBidReservation::heldByOther(m_task_hash, now))
    feasible = false;

  if(m_returning_mode)
    feasible = false;

  if(m_refuel_transit_busy && (m_task_state != "bidwon"))
    feasible = false;

  if(!m_got_fuel_input)
    feasible = false;

  if(m_fuel_dist_remaining < m_fuel_abstain_threshold)
    feasible = false;

  // Claim early when feasible so sibling task behaviors in the same helm cycle
  // see this reservation before also placing bids.
  const std::string state = tolower(stripBlankEnds(m_task_state));
  if(feasible && ((state == "bidding") || (state == "bidwon")))
    RefuelBidReservation::claim(m_task_hash, now);

  return(feasible);
}

//-----------------------------------------------------------
// Procedure: getTaskBid()
//
// Primary factor: larger remaining fuel wins.
// Tie-breaker: very small penalty for farther distance to requester.

double BHV_TaskRefuelReplaceBasic::getTaskBid()
{
  double tie_dist = hypot(m_osx, m_osy);
  if(m_requester_x_set && m_requester_y_set)
    tie_dist = hypot(m_osx - m_requester_x, m_osy - m_requester_y);
  double score = m_fuel_dist_remaining - (m_distance_tiebreak_weight * tie_dist);
  return(max(0.0, score));
}

//-----------------------------------------------------------
// Procedure: applyFlagMacros()

vector<VarDataPair> BHV_TaskRefuelReplaceBasic::applyFlagMacros(vector<VarDataPair> flags)
{
  return(flags);
}
