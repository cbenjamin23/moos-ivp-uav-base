/*****************************************************************/
/*    NAME: Charles Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: BHV_TaskRefuelReplace.cpp                            */
/*    DATE: Jan 31st 2026                                        */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#include <cstdlib>
#include <cmath>
#include <algorithm>
#include "BHV_TaskRefuelReplace.h"
#include "MBUtils.h"
#include "MacroUtils.h"

using namespace std;

//-----------------------------------------------------------
// Constructor

BHV_TaskRefuelReplace::BHV_TaskRefuelReplace(IvPDomain domain) :
  IvPTaskBehavior(domain)
{
  m_region_x     = 0;
  m_region_y     = 0;
  m_region_x_set = false;
  m_region_y_set = false;

  m_priority_weight   = 1.0;
  m_requester         = "";

  m_planning_horizon  = 600;    // 10 min
  m_opw               = 0.3;    // opportunity cost weight

  m_fuel_dist_remaining   = 0;
  m_got_fuel              = false;
  m_own_region_weight     = 0;
  m_got_own_region_weight = false;

  addInfoVars("FUEL_DISTANCE_REMAINING", "no_warning");
  addInfoVars("OWN_REGION_WEIGHT",       "no_warning");
}

//-----------------------------------------------------------
// Procedure: onHelmStart()

void BHV_TaskRefuelReplace::onHelmStart()
{
  string alert_request = "type=" + m_task_type;
  alert_request += ", var=" + m_update_var;
  postMessage("TM_ALERT_REQUEST", alert_request);
}

//-----------------------------------------------------------
// Procedure: setParam()

bool BHV_TaskRefuelReplace::setParam(string param, string value) 
{
  if(IvPTaskBehavior::setParam(param, value))
    return(true);
  
  param = tolower(param);

  // Region target (from MISSION_TASK spawn details)
  if((param == "region_x") && isNumber(value)) {
    m_region_x = atof(value.c_str());
    m_region_x_set = true;
    return(true);
  }
  else if((param == "region_y") && isNumber(value)) {
    m_region_y = atof(value.c_str());
    m_region_y_set = true;
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

  // Bid formula tuning (from .bhv config)
  else if(param == "planning_horizon")
    return(setNonNegDoubleOnString(m_planning_horizon, value));
  else if(param == "opw")
    return(setNonNegDoubleOnString(m_opw, value));

  return(false);
}

//-----------------------------------------------------------
// Procedure: updatePlatformInfo()

bool BHV_TaskRefuelReplace::updatePlatformInfo()
{
  bool ok;

  double fuel = getBufferDoubleVal("FUEL_DISTANCE_REMAINING", ok);
  if(ok) {
    m_fuel_dist_remaining = fuel;
    m_got_fuel = true;
  }

  double orw = getBufferDoubleVal("OWN_REGION_WEIGHT", ok);
  if(ok) {
    m_own_region_weight = orw;
    m_got_own_region_weight = true;
  }

  return(m_got_fuel);
}

//-----------------------------------------------------------
// Procedure: onIdleState()

void BHV_TaskRefuelReplace::onIdleState()
{
  IvPTaskBehavior::onGeneralIdleState();
}

//-----------------------------------------------------------
// Procedure: onRunState()

IvPFunction *BHV_TaskRefuelReplace::onRunState()
{
  updatePlatformInfo();
  IvPTaskBehavior::onGeneralRunState();
  return(0);
}

//-----------------------------------------------------------
// Procedure: isTaskFeasible()
//   Can we reach the region with any fuel left?
//   If not, the superclass sends an abstain to all allies.

bool BHV_TaskRefuelReplace::isTaskFeasible()
{
  if(!m_got_fuel)
    return(false);

  double dist = hypot(m_osx - m_region_x, m_osy - m_region_y);
  return(m_fuel_dist_remaining > dist);
}

//-----------------------------------------------------------
// Procedure: getTaskBid()
//
//  BidScore = w_k * max(0, min(H, T_loiter) - tau_k)
//             - opw * OWN_REGION_WEIGHT
//
//  Treating distance and time interchangeably (speed = 1 m/s
//  equivalent): tau_k = dist, T_loiter = fuel_remaining - dist.
//  Only called after isTaskFeasible() returned true.

double BHV_TaskRefuelReplace::getTaskBid()
{
  double dist = hypot(m_osx - m_region_x, m_osy - m_region_y);

  // tau_k: "time" to reach region (= distance, since dist ≡ time)
  double tau_k = dist;

  // T_loiter: how long we can stay after arriving
  double T_loiter = m_fuel_dist_remaining - dist;

  // Value of serving region k
  double term1 = m_priority_weight * max(0.0, min(m_planning_horizon, T_loiter) - tau_k);

  // Cost of leaving own region
  double term2 = m_opw * m_own_region_weight;

  return(max(0.0, term1 - term2));
}

//-----------------------------------------------------------
// Procedure: applyFlagMacros()

vector<VarDataPair> BHV_TaskRefuelReplace::applyFlagMacros(vector<VarDataPair> flags)
{
  string rx_str = doubleToStringX(m_region_x, 2);
  string ry_str = doubleToStringX(m_region_y, 2);

  // This is so our .bhv config can use REGION_X and REGION_Y 
  for(unsigned int i=0; i<flags.size(); i++) {
    if(flags[i].is_string()) {
      string sdata = flags[i].get_sdata();
      sdata = macroExpand(sdata, "REGION_X", rx_str);
      sdata = macroExpand(sdata, "REGION_Y", ry_str);
      sdata = macroExpand(sdata, "PRIORITY_WEIGHT",
                          doubleToStringX(m_priority_weight, 2));
      flags[i].set_sdata(sdata, true);
    }
  }

  return(flags);
}
