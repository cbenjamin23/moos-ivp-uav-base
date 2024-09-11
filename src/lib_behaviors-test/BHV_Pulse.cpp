/************************************************************/
/*    NAME: Steve Nomeny                                              */
/*    ORGN: MIT                                             */
/*    FILE: BHV_Pulse.cpp                                    */
/*    DATE: Mars 19 2024                                             */
/************************************************************/

#include <iterator>
#include <cstdlib>
#include "MBUtils.h"
#include "BuildUtils.h"
#include "BHV_Pulse.h"
#include "XYRangePulse.h"

using namespace std;

//---------------------------------------------------------------
// Constructor

BHV_Pulse::BHV_Pulse(IvPDomain domain) :
  IvPBehavior(domain)
{
  // Provide a default behavior name
  IvPBehavior::setParam("name", "defaultname");

  // Declare the behavior decision space
  m_domain = subDomain(m_domain, "course,speed");

  // Add any variables this behavior needs to subscribe for
  addInfoVars("NAV_X, NAV_Y");
  addInfoVars("WPT_INDEX", "no_warning");

  // Default values for configuration parameters
  m_pulse_range = 10;
  m_pulse_duration = 2;

}

//---------------------------------------------------------------
// Procedure: setParam()

bool BHV_Pulse::setParam(string param, string val)
{
  // Convert the parameter to lower case for more general matching
  param = tolower(param);

  // Get the numerical value of the param argument for convenience once
  double double_val = atof(val.c_str());
  
  if((param == "pulse_range") && isNumber(val)) {
    m_pulse_range = double_val;
    return(true);
  }
  else if (param == "pulse_duration" && isNumber(val)) {
    m_pulse_duration = double_val;
    return(true);
  }

  // If not handled above, then just return false;
  return(false);
}

//---------------------------------------------------------------
// Procedure: onSetParamComplete()
//   Purpose: Invoked once after all parameters have been handled.
//            Good place to ensure all required params have are set.
//            Or any inter-param relationships like a<b.

void BHV_Pulse::onSetParamComplete()
{
}

//---------------------------------------------------------------
// Procedure: onHelmStart()
//   Purpose: Invoked once upon helm start, even if this behavior
//            is a template and not spawned at startup

void BHV_Pulse::onHelmStart()
{
}

//---------------------------------------------------------------
// Procedure: onIdleState()
//   Purpose: Invoked on each helm iteration if conditions not met.

void BHV_Pulse::onIdleState()
{
}

//---------------------------------------------------------------
// Procedure: onCompleteState()

void BHV_Pulse::onCompleteState()
{
}

//---------------------------------------------------------------
// Procedure: postConfigStatus()
//   Purpose: Invoked each time a param is dynamically changed

void BHV_Pulse::postConfigStatus()
{
}

//---------------------------------------------------------------
// Procedure: onIdleToRunState()
//   Purpose: Invoked once upon each transition from idle to run state

void BHV_Pulse::onIdleToRunState()
{
}

//---------------------------------------------------------------
// Procedure: onRunToIdleState()
//   Purpose: Invoked once upon each transition from run to idle state

void BHV_Pulse::onRunToIdleState()
{
}

//---------------------------------------------------------------
// Procedure: onRunState()
//   Purpose: Invoked each iteration when run conditions have been met.

IvPFunction* BHV_Pulse::onRunState()
{
  bool ok1, ok2;
  m_osx = getBufferDoubleVal("NAV_X", ok1);
  m_osy = getBufferDoubleVal("NAV_Y", ok2);
  if(!ok1 || !ok2) {
    postWMessage("No ownship X/Y info in info_buffer.");
    return(0);
  }

  
  static double wpt_index = 0; //uninitilized
  static bool should_pulse = false;
  
  if(getBufferDoubleVal("WPT_INDEX") != wpt_index) {
    wpt_index = getBufferDoubleVal("WPT_INDEX");

    m_pulse_time = getBufferCurrTime() + 5;
    should_pulse = true; 
  }

  if (getBufferCurrTime() >= m_pulse_time && should_pulse) {
    postPulseMessage(m_pulse_time);
    should_pulse = false;
  }

 
  // Part 1: Build the IvP function
  IvPFunction *ipf = 0;


  // Part N: Prior to returning the IvP function, apply the priority wt
  // Actual weight applied may be some value different than the configured
  // m_priority_wt, depending on the behavior author's insite.
  if(ipf)
    ipf->setPWT(m_priority_wt);

  return(ipf);
}


void BHV_Pulse::postPulseMessage(double time) {
  XYRangePulse pulse;
  pulse.set_x(m_osx);               
  pulse.set_y(m_osy);                
  pulse.set_label("bhv_pulse");
  pulse.set_rad(m_pulse_range);
  pulse.set_time(time);       
  pulse.set_color("edge", "yellow");
  pulse.set_color("fill", "yellow");
  pulse.set_duration(m_pulse_duration);

  string spec = pulse.get_spec();
  postMessage("VIEW_RANGE_PULSE", spec);
}
