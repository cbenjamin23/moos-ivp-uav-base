/************************************************************/
/*    NAME: Steve Nomeny                                              */
/*    ORGN: MIT                                             */
/*    FILE: BHV_ZigLeg.cpp                                    */
/*    DATE: Mars 19 2024                                             */
/************************************************************/

#include <iterator>
#include <cstdlib>
#include "MBUtils.h"
#include "BuildUtils.h"
#include "BHV_ZigLeg.h"

#include "ZAIC_PEAK.h"
#include "AngleUtils.h"

using namespace std;

//---------------------------------------------------------------
// Constructor

BHV_ZigLeg::BHV_ZigLeg(IvPDomain domain) :
  IvPBehavior(domain)
{
  // Provide a default behavior name
  IvPBehavior::setParam("name", "defaultname");

  // Declare the behavior decision space
  m_domain = subDomain(m_domain, "course");

  // Add any variables this behavior needs to subscribe for
  addInfoVars("NAV_X, NAV_Y, NAV_HEADING");
  addInfoVars("WPT_INDEX", "no_warning");


  // Default values for configuration parameters
  m_zig_duration = 5;
  m_zig_anlge = 10;

}

//---------------------------------------------------------------
// Procedure: setParam()

bool BHV_ZigLeg::setParam(string param, string val)
{
  // Convert the parameter to lower case for more general matching
  param = tolower(param);

  // Get the numerical value of the param argument for convenience once
  double double_val = atof(val.c_str());
  
  if((param == "zig_angle") && isNumber(val)) {
    m_zig_anlge = double_val;
    return(true);
  }
  else if (param == "zig_duration" && isNumber(val)) {
    m_zig_duration = double_val;
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

void BHV_ZigLeg::onSetParamComplete()
{
}

//---------------------------------------------------------------
// Procedure: onHelmStart()
//   Purpose: Invoked once upon helm start, even if this behavior
//            is a template and not spawned at startup

void BHV_ZigLeg::onHelmStart()
{
}

//---------------------------------------------------------------
// Procedure: onIdleState()
//   Purpose: Invoked on each helm iteration if conditions not met.

void BHV_ZigLeg::onIdleState()
{
}

//---------------------------------------------------------------
// Procedure: onCompleteState()

void BHV_ZigLeg::onCompleteState()
{
}

//---------------------------------------------------------------
// Procedure: postConfigStatus()
//   Purpose: Invoked each time a param is dynamically changed

void BHV_ZigLeg::postConfigStatus()
{
}

//---------------------------------------------------------------
// Procedure: onIdleToRunState()
//   Purpose: Invoked once upon each transition from idle to run state

void BHV_ZigLeg::onIdleToRunState()
{
}

//---------------------------------------------------------------
// Procedure: onRunToIdleState()
//   Purpose: Invoked once upon each transition from run to idle state

void BHV_ZigLeg::onRunToIdleState()
{
}

//---------------------------------------------------------------
// Procedure: onRunState()
//   Purpose: Invoked each iteration when run conditions have been met.

IvPFunction* BHV_ZigLeg::onRunState()
{
  bool ok1, ok2, ok3;
  m_osx = getBufferDoubleVal("NAV_X", ok1);
  m_osy = getBufferDoubleVal("NAV_Y", ok2);
  if(!ok1 || !ok2) {
    postWMessage("No ownship X/Y info in info_buffer.");
    return(0);
  }
  m_osh = getBufferDoubleVal("NAV_HEADING", ok3);
  if(!ok3){
    postWMessage("No ownship heading info in info_buffer.");
    return(0);
  }


  
  static double wpt_index = 0; //uninitilized
  static bool should_zig = false;
  
  static double zig_heading = 0;
  static bool heading_fixed = false;
  
  if(getBufferDoubleVal("WPT_INDEX") != wpt_index) {
    wpt_index = getBufferDoubleVal("WPT_INDEX");

    m_zig_time = getBufferCurrTime() + 5;
    should_zig = true; 
  }
 
  // Part 1: Build the IvP function
  IvPFunction *ipf = 0;

  if (getBufferCurrTime() >= m_zig_time && should_zig) {
    // set the zig heading here
    if(!heading_fixed) {
      zig_heading = m_osh + m_zig_anlge;
      zig_heading = angle360(zig_heading);
      heading_fixed = true;
    }

    ZAIC_PEAK course_zaic(m_domain, "course");
    course_zaic.setSummit(zig_heading);
    course_zaic.setPeakWidth(10);
    course_zaic.setBaseWidth(20);
    course_zaic.setSummitDelta(2); 

    if(course_zaic.stateOK() == false) {
      string warnings = "Speed ZAIC problems " + course_zaic.getWarnings();
      postWMessage(warnings);
      return(0);
    }    

    ipf = course_zaic.extractIvPFunction();

    // Part N: Prior to returning the IvP function, apply the priority wt
    // Actual weight applied may be some value different than the configured
    // m_priority_wt, depending on the behavior author's insite.
    if(ipf)
      ipf->setPWT(m_priority_wt);


    if(getBufferCurrTime() >= m_zig_time + m_zig_duration) {
      should_zig = false;
      heading_fixed = false;
    }

  }
  
  
  
  return(ipf);
}


