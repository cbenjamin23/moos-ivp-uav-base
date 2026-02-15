/*****************************************************************/
/*    NAME: Charles Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: BHV_TaskRefuelReplaceTarget.h                              */
/*    DATE: Jan 31st 2026                                        */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/
 
#ifndef BHV_TASK_REFUEL_REPLACE_TARGET_HEADER
#define BHV_TASK_REFUEL_REPLACE_TARGET_HEADER

#include <string>
#include <vector>
#include "IvPTaskBehavior.h"

class IvPDomain;
class BHV_TaskRefuelReplaceTarget : public IvPTaskBehavior {
public:
  BHV_TaskRefuelReplaceTarget(IvPDomain);
  ~BHV_TaskRefuelReplaceTarget() {}

  // IvPTaskBehavior virtuals
  void   onHelmStart();
  double getTaskBid();
  bool   isTaskFeasible();
  bool   setParam(std::string, std::string);

  std::vector<VarDataPair> applyFlagMacros(std::vector<VarDataPair>);

  void         onIdleState();
  IvPFunction* onRunState();

 protected:
  bool updatePlatformInfo();

 protected:  // Config params

  // Target being bid on (from MISSION_TASK details)
  double m_target_x;
  double m_target_y;
  bool   m_target_x_set;
  bool   m_target_y_set;

  // Priority weight of the task target (from MISSION_TASK details)
  double m_priority_weight;
  std::string m_requester;      // requester vehicle (optional passthrough)
  double m_requester_x;
  double m_requester_y;
  bool   m_requester_x_set;
  bool   m_requester_y_set;

  // Bid formula tuning (from .bhv config)
  double m_planning_horizon;    // H  (seconds)
  double m_opw;                 // opportunity-cost weight
  double m_fuel_abstain_threshold; // abstain if fuel remaining is below this

 protected:  // State vars (read from MOOSDB)

  double m_fuel_dist_remaining;
  bool   m_got_fuel_input;
  bool   m_returning_mode;
  bool   m_refuel_transit_busy;

  double m_own_target_weight;   // 0 if not loitering an AOI
  bool   m_got_own_target_weight;
};

#ifdef WIN32
   #define IVP_EXPORT_FUNCTION __declspec(dllexport) 
#else
   #define IVP_EXPORT_FUNCTION
#endif

extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain) 
  {return new BHV_TaskRefuelReplaceTarget(domain);}
}
#endif
