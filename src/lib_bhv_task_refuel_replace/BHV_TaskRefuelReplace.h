/*****************************************************************/
/*    NAME: Charles Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: BHV_TaskRefuelReplace.h                              */
/*    DATE: Jan 31st 2026                                        */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/
 
#ifndef BHV_TASK_REFUEL_REPLACE_HEADER
#define BHV_TASK_REFUEL_REPLACE_HEADER

#include <string>
#include <vector>
#include "IvPTaskBehavior.h"

class IvPDomain;
class BHV_TaskRefuelReplace : public IvPTaskBehavior {
public:
  BHV_TaskRefuelReplace(IvPDomain);
  ~BHV_TaskRefuelReplace() {}

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

  // Region being bid on (from MISSION_TASK details)
  double m_region_x;
  double m_region_y;
  bool   m_region_x_set;
  bool   m_region_y_set;

  // Priority weight of the task region (from MISSION_TASK details)
  double m_priority_weight;
  std::string m_requester;      // requester vehicle (optional passthrough)

  // Bid formula tuning (from .bhv config)
  double m_planning_horizon;    // H  (seconds)
  double m_opw;                 // opportunity-cost weight

 protected:  // State vars (read from MOOSDB)

  double m_fuel_dist_remaining;
  bool   m_got_fuel;

  double m_own_region_weight;   // 0 if not loitering an AOI
  bool   m_got_own_region_weight;
};

#ifdef WIN32
   #define IVP_EXPORT_FUNCTION __declspec(dllexport) 
#else
   #define IVP_EXPORT_FUNCTION
#endif

extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain) 
  {return new BHV_TaskRefuelReplace(domain);}
}
#endif
