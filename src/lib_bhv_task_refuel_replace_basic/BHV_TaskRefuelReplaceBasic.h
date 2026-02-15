/*****************************************************************/
/*    NAME: Charles Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: BHV_TaskRefuelReplaceBasic.h                         */
/*    DATE: Feb 15th 2026                                        */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#ifndef BHV_TASK_REFUEL_REPLACE_BASIC_HEADER
#define BHV_TASK_REFUEL_REPLACE_BASIC_HEADER

#include <string>
#include <vector>
#include "IvPTaskBehavior.h"

class IvPDomain;
class BHV_TaskRefuelReplaceBasic : public IvPTaskBehavior {
public:
  BHV_TaskRefuelReplaceBasic(IvPDomain);
  ~BHV_TaskRefuelReplaceBasic() {}

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

  // A very small distance penalty used only as a bid tie-breaker.
  double m_distance_tiebreak_weight;
  double m_fuel_abstain_threshold;

 protected:  // State vars (read from MOOSDB)

  double m_fuel_dist_remaining;
  bool   m_got_fuel_input;
  bool   m_returning_mode;
  bool   m_refuel_transit_busy;
};

#ifdef WIN32
   #define IVP_EXPORT_FUNCTION __declspec(dllexport)
#else
   #define IVP_EXPORT_FUNCTION
#endif

extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain)
  {return new BHV_TaskRefuelReplaceBasic(domain);}
}
#endif
