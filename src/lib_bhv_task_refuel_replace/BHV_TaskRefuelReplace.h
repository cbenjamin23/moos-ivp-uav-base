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

  // virtuals defined
  void   onHelmStart();
  double getTaskBid();
  bool   setParam(std::string, std::string);

  vector<VarDataPair>  applyFlagMacros(std::vector<VarDataPair>);

  void         onIdleState();
  IvPFunction* onRunState();

 protected:
  bool  updatePlatformInfo();

  
 protected:  // Configuration Parameters

  double m_ptx;   
  double m_pty;   

  bool m_ptx_set;
  bool m_pty_set;

  bool m_consider_contacts;
  
 protected:  // State Variables

};

#ifdef WIN32
   // Windows needs to explicitly specify functions to export from a dll
   #define IVP_EXPORT_FUNCTION __declspec(dllexport) 
#else
   #define IVP_EXPORT_FUNCTION
#endif

extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain) 
  {return new BHV_TaskRefuelReplace(domain);}
}
#endif





