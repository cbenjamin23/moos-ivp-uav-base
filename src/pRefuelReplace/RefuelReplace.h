/************************************************************/
/*    NAME: Charlie Benjamin                                */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: RefuelReplace.h                                 */
/*    DATE: January 31st, 2026                              */
/************************************************************/

#ifndef RefuelReplace_HEADER
#define RefuelReplace_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

class RefuelReplace : public AppCastingMOOSApp
{
 public:
   RefuelReplace();
   ~RefuelReplace();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
  bool buildReport();

 protected:
  void registerVariables();

 private: // Configuration variables

  double m_refuel_threshold;     // meters; trigger when odom >= threshold

  // The loiter region this vehicle is currently covering
  double m_region_x;
  double m_region_y;
  bool   m_region_set;

  // Region priority weight (sent in the MISSION_TASK so bidders
  // know how important this region is)
  double m_priority_weight;

  std::string m_host_community;
  
 private: // State variables
  
  double m_nav_x;
  double m_nav_y;
  double m_odometry_dist;

  bool m_got_nav_x;
  bool m_got_nav_y;
  bool m_got_odom;

  bool m_task_sent;              // latch so we only post once
  int  m_task_id_counter;        // rr0, rr1, rr2, ...
};

#endif 