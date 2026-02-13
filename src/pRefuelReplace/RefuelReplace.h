/************************************************************/
/*    NAME: Charlie Benjamin                                */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: RefuelReplace.h                                 */
/*    DATE: January 31st, 2026                              */
/************************************************************/

#ifndef RefuelReplace_HEADER
#define RefuelReplace_HEADER

#include <map>
#include <string>
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
   void processTaskRefuel(const std::string& task_msg);
   void processTaskState(const std::string& state_msg);
   std::string normalizeTaskSpec(const std::string& msg) const;
   std::string inferRequesterFromId(const std::string& id) const;
   void sendNodeMessage(const std::string& dest_node,
                        const std::string& var_name,
                        const std::string& value);
   void notifyRequesterReturn(const std::string& requester,
                              const std::string& task_hash);

  struct TaskRecord {
    std::string id;
    std::string requester;
    double region_x = 0;
    double region_y = 0;
    bool   region_set = false;
    bool   bidwon_by_me = false;
    bool   handoff_sent = false;
  };

 private: // Configuration variables

  double m_refuel_threshold;     // meters; trigger when odom >= threshold
  double m_total_range;          // total distance this UAV can fly on full fuel
  double m_handoff_radius;       // winner notifies requester once inside this range

  // The loiter region this vehicle covers
  double m_region_x;
  double m_region_y;
  bool   m_region_set;

  double m_priority_weight;

  std::string m_host_community;

  std::map<std::string, TaskRecord> m_task_records;  // keyed by task hash
  
 private: // State variables
  
  double m_nav_x;
  double m_nav_y;
  double m_odometry_dist;
  double m_fuel_distance_remaining;

  bool m_got_nav_x;
  bool m_got_nav_y;
  bool m_got_odom;

  bool m_task_sent;              // latch so we only post once
  int  m_task_id_counter;        // rr0, rr1, rr2, ...
};

#endif 
