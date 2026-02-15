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
   // Tracks task metadata (requester/target/type) for both target/basic tasks.
   void processTaskRefuelTarget(const std::string& task_msg);
   // Tracks task state transitions. Source app/community are used as fallback
   // winner hints when TASK_STATE payloads omit an explicit winner key.
   void processTaskState(const std::string& state_msg,
                         const std::string& msg_source_app,
                         const std::string& msg_source_community);
   void clearActiveReplacementLock(const std::string& reason);
   // Centralized task posting path used by both threshold and discovery triggers.
   // bypass_task_latch=true allows discovery-driven requests without unlatching
   // threshold semantics globally.
   bool postReplacementTask(const std::string& trigger_reason,
                            bool bypass_task_latch);
   // TASK_STATE format varies by task manager version; probe known winner keys.
   std::string parseTaskStateWinner(const std::string& spec) const;
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
    std::string task_type;
    double target_x = 0;
    double target_y = 0;
    bool   target_set = false;
    // True only when this vehicle is inferred to be the actual winner.
    bool   bidwon_by_me = false;
    // Set after this replacement has notified requester to return.
    bool   handoff_sent = false;
  };

 private: // Configuration variables

  double m_refuel_threshold;     // meters; trigger when odom >= threshold
  double m_total_range;          // total distance this UAV can fly on full fuel
  double m_handoff_radius;       // winner notifies requester once inside this range
  // Replacement lock timeout guard. If no terminal signal arrives, release
  // lock after this duration to avoid permanent "busy" state.
  double m_replacement_lock_timeout; // seconds; 0 disables timeout
  // Discovery requests are queued; stale queued requests are dropped.
  double m_discovery_request_timeout; // seconds; pending request expiry
  // Fire-ID based dedupe window to avoid repost storms on repeated discovery mail.
  double m_discovery_repost_cooldown; // seconds; same-fire repost guard

  // The loiter target this vehicle covers
  double m_target_x;
  double m_target_y;
  bool   m_target_set;

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
  bool m_returning_mode;

  bool m_waiting_for_odom_reset;
  bool m_task_sent;              // latch so we only post once
  int  m_task_id_counter;        // rr0, rr1, rr2, ...

  // Single-active-replacement lock. Core concurrency guard:
  // while non-empty, additional wins are ignored for execution.
  std::string m_active_replacement_hash;
  std::string m_active_replacement_type;
  double      m_active_replacement_time;
  // For basic replacements, hold lock through the whole return leg.
  // This flips true once RETURN=true is observed for the active basic task
  // and is cleared only when the lock itself is cleared.
  bool        m_active_replacement_return_started;

  // One pending discovery-triggered request at a time.
  std::string m_pending_discovery_fire_id;
  double      m_pending_discovery_utc;
  // Last post times keyed by fire_id for cooldown/dedupe.
  std::map<std::string, double> m_last_discovery_post_utc;
};

#endif 
