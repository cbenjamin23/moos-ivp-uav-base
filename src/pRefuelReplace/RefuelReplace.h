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

// pRefuelReplace coordinates replacement-task posting and local commitment state.
// It supports two trigger paths:
// 1) Threshold-triggered: one-shot per odometry-reset cycle (m_task_sent latch).
// 2) Discovery override: queued, deduped by fire_id, and only posts
//    immediately if the vehicle is already at/above threshold.
//
// The app also enforces a single active replacement commitment per vehicle via
// m_active_lock. While this lock is held, REFUEL_TRANSIT_BUSY is true,
// and sibling task behaviors on the same helm should abstain from new auctions.
class RefuelReplace : public AppCastingMOOSApp
{
 public:
   RefuelReplace();
   ~RefuelReplace();

 protected:
   struct PendingTaskInfo;
   struct ActiveReplacementLock;

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();
   // Upserts pre-win task metadata keyed by hash for later win handling.
   void upsertTaskInfo(const std::string& task_msg);
   // Tracks task state transitions. Source app/community are used as fallback
   // winner hints when TASK_STATE payloads omit an explicit winner key.
   void processTaskState(const std::string& state_msg,
                         const std::string& msg_source_app,
                         const std::string& msg_source_community);
   // Clears the one active replacement commitment and records why it ended.
   void clearActiveReplacementLock(const std::string& reason);
   // Centralized task posting path used by both threshold and discovery triggers.
   // bypass_task_latch=true allows discovery-driven requests without unlatching
   // threshold semantics globally.
   bool postReplacementTask(const std::string& trigger_reason,
                            bool bypass_task_latch);
   // TASK_STATE format varies by task manager version; probe known winner keys.
   std::string parseTaskStateWinner(const std::string& spec) const;
   // Normalizes task strings so comma- and hash-delimited specs parse the same way.
   std::string normalizeTaskSpec(const std::string& msg) const;
   // Infers the requester vehicle name from the generated task id when absent.
   std::string inferRequesterFromId(const std::string& id) const;
   // Returns true when this vehicle is currently committed to one replacement.
   bool activeLockEngaged() const;
   // Classifies tasks that should release on return-reset instead of handoff.
   bool activeTaskUsesReturnReset(const std::string& task_type,
                                  bool target_set) const;
   // Publishes the local remaining range estimate from total range minus odometry.
   void updateFuelDistanceRemaining();
   // Re-arms threshold posting after odometry has genuinely reset low again.
   void maybeRelatchAfterOdometryReset();
   // Handles the queued discovery override that can post immediate relief only
   // when the vehicle is already in replacement-needed territory.
   void maybeHandleImmediateDiscoveryReplacement(bool have_nav, bool have_odom);
   // Posts the standard threshold-triggered replacement task once per cycle.
   void maybePostThresholdReplacement(bool have_nav, bool have_odom);
   // Sends return handoff once the active target-task winner reaches handoff range.
   void maybeSendActiveHandoff(bool have_nav);
   // Advances timeout and completion rules for the one active replacement lock.
   void maybeMaintainActiveLock();
   // Ages out stale non-active pending task metadata from the passive map.
   void prunePendingTasks(double now);
   // Sends a string-valued NODE_MESSAGE_LOCAL to one destination vehicle.
   void sendNodeMessage(const std::string& dest_node,
                        const std::string& var_name,
                        const std::string& value);
   // Sends the requester the control messages that complete a target handoff.
   void notifyRequesterReturn(const std::string& requester,
                              const std::string& task_hash);

  struct PendingTaskInfo {
    std::string id;
    std::string requester;
    std::string task_type;
    double target_x = 0;
    double target_y = 0;
    bool   target_set = false;
    // Pending-task freshness is used only to age out stale non-active entries.
    double last_seen_utc = 0.0;
  };

  struct ActiveReplacementLock {
    std::string task_hash;
    std::string requester;
    std::string task_type;
    double      target_x = 0.0;
    double      target_y = 0.0;
    bool        target_set = false;
    double      acquired_utc = 0.0;
    double      last_refresh_utc = 0.0;
    // Set after this replacement has notified requester to return.
    bool        handoff_sent = false;
    // For basic/basic-like replacements, flips true once RETURN=true is seen.
    bool        return_started = false;
    // Captures ODOMETRY_RESET while the lock is held for return-reset release.
    bool        odom_reset_seen = false;

    bool engaged() const { return(task_hash != ""); }

    void clear()
    {
      task_hash = "";
      requester = "";
      task_type = "";
      target_x = 0.0;
      target_y = 0.0;
      target_set = false;
      acquired_utc = 0.0;
      last_refresh_utc = 0.0;
      handoff_sent = false;
      return_started = false;
      odom_reset_seen = false;
    }
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

  // Passive pre-win task metadata keyed by task hash. This exists so the app
  // can resolve thin hash-based TASK_STATE mail into actionable metadata later.
  std::map<std::string, PendingTaskInfo> m_pending_tasks;
  
 private: // State variables
  
  double m_nav_x;
  double m_nav_y;
  double m_odometry_dist;
  double m_fuel_distance_remaining;

  bool m_got_nav_x;
  bool m_got_nav_y;
  bool m_got_odom;
  bool m_returning_mode;

  // Threshold relatch gate: after ODOMETRY_RESET, wait for odom to drop low
  // before allowing the next threshold-triggered task post.
  bool m_waiting_for_odom_reset;
  // Threshold-latch only. Discovery override posts may bypass this latch.
  bool m_task_sent;
  int  m_task_id_counter;        // rr0, rr1, rr2, ...

  // Single-active-replacement lock. Core concurrency guard:
  // while engaged, additional wins are ignored for execution.
  ActiveReplacementLock m_active_lock;

  // One pending discovery override request at a time. New requests overwrite
  // older pending ones to keep a bounded queue model.
  std::string m_pending_discovery_fire_id;
  double      m_pending_discovery_utc;
  // Last post times keyed by fire_id for cooldown/dedupe.
  std::map<std::string, double> m_last_discovery_post_utc;
};

#endif 
