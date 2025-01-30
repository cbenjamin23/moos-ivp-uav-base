/************************************************************/
/*    NAME: Steve Carter Feujo Nomeny                       */
/*    ORGN: NTNU, MIT                                       */
/*    FILE: UAV_Model.h                                     */
/*    DATE: September 9th, 2024                             */
/************************************************************/

#pragma once

#include <string>
#include "NodeRecord.h"
#include "AngleUtils.h"

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/mission_raw/mission_raw.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/param/param.h>
#include <iostream>

#include <functional>

#include "WarningSystem.h"

#include "XYPoint.h"

#include "definitions.h"
#include "mavlinkDefinitionsArdupilot.h"

// For threading
#include "ThreadSafeVariable.h"
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>

class UAV_Model
{
public:
  UAV_Model(std::shared_ptr<WarningSystem> ws = nullptr);
  virtual ~UAV_Model() {}

  void registerWarningSystem(std::shared_ptr<WarningSystem> ws) { m_warning_system_ptr = ws; }
  bool connectToUAV(std::string url);
  bool setUpMission(bool onlyRegisterHome = true);

  bool startMission() const;
  // async function
  bool sendArmCommandIfHealthyAndNotArmed_async() const;

  bool subscribeToTelemetry();

  // Polling functions

  // Non-blocking function to poll all parameters
  void pollAllParametersAsync();

  // Actions and commands , blocking functions
  bool commandAndSetAirSpeed(double speed);
  bool commandGroundSpeed(double speed) const { return (commandSpeed(speed, SPEED_TYPE::SPEED_TYPE_GROUNDSPEED)); }
  bool commandAndSetAltitudeAGL(double altitudeAGL_m); // requires guided mode
  bool commandGoToLocationXY(const XYPoint pos, bool holdCurrentAltitudeAGL = false);
  bool commandGoToLocation(const mavsdk::Telemetry::Position &position);

  // void commandGoToLocationXY_async(const XYPoint pos, bool holdCurrentAltitudeAGL = false, std::function<void(bool)> callback = nullptr);
  // void commandGoToLocation_async(const mavsdk::Telemetry::Position &position, std::function<void(bool)> callback = nullptr);

  bool commandReturnToLaunchAsync() const;
  bool commandLoiterAtPos(XYPoint pos, bool holdCurrentAltitude = true);
  bool commandAndSetHeading(double heading, bool isAllowed = true); // command COG if in guided mode

  bool commandGuidedMode(bool alt_hold = false);

  bool commandDisarmAsync() const;

  // Setters
  void setCallbackMOOSTrace(const std::function<void(const std::string &)> &callback) { callbackMOOSTrace = callback; }
  void setCallbackReportEvent(const std::function<void(const std::string &)> &callback) { callbackReportEvent = callback; }
  void setCallbackReportRunW(const std::function<void(const std::string &)> &callback) { callbackReportRunW = callback; }
  void setCallbackRetractRunW(const std::function<void(const std::string &)> &callback) { callbackRetractRunW = callback; }

  void setNextWaypointLatLon(const XYPoint &wp) { mts_next_waypoint_coord.set(wp); }
  void setLoiterLocationLatLon(const XYPoint &wp) { mts_current_loiter_coord.set(wp); }
  void setHeadingWyptFromHeading(double heading);
  // void setTargetAltitudeAGL(double altitude) { m_target_altitudeAGL = altitude; }

  // void setTargetAirSpeed(double speed) { m_target_airspeed = speed; }

  // void setTargetHeading(double heading) { m_target_heading = heading; }

  // Getters
  bool isHealthy() const { return m_health_all_ok; }
  bool isArmed() const { return (m_is_armed); }
  bool isInAir() const { return (m_in_air); }
  mavsdk::Telemetry::FlightMode getFlightMode() const { return (mts_flight_mode.get()); }
  bool isGuidedMode() const { return (mts_flight_mode.get() == mavsdk::Telemetry::FlightMode::Guided); }
  bool isHoldHeadingGuidedSet() const { return (m_is_hold_heading_guided_set); }

  XYPoint getNextWaypointLatLon() const { return (mts_next_waypoint_coord.get()); }
  XYPoint getHeadingWaypointLatLon() const { return (mts_heading_waypoint_coord.get()); }
  XYPoint getCurrentLoiterLatLon() const { return (mts_current_loiter_coord.get()); }
  XYPoint getHomeLatLon() const { return (mts_home_coord.get()); }
  double getLatitude() const { return (mts_position.get().latitude_deg); }
  double getLongitude() const { return (mts_position.get().longitude_deg); }

  double getMinAirSpeed() const { return (mts_polled_params.get().min_airspeed); }
  double getMaxAirSpeed() const { return (mts_polled_params.get().max_airspeed); }
  double getTargetCruiseSpeed() const { return (mts_polled_params.get().target_airspeed_cruise); }
  double getTargetAirSpeed() const { return (m_target_airspeed); }
  double getAirSpeed() const { return (sqrt(m_velocity_ned.get().north_m_s * m_velocity_ned.get().north_m_s + m_velocity_ned.get().east_m_s * m_velocity_ned.get().east_m_s) + m_velocity_ned.get().down_m_s * m_velocity_ned.get().down_m_s); }
  double getSOG() const { return (sqrt(m_velocity_ned.get().north_m_s * m_velocity_ned.get().north_m_s + m_velocity_ned.get().east_m_s * m_velocity_ned.get().east_m_s)); }
  double getHeading() const { return (angle360(mts_attitude_ned.get().yaw_deg)); }

  double getAltitudeAGL() const { return (mts_position.get().relative_altitude_m); }
  double getAltitudeMSL() const { return (mts_position.get().absolute_altitude_m); }
  double getTargetAltitudeAGL() const { return (m_target_altitudeAGL); }
  double getLastSentTargetAltitudeAGL() const { return (m_last_sent_altitudeAGL); }

  double getTargetHeading() const { return (m_target_heading); }

  double getRoll() const { return (mts_attitude_ned.get().roll_deg); }
  double getPitch() const { return (mts_attitude_ned.get().pitch_deg); }

  // For threading:
  void startCommandSender(); // Signal the thread to start
  void stopCommandSender();  // Signal the thread to stop

  void registerSendDesiredValuesFunction(std::function<void(UAV_Model &, bool)> sendDesiredValues);
  void enableSendDesiredValues(bool enable = true);

  // Thread-safe command queue
  template <typename Command>
  void pushCommand(Command &&cmd)
  {
    struct CommandWrapper : CommandBase
    {
      Command cmd;
      CommandWrapper(Command &&c) : cmd(std::move(c)) {}
      void execute(UAV_Model &uav) override { cmd(uav); }
    };

    {
      std::lock_guard lock(m_queue_mutex);
      m_command_queue.push(
          std::make_unique<CommandWrapper>(std::forward<Command>(cmd)));
    }
    m_thread_cv.notify_one();
  }

private:
  void runCommandsender(); // Main loop for the UAV thread
  std::atomic<bool> m_running{false};
  std::thread m_thread;

  std::atomic<bool> m_sendValuesEnabled{false};

  std::function<void(UAV_Model &, bool)> sendDesiredValues;
  std::mutex m_sendDesiredValues_mutex;

  struct CommandBase
  {
    virtual ~CommandBase() = default;
    virtual void execute(UAV_Model &uav) = 0;
  };
  std::queue<std::unique_ptr<CommandBase>> m_command_queue;
  std::mutex m_queue_mutex;
  std::condition_variable m_thread_cv;

protected:
  enum class Parameters
  {
    AIRSPEED_MIN,
    AIRSPEED_MAX,
    AIRSPEED_TARGET_CRUISE
  };

  // Structure to hold polled parameters
  struct PolledParameters
  {
    int min_airspeed = 0;
    int max_airspeed = 0;
    double target_airspeed_cruise = 0;
  };

protected:
  std::shared_ptr<WarningSystem> m_warning_system_ptr;
  // Callbacks for debug messages
  std::function<void(const std::string &)> callbackMOOSTrace;
  std::function<void(const std::string &)> callbackReportEvent;
  std::function<void(const std::string &)> callbackReportRunW;
  std::function<void(const std::string &)> callbackRetractRunW;

  void MOOSTraceFromCallback(const std::string &msg) const
  {
    if (callbackMOOSTrace)
      callbackMOOSTrace(msg);
  };
  void reportRunWarningFromCallback(const std::string &msg) const
  {
    if (callbackReportRunW)
      callbackReportRunW(msg);
  };
  void retractRunWarningFromCallback(const std::string &msg) const
  {
    if (callbackRetractRunW)
      callbackRetractRunW(msg);
  };
  void reportEventFromCallback(const std::string &msg) const
  {
    if (callbackReportEvent)
      callbackReportEvent(msg);
  };

  bool commandSpeed(double airspeed_m_s, SPEED_TYPE speed_type = SPEED_TYPE::SPEED_TYPE_GROUNDSPEED) const;
  bool commandArmAsync() const;

  bool setParameterAsync(Parameters param_enum, double value) const;
  bool getParameterAsync(Parameters param_enum);

  bool haveAutorythyToChangeMode() const;

  bool commandChangeAltitude_Guided(double altitude_m, bool relativeAlt = false, double vrate_ms = 0) const;
  bool commandChangeSpeed_Guided(double speed_m_s, SPEED_TYPE speed_type = SPEED_TYPE::SPEED_TYPE_AIRSPEED) const;
  bool commandChangeHeading_Guided(double hdg_deg, HEADING_TYPE hdg_type = HEADING_TYPE_COURSE_OVER_GROUND);

protected:
  std::shared_ptr<mavsdk::System> m_system_ptr;
  std::shared_ptr<mavsdk::Mavsdk> m_mavsdk_ptr;
  std::unique_ptr<mavsdk::MissionRaw> m_mission_raw_ptr;
  std::unique_ptr<mavsdk::Action> m_action_ptr;
  std::unique_ptr<mavsdk::Telemetry> m_telemetry_ptr;
  std::unique_ptr<mavsdk::Param> m_param_ptr;
  std::unique_ptr<mavsdk::MavlinkPassthrough> m_mavPass_ptr;

  std::atomic<bool> m_health_all_ok;
  std::atomic<bool> m_is_armed;
  std::atomic<bool> m_in_air;

  // Exiting GUIDED returns aircraft to normal behaviour defined elsewhere
  // If hold heading is set, it will ignore further repositions command, and Guided mode has to be exited
  std::atomic<bool> m_is_hold_heading_guided_set;

  // Telemetry
  ThreadSafeVariable<mavsdk::Telemetry::Position> mts_position;
  ThreadSafeVariable<mavsdk::Telemetry::EulerAngle> mts_attitude_ned;
  ThreadSafeVariable<mavsdk::Telemetry::VelocityNed> m_velocity_ned;

  ThreadSafeVariable<mavsdk::Telemetry::Battery> mts_battery;

  ThreadSafeVariable<mavsdk::Telemetry::FlightMode> mts_flight_mode;

  ThreadSafeVariable<XYPoint> mts_home_coord;
  ThreadSafeVariable<XYPoint> mts_current_loiter_coord;
  ThreadSafeVariable<XYPoint> mts_next_waypoint_coord;
  ThreadSafeVariable<XYPoint> mts_heading_waypoint_coord;

  // Parameters
  ThreadSafeVariable<PolledParameters> mts_polled_params;

  // Parameters not polled
  std::atomic<double> m_target_heading;
  std::atomic<double> m_target_airspeed;
  std::atomic<double> m_target_altitudeAGL;
  std::atomic<double> m_last_sent_altitudeAGL;
};

mavsdk::MissionRaw::MissionItem make_mission_item_wp(
    float latitude_deg1e7,
    float longitude_deg1e7,
    int32_t altitude_m,
    float param1,
    MAV_FRAME frame,
    MAV_CMD command,
    float p2 = 0,
    float p3 = 0);

bool create_missionPlan(std::vector<mavsdk::MissionRaw::MissionItem> &mission_plan, double lat_deg_home = -35.359833, double lon_deg_home = 149.164703);
