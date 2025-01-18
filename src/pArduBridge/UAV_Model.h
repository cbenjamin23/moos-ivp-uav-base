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
  bool sendArmCommandIfHealthyAndNotArmed() const;

  bool subscribeToTelemetry();

  // Polling functions

  // Non-blocking function to poll all parameters
  void pollAllParametersAsync();

  // Actions and commands , blocking functions
  bool commandAndSetAirSpeed(double speed);
  bool commandGroundSpeed(double speed) const { return (commandSpeed(speed, SPEED_TYPE::SPEED_TYPE_GROUNDSPEED)); }
  bool commandGoToLocationXY(const XYPoint pos, bool holdCurrentAltitudeAGL = false);
  bool commandGoToLocation(const mavsdk::Telemetry::Position &position);
  bool commandAndSetAltitudeAGL(double altitudeAGL_m); // requires guided mode

  bool commandReturnToLaunchAsync() const;
  bool commandLoiterAtPos(XYPoint pos, bool holdCurrentAltitude = true);
  bool commandAndSetHeading(double heading); // command COG if in guided mode

  bool commandDisarmAsync() const;

  bool commandGuidedMode(bool alt_hold = false);

  // Setters
  void setCallbackMOOSTrace(const std::function<void(const std::string &)> &callback) { callbackMOOSTrace = callback; }
  void setCallbackReportEvent(const std::function<void(const std::string &)> &callback) { callbackReportEvent = callback; }
  void setCallbackReportRunW(const std::function<void(const std::string &)> &callback) { callbackReportRunW = callback; }
  void setCallbackRetractRunW(const std::function<void(const std::string &)> &callback) { callbackRetractRunW = callback; }

  void setNextWaypointLatLon(const XYPoint &wp) { m_next_waypoint_coord = wp; }
  void setLoiterLocationLatLon(const XYPoint &wp) { m_current_loiter_coord = wp; }
  void setHeadingWyptFromHeading(double heading);
  // void setTargetAltitudeAGL(double altitude) { m_target_altitudeAGL = altitude; }

  // void setTargetAirSpeed(double speed) { m_target_airspeed = speed; }

  // void setTargetHeading(double heading) { m_target_heading = heading; }

  // Getters
  bool isHealthy() const { return (m_health_all_ok); }
  bool isArmed() const { return (m_is_armed); }
  bool isInAir() const { return (m_in_air); }
  mavsdk::Telemetry::FlightMode getFlightMode() const { return (m_flight_mode); }
  bool isGuidedMode() const { return (m_flight_mode == mavsdk::Telemetry::FlightMode::Guided); }

  XYPoint getNextWaypointLatLon() const { return (m_next_waypoint_coord); }
  XYPoint getHeadingWaypointLatLon() const { return (m_heading_waypoint_coord); }
  XYPoint getCurrentLoiterLatLon() const { return (m_current_loiter_coord); }
  XYPoint getHomeLatLon() const { return (m_home_coord); }
  double getLatitude() const { return (m_position.latitude_deg); }
  double getLongitude() const { return (m_position.longitude_deg); }

  double getMinAirSpeed() const { return (m_polled_params.min_airspeed); }
  double getMaxAirSpeed() const { return (m_polled_params.max_airspeed); }
  double getTargetCruiseSpeed() const { return (m_polled_params.target_airspeed_cruise); }
  double getTargetAirSpeed() const { return (m_target_airspeed); }
  double getAirSpeed() const { return (sqrt(m_velocity_ned.north_m_s * m_velocity_ned.north_m_s + m_velocity_ned.east_m_s * m_velocity_ned.east_m_s) + m_velocity_ned.down_m_s * m_velocity_ned.down_m_s); }
  double getSOG() const { return (sqrt(m_velocity_ned.north_m_s * m_velocity_ned.north_m_s + m_velocity_ned.east_m_s * m_velocity_ned.east_m_s)); }
  double getHeading() const { return (angle360(m_attitude_ned.yaw_deg)); }

  double getAltitudeAGL() const { return (m_position.relative_altitude_m); }
  double getAltitudeMSL() const { return (m_position.absolute_altitude_m); }
  double getTargetAltitudeAGL() const { return (m_target_altitudeAGL); }
  double getLastSentTargetAltitudeAGL() const { return (m_last_sent_altitudeAGL); }

  double getTargetHeading() const { return (m_target_heading); }

  double getRoll() const { return (m_attitude_ned.roll_deg); }
  double getPitch() const { return (m_attitude_ned.pitch_deg); }

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

  // Parameters not polled
  double m_target_heading;
  double m_target_airspeed;
  double m_target_altitudeAGL;
  double m_last_sent_altitudeAGL;

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

  bool m_health_all_ok;
  bool m_is_armed;
  bool m_in_air;

  // Exiting GUIDED returns aircraft to normal behaviour defined elsewhere
  // If hold heading is set, it will ignore further repositions command, and Guided mode has to be exited
  bool m_is_hold_heading_guided_set;

  // Telemetry
  mavsdk::Telemetry::Position m_position;
  mavsdk::Telemetry::EulerAngle m_attitude_ned;
  mavsdk::Telemetry::VelocityNed m_velocity_ned;

  mavsdk::Telemetry::Battery m_battery;

  mavsdk::Telemetry::FlightMode m_flight_mode;

  // Parameters
  PolledParameters m_polled_params;

  XYPoint m_home_coord;
  XYPoint m_current_loiter_coord;
  XYPoint m_next_waypoint_coord;
  XYPoint m_heading_waypoint_coord;
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
