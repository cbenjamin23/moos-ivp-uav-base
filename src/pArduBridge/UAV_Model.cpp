/************************************************************/
/*    NAME: Steve Carter Feujo Nomeny                       */
/*    ORGN: NTNU, MIT                                       */
/*    FILE: UAV_Model.h                                     */
/*    DATE: September 9th, 2024                             */
/************************************************************/

#include <iostream>
#include <cmath>
#include <cstdlib>
#include "UAV_Model.h"
#include "MBUtils.h"
#include "AngleUtils.h"
#include "MOOSGeodesy.h"

#include "thread"

#include <cmath>

#include <future> // for async and promises
#include <algorithm>
#include <chrono>
#include "Logger.h"
//------------------------------------------------------------------------
// Constructor

UAV_Model::UAV_Model(std::shared_ptr<WarningSystem> ws) : m_mavsdk_ptr{std::make_shared<mavsdk::Mavsdk>(mavsdk::Mavsdk::Configuration{mavsdk::Mavsdk::ComponentType::GroundStation})},

                                                          callbackMOOSTrace{nullptr},
                                                          callbackReportRunW{nullptr},
                                                          callbackRetractRunW{nullptr},
                                                          callbackReportEvent{nullptr},
                                                          m_vehicle_type{VehicleType::Plane},

                                                          m_is_hold_course_guided_set{false},
                                                          m_health_all_ok{false},
                                                          m_health_received{false},
                                                          m_last_health_update_s{0.0},
                                                          m_is_armed{false},
                                                          m_in_air{false},
                                                          m_target_altitudeAGL{100.0},
                                                          m_target_airspeed{0.0},
                                                          m_target_course{0.0},
                                                          m_last_sent_altitudeAGL{double(NAN)},
                                                          m_GPS_SOG_m_s{0.0},
                                                          m_GPS_COG_deg{0.0},
                                                          m_gps_received{false},
                                                          m_last_gps_update_s{0.0},
                                                          m_landed_state_received{false},
                                                          m_last_landed_state_update_s{0.0},
                                                          mts_position{mavsdk::Telemetry::Position()},
                                                          mts_attitude_ned{mavsdk::Telemetry::EulerAngle()},
                                                          mts_velocity_ned{mavsdk::Telemetry::VelocityNed()},
                                                          mts_battery{mavsdk::Telemetry::Battery()},
                                                          mts_health{mavsdk::Telemetry::Health()},
                                                          mts_gps_info{mavsdk::Telemetry::GpsInfo()},
                                                          mts_raw_gps{mavsdk::Telemetry::RawGps()},
                                                          mts_landed_state{mavsdk::Telemetry::LandedState::Unknown},
                                                          mts_flight_mode{mavsdk::Telemetry::FlightMode::Unknown},
                                                          mts_home_coord{XYPoint(0, 0)},
                                                          mts_current_loiter_coord{XYPoint(0, 0)},
                                                          mts_next_waypoint_coord{XYPoint(0, 0)},
                                                          mts_course_waypoint_coord{XYPoint(0, 0)},
                                                          mts_polled_params{PolledParameters()}

{
  // Initalize the configuration variables

  m_warning_system_ptr = ws;
}

void UAV_Model::setVehicleType(VehicleType vehicle_type)
{
  m_vehicle_type = vehicle_type;
}

std::string UAV_Model::getVehicleTypeString() const
{
  switch (m_vehicle_type)
  {
  case VehicleType::Copter:
    return "copter";
  case VehicleType::Plane:
  default:
    return "plane";
  }
}

//------------------------------------------------------------------------
// Procedure: connectToUAV()

bool UAV_Model::connectToUAV(std::string url)
{
  if (url.empty())
  {
    // reportRunWarning("No URL specified");
    return false;
  }

  // MOOSTraceFromCallback("Connecting to the URL: " + url + "\n");
  std::cout << "Connecting to the URL: " << url << std::endl;
  mavsdk::ConnectionResult connection_result = m_mavsdk_ptr->add_any_connection(url);

  if (connection_result != mavsdk::ConnectionResult::Success)
  {
    std::stringstream ss;
    ss << "Connection failed: " << connection_result << '\n';
    MOOSTraceFromCallback(ss.str().c_str());

    std::cout << ss.str() << std::endl;
    m_warning_system_ptr->queue_monitorWarningForXseconds(ss.str(), WARNING_DURATION);

    return false;
  }

  std::cout << "Connected to UAV\n";

  MOOSTraceFromCallback("Waiting to discover system...\n");

  auto system = m_mavsdk_ptr->first_autopilot(3.0);
  if (!system.has_value())
  {
    m_warning_system_ptr->queue_monitorWarningForXseconds(WARNING_TIMED_OUT, WARNING_DURATION);
    std::cout << "UAV System NOT discovered\n";
    return false;
  }

  std::cout << "UAV System discovered\n";

  m_system_ptr = system.value();

  // m_mission_raw = MissionRaw{system.value()};
  m_mission_raw_ptr = std::make_unique<mavsdk::MissionRaw>(m_system_ptr);
  m_action_ptr = std::make_unique<mavsdk::Action>(m_system_ptr);
  m_telemetry_ptr = std::make_unique<mavsdk::Telemetry>(m_system_ptr);
  m_mavPass_ptr = std::make_unique<mavsdk::MavlinkPassthrough>(m_system_ptr);
  m_param_ptr = std::make_unique<mavsdk::Param>(m_system_ptr);

  std::cout << "Created mission_raw, action, telemetry, mavlinkPassthrough and param\n";

  if (!isCopter())
  {
    // Poll cruise speed and set to target airspeed for fixed-wing vehicles.
    auto resPair = m_action_ptr->get_target_speed();
    if (resPair.first != mavsdk::Action::Result::Success)
    {
      std::cout << "Failed to get initial target speed\n";
      m_warning_system_ptr->queue_monitorWarningForXseconds("Failed to get initial target speed", WARNING_DURATION);
    }
    else
    {
      m_target_airspeed = resPair.second;
    }
  }

  return true;
}

bool UAV_Model::setUpMission(bool onlyRegisterHome)
{
  if (isCopter())
  {
    onlyRegisterHome = true;
  }

  if (!onlyRegisterHome)
  {
    auto clear_result = m_mission_raw_ptr->clear_mission();

    if (clear_result != mavsdk::MissionRaw::Result::Success)
    {
      m_warning_system_ptr->queue_monitorWarningForXseconds("Failed to clear mission", WARNING_DURATION);
    }
  }

  auto download_result = m_mission_raw_ptr->download_mission();

  std::vector<mavsdk::MissionRaw::MissionItem> mission_plan;

  if (download_result.first != mavsdk::MissionRaw::Result::Success || download_result.second.size() == 0)
  {
    m_warning_system_ptr->queue_monitorWarningForXseconds("Failed to download mission", WARNING_DURATION);
    std::cout << "Failed to download mission - Using default home location\n";

    mavsdk::MissionRaw::MissionItem home_item{};
    home_item.seq = 0;
    home_item.frame = static_cast<uint32_t>(MAV_FRAME_GLOBAL);
    home_item.command = static_cast<uint32_t>(MAV_CMD_NAV_WAYPOINT);
    home_item.current = 1;
    home_item.autocontinue = 1;
    home_item.param1 = 0;
    home_item.param2 = 0;
    home_item.param3 = 0;
    home_item.param4 = 0;
    home_item.x = 633975181; // default at NTNU airport
    home_item.y = 101435316; // default at NTNU airport
    home_item.z = 106.25;    // default at NTNU airport
    home_item.mission_type = MAV_MISSION_TYPE_MISSION;
    mission_plan.push_back(home_item);
  }
  else
  {
    mission_plan = download_result.second;
  }

  mavsdk::MissionRaw::MissionItem home_point = mission_plan[0];

  std::stringstream ss;
  ss << "Home point: " << home_point << std::endl;
  ss << "-----------------------------------------------\n";
  MOOSTraceFromCallback(ss.str());
  std::cout << ss.str();
  ss.clear();

  mts_home_coord->set_vx(home_point.x * 1e-7);
  mts_home_coord->set_vy(home_point.y * 1e-7);

  if (home_point.frame == MAV_FRAME_GLOBAL)
  {
    mts_home_coord->set_vz(home_point.z);
  }
  else
  {
    std::cout << "Home point is not in global frame, but in frame" << home_point.frame << std::endl;
    m_warning_system_ptr->queue_monitorWarningForXseconds("Home point is not in global frame, but in frame" + intToString(home_point.frame), WARNING_DURATION);
  }

  auto m_coord = mts_home_coord.get();
  std::cout << "-> Home point: " << m_coord.x() << ", " << m_coord.y() << " , " << home_point.z << std::endl;

  if (onlyRegisterHome)
  {
    return true;
  }

  mission_plan.clear();

  std::cout << "Creating mission plan\n";
  create_missionPlan(mission_plan, m_coord.x(), m_coord.y());

  auto upload_result = m_mission_raw_ptr->upload_mission(mission_plan);
  if (upload_result != mavsdk::MissionRaw::Result::Success)
  {
    m_warning_system_ptr->queue_monitorWarningForXseconds("Mission upload failed", WARNING_DURATION);
    std::stringstream ss;
    ss << "Failed to upload mission" << std::endl;
    ss << "upload result: " << upload_result << std::endl;
    MOOSTraceFromCallback(ss.str());
    std::cout << ss.str();
  }

  m_mission_raw_ptr->set_current_mission_item(0);

  std::cout << "Mission uploaded\n";
  return true;
}

bool UAV_Model::startMission() const
{

  if (!m_is_armed)
  {
    // reportRunWarning("Not armed");

    m_warning_system_ptr->queue_monitorCondition(WARNING_UAV_NOT_ARMED,
                                                 [&]()
                                                 { return !m_is_armed; });
    return false;
  }

  if (isCopter())
  {
    auto altitude_result = m_action_ptr->set_takeoff_altitude(m_target_altitudeAGL);
    if (altitude_result != mavsdk::Action::Result::Success)
    {
      std::stringstream ss;
      ss << "Failed to set Copter takeoff altitude: " << altitude_result;
      m_warning_system_ptr->queue_monitorWarningForXseconds(ss.str(), WARNING_DURATION);
      return false;
    }

    auto takeoff_result = m_action_ptr->takeoff();
    if (takeoff_result != mavsdk::Action::Result::Success)
    {
      std::stringstream ss;
      ss << "Failed to send Copter takeoff command: " << takeoff_result;
      m_warning_system_ptr->queue_monitorWarningForXseconds(ss.str(), WARNING_DURATION);
      return false;
    }

    MOOSTraceFromCallback("Copter takeoff command succeeded\n");
    return true;
  }

  auto start_result = m_mission_raw_ptr->start_mission();

  if (start_result != mavsdk::MissionRaw::Result::Success)
  {
    m_warning_system_ptr->queue_monitorWarningForXseconds("Failed to start mission", WARNING_DURATION);
    return false;
  }

  MOOSTraceFromCallback("Mission started\n");
  return true;
}

bool UAV_Model::requestTakeoff() const
{
  if (!m_is_armed)
  {
    reportCommandResult("TAKEOFF", "REJECTED", "NOT_ARMED");
    m_warning_system_ptr->queue_monitorWarningForXseconds(
        "TAKEOFF rejected by bridge policy: NOT_ARMED", WARNING_DURATION);
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(m_mode_confirmation_mutex);
    if (m_takeoff_confirmation_tracker.hasPending())
    {
      reportCommandResult("TAKEOFF", "NO_OP", "TAKEOFF_ALREADY_ACTIVE");
      return true;
    }
  }

  if (isCopter())
  {
    if (!m_landed_state_received)
    {
      reportCommandResult("TAKEOFF", "REJECTED", "LANDED_STATE_UNAVAILABLE");
      return false;
    }
    if (getLandedStateTelemetryAge() > LANDED_STATE_TELEMETRY_MAX_AGE_S)
    {
      reportCommandResult("TAKEOFF", "REJECTED", "LANDED_STATE_STALE");
      return false;
    }

    const auto landed_state = mts_landed_state.get();
    if (landed_state == mavsdk::Telemetry::LandedState::TakingOff ||
        landed_state == mavsdk::Telemetry::LandedState::InAir)
    {
      reportCommandResult("TAKEOFF", "NO_OP", "ALREADY_AIRBORNE");
      return true;
    }
    if (landed_state != mavsdk::Telemetry::LandedState::OnGround)
    {
      reportCommandResult("TAKEOFF", "REJECTED", "NOT_ON_GROUND");
      return false;
    }
  }

  const uint64_t command_id = reportCommandResult(
      "TAKEOFF", "SUBMITTED", isCopter() ? "READY" : "MISSION_START_READY");
  if (!startMission())
  {
    reportCommandResult("TAKEOFF", "FAILED",
                        isCopter() ? "TAKEOFF_COMMAND_FAILED" : "MISSION_START_FAILED",
                        command_id);
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(m_mode_confirmation_mutex);
    m_takeoff_confirmation_tracker.accept(
        command_id, m_target_altitudeAGL, isCopter());
  }
  reportCommandResult("TAKEOFF", "ACCEPTED", isCopter() ? "SUCCESS" : "MISSION_STARTED",
                      command_id);
  return true;
}

UAV_Model::PolicyDecision UAV_Model::evaluateArmPolicy(const ArmDisarmPolicyInputs &inputs)
{
  if (inputs.armed)
    return {PolicyAction::NoOp, "ALREADY_ARMED"};
  if (!inputs.health_available)
    return {PolicyAction::Reject, "HEALTH_UNAVAILABLE"};
  if (inputs.health_age_s < 0.0 || inputs.health_age_s > HEALTH_TELEMETRY_MAX_AGE_S)
    return {PolicyAction::Reject, "HEALTH_STALE"};
  if (!inputs.armable)
    return {PolicyAction::Reject, "AUTOPILOT_NOT_ARMABLE"};
  if (!inputs.health_all_ok)
    return {PolicyAction::Reject, "HEALTH_NOT_OK"};
  if (!inputs.landed_state_available)
    return {PolicyAction::Reject, "LANDED_STATE_UNAVAILABLE"};
  if (inputs.landed_state_age_s < 0.0 || inputs.landed_state_age_s > LANDED_STATE_TELEMETRY_MAX_AGE_S)
    return {PolicyAction::Reject, "LANDED_STATE_STALE"};
  if (inputs.landed_state == mavsdk::Telemetry::LandedState::Unknown)
    return {PolicyAction::Reject, "LANDED_STATE_UNKNOWN"};
  if (inputs.landed_state != mavsdk::Telemetry::LandedState::OnGround)
    return {PolicyAction::Reject, "NOT_ON_GROUND"};
  return {PolicyAction::Submit, "READY"};
}

UAV_Model::PolicyDecision UAV_Model::evaluateDisarmPolicy(const ArmDisarmPolicyInputs &inputs)
{
  if (!inputs.armed)
    return {PolicyAction::NoOp, "ALREADY_DISARMED"};
  if (!inputs.landed_state_available)
    return {PolicyAction::Reject, "LANDED_STATE_UNAVAILABLE"};
  if (inputs.landed_state_age_s < 0.0 || inputs.landed_state_age_s > LANDED_STATE_TELEMETRY_MAX_AGE_S)
    return {PolicyAction::Reject, "LANDED_STATE_STALE"};
  if (inputs.landed_state == mavsdk::Telemetry::LandedState::Unknown)
    return {PolicyAction::Reject, "LANDED_STATE_UNKNOWN"};
  if (inputs.landed_state != mavsdk::Telemetry::LandedState::OnGround)
    return {PolicyAction::Reject, "NOT_ON_GROUND"};
  return {PolicyAction::Submit, "READY"};
}

UAV_Model::PolicyDecision UAV_Model::evaluateLandPolicy(const LandPolicyInputs &inputs)
{
  const bool landed_state_fresh = inputs.landed_state_available &&
                                  inputs.landed_state_age_s >= 0.0 &&
                                  inputs.landed_state_age_s <= LANDED_STATE_TELEMETRY_MAX_AGE_S;

  if (inputs.flight_mode == mavsdk::Telemetry::FlightMode::Land)
    return {PolicyAction::NoOp, "ALREADY_LANDING"};
  if (landed_state_fresh && inputs.landed_state == mavsdk::Telemetry::LandedState::OnGround)
    return {PolicyAction::NoOp, "ALREADY_ON_GROUND"};
  if (landed_state_fresh && inputs.landed_state == mavsdk::Telemetry::LandedState::Landing)
    return {PolicyAction::NoOp, "ALREADY_LANDING"};

  // Closed allowlist: only modes intentionally used by pArduBridge may submit
  // routine LAND. Pilot-controlled, RTL, unknown, and future modes default-deny.
  switch (inputs.flight_mode)
  {
  case mavsdk::Telemetry::FlightMode::Guided:
  // MAVSDK reports ArduCopter GUIDED telemetry as Offboard.
  case mavsdk::Telemetry::FlightMode::Offboard:
  case mavsdk::Telemetry::FlightMode::Mission:
  case mavsdk::Telemetry::FlightMode::Hold:
    break;
  default:
    return {PolicyAction::Reject, "FLIGHT_MODE_NOT_ALLOWED"};
  }

  if (!inputs.landed_state_available)
    return {PolicyAction::Submit, "READY_LANDED_STATE_UNAVAILABLE"};
  if (!landed_state_fresh)
    return {PolicyAction::Submit, "READY_LANDED_STATE_STALE"};
  if (inputs.landed_state == mavsdk::Telemetry::LandedState::Unknown)
    return {PolicyAction::Submit, "READY_LANDED_STATE_UNKNOWN"};
  return {PolicyAction::Submit, "READY"};
}

UAV_Model::PolicyDecision UAV_Model::evaluatePrecisionLoiterPolicy(const PrecisionLoiterPolicyInputs &inputs)
{
  if (!inputs.copter)
    return {PolicyAction::Reject, "COPTER_ONLY"};

  // Disabling the auxiliary function is always safe for Copter, including
  // after disarm or an external mode change.
  if (!inputs.enable)
    return {PolicyAction::Submit, "READY"};
  if (!inputs.armed)
    return {PolicyAction::Reject, "NOT_ARMED"};
  if (!inputs.precland_enabled_available)
    return {PolicyAction::Reject, "PLND_ENABLED_UNAVAILABLE"};
  if (!inputs.precland_enabled)
    return {PolicyAction::Reject, "PLND_DISABLED"};
  if (!inputs.precland_type_available)
    return {PolicyAction::Reject, "PLND_TYPE_UNAVAILABLE"};
  if (inputs.precland_type == 0)
    return {PolicyAction::Reject, "PLND_TYPE_NONE"};
  if (inputs.flight_mode == mavsdk::Telemetry::FlightMode::Hold)
    return {PolicyAction::Submit, "READY"};
  if (!inputs.enter_loiter_mode)
    return {PolicyAction::Reject, "FC_LOITER_REQUIRED"};

  // Closed allowlist matching the bridge's normal autonomy-controlled modes.
  switch (inputs.flight_mode)
  {
  case mavsdk::Telemetry::FlightMode::Guided:
  case mavsdk::Telemetry::FlightMode::Offboard:
  case mavsdk::Telemetry::FlightMode::Mission:
  case mavsdk::Telemetry::FlightMode::Land:
    return {PolicyAction::Submit, "READY"};
  default:
    return {PolicyAction::Reject, "FLIGHT_MODE_NOT_ALLOWED"};
  }
}

UAV_Model::ArmDisarmPolicyInputs UAV_Model::getArmDisarmPolicyInputs() const
{
  const auto health = getHealth();
  return {
      isArmed(),
      hasHealthTelemetry(),
      getHealthTelemetryAge(),
      health.is_armable,
      isHealthy(),
      hasLandedStateTelemetry(),
      getLandedStateTelemetryAge(),
      getLandedState()};
}

UAV_Model::PolicyDecision UAV_Model::getArmPolicyDecision() const
{
  return evaluateArmPolicy(getArmDisarmPolicyInputs());
}

UAV_Model::PolicyDecision UAV_Model::getDisarmPolicyDecision() const
{
  return evaluateDisarmPolicy(getArmDisarmPolicyInputs());
}

UAV_Model::PolicyDecision UAV_Model::getLandPolicyDecision() const
{
  return evaluateLandPolicy({
      getFlightMode(),
      hasLandedStateTelemetry(),
      getLandedStateTelemetryAge(),
      getLandedState()});
}

bool UAV_Model::requestArmAsync() const
{
  const auto decision = getArmPolicyDecision();
  if (decision.action == PolicyAction::Submit)
  {
    const uint64_t command_id = reportCommandResult("ARM", "SUBMITTED", decision.reason);
    return commandArmAsync(command_id);
  }
  if (decision.action == PolicyAction::NoOp)
  {
    reportCommandResult("ARM", "NO_OP", decision.reason);
    return true;
  }

  reportCommandResult("ARM", "REJECTED", decision.reason);
  m_warning_system_ptr->queue_monitorWarningForXseconds(
      "ARM rejected by bridge policy: " + decision.reason, WARNING_DURATION);
  return false;
}

bool UAV_Model::requestDisarmAsync() const
{
  const auto decision = getDisarmPolicyDecision();
  if (decision.action == PolicyAction::Submit)
  {
    const uint64_t command_id = reportCommandResult("DISARM", "SUBMITTED", decision.reason);
    return commandDisarmAsync(command_id);
  }
  if (decision.action == PolicyAction::NoOp)
  {
    reportCommandResult("DISARM", "NO_OP", decision.reason);
    return true;
  }

  reportCommandResult("DISARM", "REJECTED", decision.reason);
  m_warning_system_ptr->queue_monitorWarningForXseconds(
      "DISARM rejected by bridge policy: " + decision.reason, WARNING_DURATION);
  return false;
}

bool UAV_Model::subscribeToTelemetry()
{

  m_telemetry_ptr->subscribe_armed([&](bool is_armed)
                                   { m_is_armed = is_armed; });

  m_telemetry_ptr->subscribe_health_all_ok([&, this](bool is_health_all_ok)
                                           { this->m_health_all_ok = is_health_all_ok; });

  m_telemetry_ptr->subscribe_health([this](mavsdk::Telemetry::Health health)
                                    {
                                      mts_health = health;
                                      m_last_health_update_s = std::chrono::duration<double>(
                                          std::chrono::steady_clock::now().time_since_epoch()).count();
                                      m_health_received = true;
                                    });

  m_telemetry_ptr->subscribe_position([&](mavsdk::Telemetry::Position position)
                                      {
                                        mts_position = position;

                                        m_in_air = (position.relative_altitude_m >= IN_AIR_HIGHT_THRESHOLD); });

  m_telemetry_ptr->subscribe_gps_info([this](mavsdk::Telemetry::GpsInfo gps_info)
                                      { mts_gps_info = gps_info; });

  m_telemetry_ptr->subscribe_raw_gps([this](mavsdk::Telemetry::RawGps raw_gps)
                                     {
                                      m_GPS_SOG_m_s = raw_gps.velocity_m_s;
                                      m_GPS_COG_deg = raw_gps.cog_deg;
                                      mts_raw_gps = raw_gps;
                                      m_last_gps_update_s = std::chrono::duration<double>(
                                          std::chrono::steady_clock::now().time_since_epoch()).count();
                                      m_gps_received = true; });

  m_telemetry_ptr->subscribe_landed_state([this](mavsdk::Telemetry::LandedState landed_state)
                                          {
                                            mts_landed_state = landed_state;
                                            m_last_landed_state_update_s = std::chrono::duration<double>(
                                                std::chrono::steady_clock::now().time_since_epoch()).count();
                                            m_landed_state_received = true;
                                          });

  m_telemetry_ptr->subscribe_attitude_euler([&](mavsdk::Telemetry::EulerAngle attitude_ned)
                                            { mts_attitude_ned = attitude_ned; });

  m_telemetry_ptr->subscribe_velocity_ned([&](mavsdk::Telemetry::VelocityNed vel)
                                          { mts_velocity_ned = vel; });

  m_telemetry_ptr->subscribe_battery([&](mavsdk::Telemetry::Battery battery)
                                     { mts_battery = battery; });

  m_telemetry_ptr->subscribe_flight_mode([&](mavsdk::Telemetry::FlightMode flight_mode)
                                         { mts_flight_mode = flight_mode; });

  if (m_telemetry_ptr->set_rate_gps_info(5.0) != mavsdk::Telemetry::Result::Success)
  {
    m_warning_system_ptr->queue_monitorWarningForXseconds("Failed to request GPS telemetry", WARNING_DURATION);
  }

  if (m_telemetry_ptr->set_rate_landed_state(2.0) != mavsdk::Telemetry::Result::Success)
  {
    m_warning_system_ptr->queue_monitorWarningForXseconds("Failed to request landed-state telemetry", WARNING_DURATION);
  }

  if (!requestTelemetryMessageRate(MAVLINK_MSG_ID_SYS_STATUS, 1.0))
  {
    m_warning_system_ptr->queue_monitorWarningForXseconds("Failed to request SYS_STATUS telemetry", WARNING_DURATION);
  }

  // gives wrong data
  // m_telemetry_ptr->subscribe_in_air([&](bool in_air) {
  //   m_in_air = in_air;
  // });

  return true;
}

double UAV_Model::getGpsTelemetryAge() const
{
  if (!m_gps_received)
  {
    return -1.0;
  }

  const double now_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  return std::max(0.0, now_s - m_last_gps_update_s.load());
}

double UAV_Model::getHealthTelemetryAge() const
{
  if (!m_health_received)
  {
    return -1.0;
  }

  const double now_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  return std::max(0.0, now_s - m_last_health_update_s.load());
}

double UAV_Model::getLandedStateTelemetryAge() const
{
  if (!m_landed_state_received)
  {
    return -1.0;
  }

  const double now_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  return std::max(0.0, now_s - m_last_landed_state_update_s.load());
}

std::string UAV_Model::landedStateToString(mavsdk::Telemetry::LandedState landed_state)
{
  switch (landed_state)
  {
  case mavsdk::Telemetry::LandedState::OnGround:
    return "ON_GROUND";
  case mavsdk::Telemetry::LandedState::InAir:
    return "IN_AIR";
  case mavsdk::Telemetry::LandedState::TakingOff:
    return "TAKING_OFF";
  case mavsdk::Telemetry::LandedState::Landing:
    return "LANDING";
  case mavsdk::Telemetry::LandedState::Unknown:
  default:
    return "UNKNOWN";
  }
}

///////////////////////////////////
/////////////  POLLING  //////////
///////////////////////////////////

void UAV_Model::pollAllParametersAsync()
{
  if (isCopter())
  {
    return;
  }

  getParameterAsync(Parameters::AIRSPEED_TARGET_CRUISE);
  getParameterAsync(Parameters::AIRSPEED_MAX);
  getParameterAsync(Parameters::AIRSPEED_MIN);
}

bool UAV_Model::getParameterAsync(Parameters param_enum)
{
  switch (param_enum)
  {
  case Parameters::AIRSPEED_TARGET_CRUISE:
    m_action_ptr->get_target_speed_async([this](mavsdk::Action::Result result, float target_speed)
                                         {
        if (result != mavsdk::Action::Result::Success) {
            std::stringstream ss;
            ss << "Failed to get target speed: " << result;
            m_warning_system_ptr->queue_monitorWarningForXseconds(ss.str(), WARNING_DURATION);
            return;
        }
        std::cout << "target speed: " << target_speed << std::endl;
        mts_polled_params->target_airspeed_cruise = target_speed; });
    break;

  case Parameters::AIRSPEED_MAX:
    m_action_ptr->get_maximum_speed_async([this](mavsdk::Action::Result result, int max_speed)
                                          {
        if (result != mavsdk::Action::Result::Success) {
            std::stringstream ss;
            ss << "Failed to get maximum speed: " << result;
            m_warning_system_ptr->queue_monitorWarningForXseconds(ss.str(), WARNING_DURATION);
            return;
        }

        mts_polled_params->max_airspeed = static_cast<int>(max_speed); });
    break;
  case Parameters::AIRSPEED_MIN:
    m_action_ptr->get_minimum_speed_async([this](mavsdk::Action::Result result, int min_speed)
                                          {
        if (result != mavsdk::Action::Result::Success) {
            std::stringstream ss;
            ss << "Failed to get minimum speed: " << result;
            m_warning_system_ptr->queue_monitorWarningForXseconds(ss.str(), WARNING_DURATION);
            return;
        }

        mts_polled_params->min_airspeed = min_speed; });
    break;
  default:
    m_warning_system_ptr->queue_monitorWarningForXseconds("Parameter unknown: " + intToString(static_cast<int>(param_enum)), WARNING_DURATION);
    return false;
  }
  return true;
}

bool UAV_Model::setParameterAsync(Parameters param_enum, double value) const
{

  switch (param_enum)
  {
  case Parameters::AIRSPEED_TARGET_CRUISE:
    m_action_ptr->set_target_speed_async(value, [this](mavsdk::Action::Result result)
                                         {
          if (result != mavsdk::Action::Result::Success) {
              std::stringstream ss;
              ss << "Failed to set target speed: " << result;
              m_warning_system_ptr->queue_monitorWarningForXseconds(ss.str(), WARNING_DURATION);
              return;
          } });
    break;
  case Parameters::AIRSPEED_MAX:
    m_action_ptr->set_maximum_speed_async(value, [this](mavsdk::Action::Result result)
                                          {
          if (result != mavsdk::Action::Result::Success) {
              std::stringstream ss;
              ss << "Failed to set maximum speed: " << result;
              m_warning_system_ptr->queue_monitorWarningForXseconds(ss.str(), WARNING_DURATION);
              return;
          } });
    break;
  case Parameters::AIRSPEED_MIN:
    m_action_ptr->set_minimum_speed_async(value, [this](mavsdk::Action::Result result)
                                          {
          if (result != mavsdk::Action::Result::Success) {
              std::stringstream ss;
              ss << "Failed to set minimum speed: " << result;
              m_warning_system_ptr->queue_monitorWarningForXseconds(ss.str(), WARNING_DURATION);
              return;
          } });
    break;
  default:
    m_warning_system_ptr->queue_monitorWarningForXseconds("Parameter unknown: " + intToString(static_cast<int>(param_enum)), WARNING_DURATION);
    return false;
    break;
  }

  return true;
}

bool UAV_Model::haveAutorythyToChangeMode() const
{
  // Read the values from the thread safe variables
  auto m_flight_mode = mts_flight_mode.get();

  if (m_flight_mode == mavsdk::Telemetry::FlightMode::Mission                                                             //   Mission mode is ardupilots AUTO mode
      || m_flight_mode == mavsdk::Telemetry::FlightMode::Hold                                                             // Also loiter mode
      || m_flight_mode == mavsdk::Telemetry::FlightMode::Land || m_flight_mode == mavsdk::Telemetry::FlightMode::Offboard // Previous guided mode
      || m_flight_mode == mavsdk::Telemetry::FlightMode::Guided)
  {
    return true;
  }

  // NOT allowed in Stabilized, Manual or RTL

  return false;
}

///////////////////////////////////
/////////////  COMMANDS  //////////
///////////////////////////////////

bool UAV_Model::commandGuidedMode(bool alt_hold)
{

  if (!haveAutorythyToChangeMode())
  {
    std::stringstream ss;
    ss << "Cannot change mode. Do not have autorithy. Flight mode in " << mts_flight_mode;
    m_warning_system_ptr->queue_monitorWarningForXseconds(ss.str(), WARNING_DURATION);
    return false;
  }

  if (!isCopter() && m_is_hold_course_guided_set && !alt_hold)
  {
    mavsdk::Action::Result result = m_action_ptr->set_flight_mode_auto();

    if (result != mavsdk::Action::Result::Success)
    {
      std::stringstream ss;
      ss << "Failed to exit Guided after hold is activated: " << result;
      m_warning_system_ptr->queue_monitorWarningForXseconds(ss.str(), WARNING_DURATION);
      return false;
    }

    m_is_hold_course_guided_set = false;
  }

  if (isGuidedMode())
    return true;

  mavsdk::Action::Result result = m_action_ptr->set_flight_mode_guided();

  if (result != mavsdk::Action::Result::Success)
  {
    std::stringstream ss;
    ss << "Failed to enter Guided mode: " << result;
    m_warning_system_ptr->queue_monitorWarningForXseconds(ss.str(), WARNING_DURATION);
    return false;
  }

  MOOSTraceFromCallback("command Change Altitude succeeded\n");

  return true;
}

bool UAV_Model::commandReturnToLaunchAsync(const CommandCompletion &completion) const
{
  if (!m_is_armed)
  {
    const std::string detail = "NOT_ARMED";
    reportCommandResult("RTL", "REJECTED", detail);
    if (completion)
      completion(false, detail);
    return false;
  }

  if (mts_flight_mode.get() == mavsdk::Telemetry::FlightMode::ReturnToLaunch)
  {
    reportCommandResult("RTL", "NO_OP", "ALREADY_RTL");
    if (completion)
      completion(true, "ALREADY_RTL");
    return true;
  }

  if (!haveAutorythyToChangeMode())
  {
    const std::string detail = "FLIGHT_MODE_NOT_ALLOWED";
    reportCommandResult("RTL", "REJECTED", detail);
    std::stringstream ss;
    ss << "Cannot change mode. Do not have autorithy. Flight mode in " << mts_flight_mode;
    m_warning_system_ptr->queue_monitorWarningForXseconds(ss.str(), WARNING_DURATION);
    if (completion)
      completion(false, detail);
    return false;
  }

  const uint64_t command_id = reportCommandResult("RTL", "SUBMITTED", "READY");
  m_action_ptr->return_to_launch_async([this, command_id, completion](mavsdk::Action::Result result)
                                       {
    if (result != mavsdk::Action::Result::Success) {
        std::stringstream ss;
        ss << "Return to launch failed: " << result;
        reportCommandResult("RTL", "FAILED", ss.str(), command_id);
        m_warning_system_ptr->queue_monitorWarningForXseconds(ss.str(), WARNING_DURATION);
        if (completion)
          completion(false, ss.str());
        return;
    }
    reportCommandResult("RTL", "ACCEPTED", "SUCCESS", command_id);
    // MAVSDK acceptance is not proof of a mode change; telemetry confirms it.
    {
      std::lock_guard<std::mutex> lock(m_mode_confirmation_mutex);
      m_rtl_confirmation_tracker.accept(command_id);
    }
    if (completion)
      completion(true, "SUCCESS"); });

  return true;
}

bool UAV_Model::commandFlightControllerLoiterAsync(const CommandCompletion &completion) const
{
  const auto flight_mode = mts_flight_mode.get();
  if (flight_mode == mavsdk::Telemetry::FlightMode::Hold)
  {
    reportCommandResult("LOITER_FC", "NO_OP", "ALREADY_FC_LOITER");
    if (completion)
      completion(true, "ALREADY_FC_LOITER");
    return true;
  }

  bool rtl_override_ready = false;
  if (flight_mode == mavsdk::Telemetry::FlightMode::ReturnToLaunch && m_is_armed)
  {
    const auto health = getHealth();
    rtl_override_ready = hasHealthTelemetry() &&
                         getHealthTelemetryAge() <= HEALTH_TELEMETRY_MAX_AGE_S &&
                         isHealthy() &&
                         health.is_local_position_ok &&
                         health.is_global_position_ok &&
                         health.is_home_position_ok;
  }

  if (!haveAutorythyToChangeMode() && !rtl_override_ready)
  {
    const std::string detail =
        flight_mode == mavsdk::Telemetry::FlightMode::ReturnToLaunch
            ? (!m_is_armed ? "NOT_ARMED" : "POSITION_HEALTH_NOT_OK")
            : "FLIGHT_MODE_NOT_ALLOWED";
    reportCommandResult("LOITER_FC", "REJECTED", detail);
    std::stringstream ss;
    ss << "Cannot enter flight-controller Loiter from " << mts_flight_mode;
    m_warning_system_ptr->queue_monitorWarningForXseconds(ss.str(), WARNING_DURATION);
    if (completion)
      completion(false, detail);
    return false;
  }

  const uint64_t command_id = reportCommandResult("LOITER_FC", "SUBMITTED", "READY");
  m_action_ptr->hold_async([this, command_id, completion](mavsdk::Action::Result result)
                           {
    if (result != mavsdk::Action::Result::Success) {
      std::stringstream ss;
      ss << "Flight-controller Loiter failed: " << result;
      reportCommandResult("LOITER_FC", "FAILED", ss.str(), command_id);
      m_warning_system_ptr->queue_monitorWarningForXseconds(ss.str(), WARNING_DURATION);
      if (completion)
        completion(false, ss.str());
      return;
    }

    reportCommandResult("LOITER_FC", "ACCEPTED", "SUCCESS", command_id);
    {
      std::lock_guard<std::mutex> lock(m_mode_confirmation_mutex);
      m_fc_loiter_confirmation_tracker.accept(command_id);
    }
    if (completion)
      completion(true, "SUCCESS"); });

  return true;
}

bool UAV_Model::commandAutoland() const
{
  const auto decision = getLandPolicyDecision();
  if (decision.action == PolicyAction::NoOp)
  {
    reportCommandResult("LAND", "NO_OP", decision.reason);
    reportEventFromCallback("LAND not sent: " + decision.reason);
    return true;
  }
  if (decision.action == PolicyAction::Reject)
  {
    reportCommandResult("LAND", "REJECTED", decision.reason);
    std::stringstream ss;
    ss << "LAND rejected by bridge policy: " << decision.reason
       << "; flight mode: " << mts_flight_mode;
    m_warning_system_ptr->queue_monitorWarningForXseconds(ss.str(), WARNING_DURATION);
    return false;
  }

  if (decision.reason != "READY")
  {
    m_warning_system_ptr->queue_monitorWarningForXseconds(
        "LAND proceeding with limited state confirmation: " + decision.reason,
        WARNING_DURATION);
  }

  const uint64_t command_id = reportCommandResult("LAND", "SUBMITTED", decision.reason);

  mavsdk::Action::Result result = isCopter() ? m_action_ptr->land() : m_action_ptr->set_flight_mode_autoland();

  if (result != mavsdk::Action::Result::Success)
  {
    std::stringstream ss;
    ss << (isCopter() ? "LAND" : "AUTOLAND") << " command error: " << result;
    reportCommandResult("LAND", "FAILED", ss.str(), command_id);
    m_warning_system_ptr->queue_monitorWarningForXseconds(ss.str(), WARNING_DURATION);
    return false;
  }

  reportCommandResult("LAND", "ACCEPTED", "SUCCESS", command_id);

  MOOSTraceFromCallback(isCopter() ? "LAND command succeeded\n" : "AUTOLAND command succeeded\n");
  reportEventFromCallback(isCopter() ? "LAND command sent to UAV" : "AUTOLAND command sent to UAV");

  return true;
}

bool UAV_Model::commandLoiterAtPos(XYPoint pos, bool holdCurrentAltitude)
{

  if (!commandGuidedMode())
    return false;

  // lat lon 0 0  should not be possible
  if (pos == XYPoint(0, 0))
  {
    mts_current_loiter_coord = XYPoint(mts_position.get().latitude_deg, mts_position.get().longitude_deg);
    m_warning_system_ptr->queue_monitorWarningForXseconds("Received empty loiter pos: Loitering at current position", WARNING_DURATION);
  }
  else
  {
    mts_current_loiter_coord = pos;
  }

  if (commandGoToLocationXY(mts_current_loiter_coord, holdCurrentAltitude))
  {
    auto m_coord = mts_current_loiter_coord.get();
    std::stringstream ss;
    ss << "Loitering at (Lat/Long): " << m_coord.x() << "/" << m_coord.y() << "\n";
    reportEventFromCallback(ss.str());
    return true;
  }

  m_warning_system_ptr->queue_monitorWarningForXseconds("Loitering failed", WARNING_DURATION);
  return false;
}

bool UAV_Model::commandAuxFunction(int function, int switch_position) const
{
  mavsdk::MavlinkPassthrough::CommandLong command_mode;
  command_mode.command = MAV_CMD_DO_AUX_FUNCTION_ARDUPILOT;
  command_mode.target_sysid = m_system_ptr->get_system_id();
  command_mode.target_compid = MAV_COMP_ID_AUTOPILOT1;
  command_mode.param1 = function;
  command_mode.param2 = switch_position;
  command_mode.param3 = 0.0;
  command_mode.param4 = 0.0;
  command_mode.param5 = 0.0;
  command_mode.param6 = 0.0;
  command_mode.param7 = 0.0;

  auto result = m_mavPass_ptr->send_command_long(command_mode);
  if (result != mavsdk::MavlinkPassthrough::Result::Success)
  {
    std::stringstream ss;
    ss << "Aux function command error: " << result << " function=" << function << " switch=" << switch_position;
    m_warning_system_ptr->queue_monitorWarningForXseconds(ss.str(), WARNING_DURATION);
    return false;
  }

  return true;
}

bool UAV_Model::requestTelemetryMessageRate(uint32_t message_id, double rate_hz) const
{
  if (!m_mavPass_ptr || !m_system_ptr || rate_hz <= 0.0)
  {
    return false;
  }

  mavsdk::MavlinkPassthrough::CommandLong command{};
  command.command = MAV_CMD_SET_MESSAGE_INTERVAL;
  command.target_sysid = m_system_ptr->get_system_id();
  command.target_compid = MAV_COMP_ID_AUTOPILOT1;
  command.param1 = static_cast<float>(message_id);
  command.param2 = static_cast<float>(1e6 / rate_hz);

  return m_mavPass_ptr->send_command_long(command) == mavsdk::MavlinkPassthrough::Result::Success;
}

bool UAV_Model::commandPrecisionLoiter(bool enable, bool enterLoiterMode)
{
  const std::string command = enable ? "PRECISION_LOITER" : "PRECISION_LOITER_OFF";
  PrecisionLoiterPolicyInputs inputs{
      isCopter(), enable, isArmed(), enterLoiterMode, mts_flight_mode.get(),
      false, false, false, 0};

  // ArduPilot acknowledges auxiliary function 39 even with no precision-
  // landing backend configured, so verify the required parameters first.
  if (enable && inputs.copter && inputs.armed)
  {
    const auto enabled = m_param_ptr->get_param_int("PLND_ENABLED");
    inputs.precland_enabled_available = enabled.first == mavsdk::Param::Result::Success;
    inputs.precland_enabled = inputs.precland_enabled_available && enabled.second == 1;

    const auto type = m_param_ptr->get_param_int("PLND_TYPE");
    inputs.precland_type_available = type.first == mavsdk::Param::Result::Success;
    inputs.precland_type = inputs.precland_type_available ? type.second : 0;
  }

  const auto decision = evaluatePrecisionLoiterPolicy(inputs);
  if (decision.action != PolicyAction::Submit)
  {
    reportCommandResult(command, "REJECTED", decision.reason);
    m_warning_system_ptr->queue_monitorWarningForXseconds(
        "Precision Loiter rejected: " + decision.reason, WARNING_DURATION);
    return false;
  }

  const uint64_t command_id = reportCommandResult(command, "SUBMITTED", decision.reason);

  if (enable && enterLoiterMode && mts_flight_mode.get() != mavsdk::Telemetry::FlightMode::Hold)
  {
    auto hold_result = m_action_ptr->hold();
    if (hold_result != mavsdk::Action::Result::Success)
    {
      std::stringstream ss;
      ss << "Failed to enter Copter Loiter before Precision Loiter: " << hold_result;
      reportCommandResult(command, "FAILED", "FC_LOITER_COMMAND_FAILED", command_id);
      m_warning_system_ptr->queue_monitorWarningForXseconds(ss.str(), WARNING_DURATION);
      return false;
    }
  }

  int switch_position = enable ? AUX_SWITCH_HIGH : AUX_SWITCH_LOW;
  if (!commandAuxFunction(AUX_FUNC_PRECISION_LOITER, switch_position))
  {
    reportCommandResult(command, "FAILED", "AUX_FUNCTION_COMMAND_FAILED", command_id);
    return false;
  }

  reportCommandResult(command, "ACCEPTED",
                      enable ? "AUX_FUNCTION_ENABLE_ACCEPTED" : "AUX_FUNCTION_DISABLE_ACCEPTED",
                      command_id);

  if (enable)
  {
    // MAVLink acknowledges auxiliary function 39 but does not publish a
    // durable enabled bit. Confirm the observable prerequisite: FC Loiter.
    std::lock_guard<std::mutex> lock(m_mode_confirmation_mutex);
    m_precision_loiter_confirmation_tracker.accept(command_id);
  }

  reportEventFromCallback(enable ? "Precision Loiter enabled" : "Precision Loiter disabled");
  MOOSTraceFromCallback(enable ? "Precision Loiter enabled\n" : "Precision Loiter disabled\n");
  return true;
}

bool UAV_Model::commandAndSetAirSpeed(double speed)
{
  if (isCopter())
  {
    if (commandGroundSpeed(speed))
    {
      m_target_airspeed = speed;
      return true;
    }
    m_warning_system_ptr->queue_monitorWarningForXseconds("Failed to set Copter ground speed to " + doubleToString(speed), WARNING_DURATION);
    return false;
  }

  if (commandSpeed(speed, SPEED_TYPE::SPEED_TYPE_AIRSPEED))
  {
    // setParameterAsync(Parameters::AIRSPEED_TARGET_CRUISE, speed);
    m_target_airspeed = speed;
    return true;
  };
  m_warning_system_ptr->queue_monitorWarningForXseconds("Failed to set airspeed to " + doubleToString(speed), WARNING_DURATION);
  return false;
}

uint64_t UAV_Model::reportCommandResult(const std::string &command,
                                        const std::string &status,
                                        const std::string &detail,
                                        uint64_t command_id) const
{
  if (command_id == 0)
  {
    command_id = m_next_command_id.fetch_add(1);
  }
  if (callbackCommandResult)
  {
    callbackCommandResult(CommandResult{command_id, command, status, detail});
  }
  return command_id;
}

void UAV_Model::pollCommandConfirmations()
{
  ModeConfirmationTracker::Outcome rtl_outcome;
  ModeConfirmationTracker::Outcome fc_loiter_outcome;
  ModeConfirmationTracker::Outcome precision_loiter_outcome;
  TakeoffConfirmationTracker::Outcome takeoff_outcome;
  {
    std::lock_guard<std::mutex> lock(m_mode_confirmation_mutex);
    rtl_outcome = m_rtl_confirmation_tracker.evaluate(
        mts_flight_mode.get() == mavsdk::Telemetry::FlightMode::ReturnToLaunch,
        ModeConfirmationTracker::Clock::now(),
        MODE_CONFIRMATION_TIMEOUT_S,
        MODE_CONFIRMATION_DWELL_S);
    fc_loiter_outcome = m_fc_loiter_confirmation_tracker.evaluate(
        mts_flight_mode.get() == mavsdk::Telemetry::FlightMode::Hold,
        ModeConfirmationTracker::Clock::now(),
        MODE_CONFIRMATION_TIMEOUT_S,
        MODE_CONFIRMATION_DWELL_S);
    precision_loiter_outcome = m_precision_loiter_confirmation_tracker.evaluate(
        mts_flight_mode.get() == mavsdk::Telemetry::FlightMode::Hold,
        ModeConfirmationTracker::Clock::now(),
        MODE_CONFIRMATION_TIMEOUT_S,
        MODE_CONFIRMATION_DWELL_S);

    const auto flight_mode = mts_flight_mode.get();
    const auto landed_state = mts_landed_state.get();
    const bool takeoff_active = isCopter()
                                    ? (flight_mode == mavsdk::Telemetry::FlightMode::Takeoff ||
                                       landed_state == mavsdk::Telemetry::LandedState::TakingOff ||
                                       landed_state == mavsdk::Telemetry::LandedState::InAir)
                                    : (flight_mode == mavsdk::Telemetry::FlightMode::Mission ||
                                       flight_mode == mavsdk::Telemetry::FlightMode::Takeoff);
    const double takeoff_target_m = m_takeoff_confirmation_tracker.targetAltitudeM();
    const bool takeoff_target_reached =
        isCopter() && landed_state == mavsdk::Telemetry::LandedState::InAir &&
        mts_position.get().relative_altitude_m >=
            takeoff_target_m - TAKEOFF_ALTITUDE_TOLERANCE_M;
    const bool takeoff_aborted =
        !m_is_armed || flight_mode == mavsdk::Telemetry::FlightMode::Land ||
        landed_state == mavsdk::Telemetry::LandedState::Landing;
    takeoff_outcome = m_takeoff_confirmation_tracker.evaluate(
        takeoff_active, takeoff_target_reached, takeoff_aborted,
        TakeoffConfirmationTracker::Clock::now(),
        MODE_CONFIRMATION_TIMEOUT_S, TAKEOFF_COMPLETION_TIMEOUT_S);
  }

  if (rtl_outcome.status == ModeConfirmationTracker::OutcomeStatus::Confirmed)
    reportCommandResult("RTL", "CONFIRMED", "FLIGHT_MODE_RTL", rtl_outcome.command_id);
  else if (rtl_outcome.status == ModeConfirmationTracker::OutcomeStatus::TimedOut)
    reportCommandResult("RTL", "TIMED_OUT", "FLIGHT_MODE_NOT_CONFIRMED", rtl_outcome.command_id);

  if (fc_loiter_outcome.status == ModeConfirmationTracker::OutcomeStatus::Confirmed)
    reportCommandResult("LOITER_FC", "CONFIRMED", "FLIGHT_MODE_FC_LOITER", fc_loiter_outcome.command_id);
  else if (fc_loiter_outcome.status == ModeConfirmationTracker::OutcomeStatus::TimedOut)
    reportCommandResult("LOITER_FC", "TIMED_OUT", "FLIGHT_MODE_NOT_CONFIRMED", fc_loiter_outcome.command_id);

  if (precision_loiter_outcome.status == ModeConfirmationTracker::OutcomeStatus::Confirmed)
    reportCommandResult("PRECISION_LOITER", "CONFIRMED", "FC_LOITER_AND_AUX_COMMAND_ACCEPTED", precision_loiter_outcome.command_id);
  else if (precision_loiter_outcome.status == ModeConfirmationTracker::OutcomeStatus::TimedOut)
    reportCommandResult("PRECISION_LOITER", "TIMED_OUT", "FLIGHT_MODE_NOT_CONFIRMED", precision_loiter_outcome.command_id);

  if (takeoff_outcome.confirmed)
    reportCommandResult("TAKEOFF", "CONFIRMED",
                        isCopter() ? "TAKEOFF_ACTIVE" : "FLIGHT_MODE_MISSION",
                        takeoff_outcome.command_id);
  if (takeoff_outcome.completed)
    reportCommandResult("TAKEOFF", "COMPLETED", "TARGET_ALTITUDE_REACHED",
                        takeoff_outcome.command_id);
  if (takeoff_outcome.aborted)
    reportCommandResult("TAKEOFF", "FAILED", "TAKEOFF_ABORTED",
                        takeoff_outcome.command_id);
  if (takeoff_outcome.timeout_phase == TakeoffConfirmationTracker::TimeoutPhase::Activation)
    reportCommandResult("TAKEOFF", "TIMED_OUT", "TAKEOFF_NOT_CONFIRMED",
                        takeoff_outcome.command_id);
  else if (takeoff_outcome.timeout_phase == TakeoffConfirmationTracker::TimeoutPhase::Completion)
    reportCommandResult("TAKEOFF", "TIMED_OUT", "TARGET_ALTITUDE_NOT_REACHED",
                        takeoff_outcome.command_id);
}

bool UAV_Model::commandArmAsync(uint64_t command_id) const
{

  m_action_ptr->arm_async([this, command_id](mavsdk::Action::Result result)
                          {
    if (result != mavsdk::Action::Result::Success) {
        std::stringstream ss;
        ss << "Arming failed: " << result;
        reportCommandResult("ARM", "FAILED", ss.str(), command_id);
        std::cout << ss.str();
        m_warning_system_ptr->queue_monitorWarningForXseconds(ss.str(), WARNING_DURATION);
        return;
    }
    reportCommandResult("ARM", "ACCEPTED", "SUCCESS", command_id); });

  return true;
}

bool UAV_Model::commandDisarmAsync(uint64_t command_id) const
{

  m_action_ptr->disarm_async([this, command_id](mavsdk::Action::Result result)
                             {
      if (result != mavsdk::Action::Result::Success) {
          std::stringstream ss;
          ss << "Disarming failed: " << result;
          reportCommandResult("DISARM", "FAILED", ss.str(), command_id);
          std::cout << ss.str();
          m_warning_system_ptr->queue_monitorWarningForXseconds(ss.str(), WARNING_DURATION);
          return;
      }
      reportCommandResult("DISARM", "ACCEPTED", "SUCCESS", command_id); });

  return true;
}

bool UAV_Model::commandGoToLocationXY(const XYPoint pos, bool holdCurrentAltitudeAGL)
{
  if (!commandGuidedMode())
    return false;

  auto m_position = mts_position.get();

  float alt_msl = m_position.absolute_altitude_m;
  double terrain_altitude = m_position.absolute_altitude_m - m_position.relative_altitude_m;
  if (!holdCurrentAltitudeAGL)
  {
    alt_msl = terrain_altitude + m_target_altitudeAGL;
  }
  mavsdk::Telemetry::Position wpt = {pos.x(), pos.y(), alt_msl};

  m_last_sent_altitudeAGL = alt_msl - terrain_altitude;

  return commandGoToLocation(wpt);
}

bool UAV_Model::commandGoToLocation(const mavsdk::Telemetry::Position &position)
{

  if (!commandGuidedMode())
    return false;

  if (!haveAutorythyToChangeMode())
  {
    std::stringstream ss;
    ss << "Cannot change mode. Do not have autorithy. Flight mode in " << mts_flight_mode;
    m_warning_system_ptr->queue_monitorWarningForXseconds(ss.str(), WARNING_DURATION);
    return false;
  }

  double loiter_direction = 0; // 0 for clockwise, 1 for counter clockwise

  // blocking //TODO modify so it is non
  auto res = m_action_ptr->goto_location(position.latitude_deg, position.longitude_deg, position.absolute_altitude_m, loiter_direction);

  if (res != mavsdk::Action::Result::Success)
  {
    std::stringstream ss;
    ss << "goto_location failed: " << res;
    m_warning_system_ptr->queue_monitorWarningForXseconds(ss.str(), WARNING_DURATION);
    return false;
  }

  Logger::info("UAV_Model: goto_location succeeded");
  // MOOSTraceFromCallback("goto_location succeeded\n");

  return true;
}

// void UAV_Model::commandGoToLocation_async(const mavsdk::Telemetry::Position &position, std::function<void(bool)> callback)
// {
//   std::async(std::launch::async,
//              [position, callback, this]()
//              {
//                bool res = commandGoToLocation(position);
//                if (callback)
//                  callback(res);
//              });
// }

// void UAV_Model::commandGoToLocationXY_async(const XYPoint pos, bool holdCurrentAltitudeAGL, std::function<void(bool)> callback)
// {
//   std::async(std::launch::async,
//              [pos, holdCurrentAltitudeAGL, callback, this]()
//              {
//                bool res = commandGoToLocationXY(pos, holdCurrentAltitudeAGL);
//                if (callback)
//                  callback(res);
//              });
// }

bool UAV_Model::commandAndSetAltitudeAGL(double altitudeAGL_m)
{
  if (isCopter())
  {
    if (altitudeAGL_m < IN_AIR_HIGHT_THRESHOLD)
    {
      std::stringstream ss;
      ss << "Altitude, " << doubleToString(altitudeAGL_m) << " m, is too low. Below in air threshold: " << IN_AIR_HIGHT_THRESHOLD;
      m_warning_system_ptr->queue_monitorWarningForXseconds(ss.str(), WARNING_DURATION);
      return false;
    }

    m_target_altitudeAGL = altitudeAGL_m;
    auto position = mts_position.get();
    double terrain_altitude = position.absolute_altitude_m - position.relative_altitude_m;
    mavsdk::Telemetry::Position target_position = {
        position.latitude_deg,
        position.longitude_deg,
        static_cast<float>(terrain_altitude + altitudeAGL_m)};
    m_last_sent_altitudeAGL = altitudeAGL_m;
    return commandGoToLocation(target_position);
  }

  if (!commandGuidedMode(true))
    return false;

  m_target_altitudeAGL = altitudeAGL_m;
  return commandChangeAltitude_Guided(altitudeAGL_m, true);
}

bool UAV_Model::commandSpeed(double speed_m_s, SPEED_TYPE speed_type)
{

  if (!m_in_air)
  {
    m_warning_system_ptr->queue_monitorWarningForXseconds("UAV is not in air! Cannot send speed", WARNING_DURATION);
    return false;
  }

  if (mts_flight_mode.get() == mavsdk::Telemetry::FlightMode::ReturnToLaunch)
  {
    m_warning_system_ptr->queue_monitorWarningForXseconds("UAV is in RTL mode! Cannot command speed", WARNING_DURATION);
    return false;
  }

  if (speed_type == SPEED_TYPE::SPEED_TYPE_AIRSPEED)
  {
    // check if speed is within bounds
    if (speed_m_s < mts_polled_params.get().min_airspeed || speed_m_s > mts_polled_params.get().max_airspeed)
    {
      std::stringstream ss;
      ss << "Speed out of bounds: " << speed_m_s << " min: " << mts_polled_params.get().min_airspeed << " max: " << mts_polled_params.get().max_airspeed;
      m_warning_system_ptr->queue_monitorWarningForXseconds(ss.str(), WARNING_DURATION);
      return false;
    }

    if (isGuidedMode())
    {
      return commandChangeSpeed_Guided(speed_m_s, speed_type);
    }
  }
  // if we're in failsafe modes (e.g., RTL, LOITER) or in pilot
  // controlled modes (e.g., MANUAL, TRAINING)
  // this command should is ignored and failed (since it comes in from GCS
  // or a companion computer)

  mavsdk::MavlinkPassthrough::CommandLong command_mode;
  command_mode.command = MAV_CMD_DO_CHANGE_SPEED;
  command_mode.target_sysid = m_system_ptr->get_system_id();
  command_mode.target_compid = MAV_COMP_ID_AUTOPILOT1; // system.value()->component_ids().front(); // assuming first component is autopilot
  command_mode.param1 = speed_type;
  command_mode.param2 = speed_m_s;
  command_mode.param3 = -1; // -1 throttle no change
  command_mode.param4 = -1; // Not used
  command_mode.param5 = -1; //
  command_mode.param6 = -1; //
  command_mode.param7 = -1; //

  // blocking
  auto result = m_mavPass_ptr->send_command_long(command_mode);

  // auto res = m_action_ptr->set_takeoff_speed(speed_m_s);

  if (result != mavsdk::MavlinkPassthrough::Result::Success)
  {
    std::stringstream ss;
    ss << "command Speed error: " << result << " with speed " << speed_m_s << " and type ";
    switch (speed_type)
    {
    case SPEED_TYPE_AIRSPEED:
      ss << "SPEED_TYPE_AIRSPEED";
      break;
    case SPEED_TYPE_GROUNDSPEED:
      ss << "SPEED_TYPE_GROUNDSPEED";
      break;
    default:
      ss << doubleToString(speed_type);
      break;
    }
    m_warning_system_ptr->queue_monitorWarningForXseconds(ss.str(), WARNING_DURATION);
    return false;
  }

  MOOSTraceFromCallback("command Speed succeeded\n");

  return true;
}

bool UAV_Model::commandChangeAltitude_Guided(double altitude_m, bool relativeAlt, double vrate_ms)
{

  if (!m_in_air)
  {
    m_warning_system_ptr->queue_monitorWarningForXseconds("UAV is not in air! Cannot send altitude", WARNING_DURATION);
    return false;
  }

  if (mts_flight_mode.get() == mavsdk::Telemetry::FlightMode::ReturnToLaunch)
  {
    m_warning_system_ptr->queue_monitorWarningForXseconds("UAV is in RTL mode! Cannot command altitude", WARNING_DURATION);
    return false;
  }

  if (!isGuidedMode() && !commandGuidedMode())
  {
    m_warning_system_ptr->queue_monitorWarningForXseconds("UAV is not in guided mode! Cannot send altitude", WARNING_DURATION);
    return false;
  }

  if (altitude_m < IN_AIR_HIGHT_THRESHOLD)
  {
    std::stringstream ss;
    ss << "Altitude, " << doubleToString(altitude_m) << " m, is too low. Below in air threshold: " << IN_AIR_HIGHT_THRESHOLD;
    m_warning_system_ptr->queue_monitorWarningForXseconds(ss.str(), WARNING_DURATION);
    return false;
  }

  // mavsdk::MavlinkCommandSender

  mavsdk::MavlinkPassthrough::CommandInt command_mode;
  command_mode.command = MAV_CMD_GUIDED_CHANGE_ALTITUDE;
  command_mode.target_sysid = m_system_ptr->get_system_id();
  command_mode.target_compid = MAV_COMP_ID_AUTOPILOT1; // assuming first component is autopilot
  command_mode.frame = (relativeAlt) ? MAV_FRAME_GLOBAL_RELATIVE_ALT : MAV_FRAME_GLOBAL;
  command_mode.param3 = (vrate_ms >= 0) ? vrate_ms : 0; // altitude rate of change (0 for max)
  command_mode.z = altitude_m;                          // altitude in meters

  command_mode.param1 = -1; // unused
  command_mode.param2 = -1; // unused
  command_mode.param4 = -1; // unused
  command_mode.x = -1;      // unused
  command_mode.y = -1;      // unused

  mavsdk::Action::ResultCallback retultFuncion = [altitude_m, this](mavsdk::Action::Result result)
  {
    std::cout << "Result: " << result << std::endl;

    if (result != mavsdk::Action::Result::Success)
    {
      std::stringstream ss;
      ss << "command Change Altitude error: " << result << " with altitude " << altitude_m;
      m_warning_system_ptr->queue_monitorWarningForXseconds(ss.str(), WARNING_DURATION);
    }
    else
    {
      reportEventFromCallback("command Change Altitude succeeded\n");
      MOOSTraceFromCallback("command Change Altitude succeeded\n");
    }
  };

  m_action_ptr->send_command_async(command_mode, retultFuncion);

  // blocking
  // auto result = m_mavPass_ptr->send_command_int(command_mode);

  // if (result != mavsdk::MavlinkPassthrough::Result::Success)
  // {
  //   std::stringstream ss;
  //   ss << "command Change Altitude error: " << result << " with altitude " << altitude_m;
  //   m_warning_system_ptr->monitorWarningForXseconds(ss.str(), WARNING_DURATION);
  //   return false;
  // }

  return true;
}

bool UAV_Model::commandChangeHeading_Guided(double hdg_deg, HEADING_TYPE hdg_type)
{

  if (!m_in_air)
  {
    m_warning_system_ptr->queue_monitorWarningForXseconds("UAV is not in air! Cannot send heading", WARNING_DURATION);
    return false;
  }

  if (mts_flight_mode.get() == mavsdk::Telemetry::FlightMode::ReturnToLaunch)
  {
    m_warning_system_ptr->queue_monitorWarningForXseconds("UAV is in RTL mode! Cannot command heading", WARNING_DURATION);
    return false;
  }

  if (!isGuidedMode() && !commandGuidedMode())
  {
    m_warning_system_ptr->queue_monitorWarningForXseconds("UAV is not in guided mode! Cannot send heading", WARNING_DURATION);
    return false;
  }

  if (hdg_deg < 0 || hdg_deg > 360)
  {
    std::stringstream ss;
    ss << "Heading, " << doubleToString(hdg_deg) << " deg, is out of bounds. Must be between 0 and 360";
    m_warning_system_ptr->queue_monitorWarningForXseconds(ss.str(), WARNING_DURATION);
    return false;
  }

  mavsdk::MavlinkPassthrough::CommandInt command_mode;
  command_mode.command = MAV_CMD_GUIDED_CHANGE_HEADING;
  command_mode.target_sysid = m_system_ptr->get_system_id();
  command_mode.target_compid = MAV_COMP_ID_AUTOPILOT1; // first component is autopilot
  command_mode.param1 = hdg_type;
  command_mode.param2 = hdg_deg;
  command_mode.param3 = MAX_CENTRIPITAL_ACC_TURN; // Maximum centripetal accelearation (m/s^2)

  command_mode.param4 = -1;              // unused
  command_mode.x = -1;                   // unused
  command_mode.y = -1;                   // unused
  command_mode.z = -1;                   // unused
  command_mode.frame = MAV_FRAME_GLOBAL; // unused
  // blocking
  auto result = m_mavPass_ptr->send_command_int(command_mode);

  if (result != mavsdk::MavlinkPassthrough::Result::Success)
  {
    std::stringstream ss;
    ss << "command Change Heading error: " << result << " with heading " << hdg_deg << " and type ";
    switch (hdg_type)
    {
    case HEADING_TYPE::HEADING_TYPE_COURSE_OVER_GROUND:
      ss << "HEADING_TYPE_COURSE_OVER_GROUND";
      break;
    case HEADING_TYPE::HEADING_TYPE_HEADING:
      ss << "HEADING_TYPE_HEADING";
      break;
    default:
      ss << "HEADING_TYPE_DEFAULT";
      break;
    }
    m_warning_system_ptr->queue_monitorWarningForXseconds(ss.str(), WARNING_DURATION);
    return false;
  }

  m_is_hold_course_guided_set = true;
  MOOSTraceFromCallback("command Change Heading succeeded\n");

  return true;
}

bool UAV_Model::commandChangeSpeed_Guided(double speed_m_s, SPEED_TYPE speed_type)
{

  if (!m_in_air)
  {
    m_warning_system_ptr->queue_monitorWarningForXseconds("UAV is not in air! Cannot send speed", WARNING_DURATION);
    return false;
  }

  if (!isGuidedMode() && !commandGuidedMode())
  {
    m_warning_system_ptr->queue_monitorWarningForXseconds("UAV is not in guided mode! Cannot send speed", WARNING_DURATION);
    return false;
  }

  mavsdk::MavlinkPassthrough::CommandInt command_mode;
  command_mode.command = MAV_CMD_GUIDED_CHANGE_SPEED;
  command_mode.target_sysid = m_system_ptr->get_system_id();
  command_mode.target_compid = MAV_COMP_ID_AUTOPILOT1; // first component is autopilot
  command_mode.param1 = speed_type;
  command_mode.param2 = speed_m_s;
  command_mode.param3 = 0; // Acceleration rate, 0 to take effect instantly

  command_mode.param4 = -1;              // unused
  command_mode.x = -1;                   // unused
  command_mode.y = -1;                   // unused
  command_mode.z = -1;                   // unused
  command_mode.frame = MAV_FRAME_GLOBAL; // unused

  // blocking
  auto result = m_mavPass_ptr->send_command_int(command_mode);

  if (result != mavsdk::MavlinkPassthrough::Result::Success)
  {
    std::stringstream ss;
    ss << "command Change Speed error: " << result << " with speed " << speed_m_s << " and type ";
    switch (speed_type)
    {
    case SPEED_TYPE::SPEED_TYPE_AIRSPEED:
      ss << "SPEED_TYPE_AIRSPEED";
      break;
    case SPEED_TYPE::SPEED_TYPE_GROUNDSPEED:
      ss << "SPEED_TYPE_GROUNDSPEED";
      break;
    default:
      ss << doubleToString(speed_type);
      break;
    }
    m_warning_system_ptr->queue_monitorWarningForXseconds(ss.str(), WARNING_DURATION);
    return false;
  }

  MOOSTraceFromCallback("command Change Speed succeeded\n");

  return true;
}

void UAV_Model::setHeadingWyptFromHeading(double heading_deg)
{

  heading_deg = angle360(heading_deg); // make sure heading is within 0-360

  static const auto deg_to_rad = [](double degrees)
  { return degrees * deg2rad; };
  static const auto rad_to_deg = [](double radians)
  { return radians * rad2deg; };

  // Convert heading to radians
  double heading_rad = deg_to_rad(heading_deg);

  // Get current position in radians
  double lat_rad = deg_to_rad(mts_position.get().latitude_deg);
  double lon_rad = deg_to_rad(mts_position.get().longitude_deg);

  // Calculate new latitude using the bearing formula
  double new_lat_rad = std::asin(
      std::sin(lat_rad) * std::cos(DISTANCE_TO_HEADING_WAYPOINT / EARTH_RADIUS) +
      std::cos(lat_rad) * std::sin(DISTANCE_TO_HEADING_WAYPOINT / EARTH_RADIUS) * std::cos(heading_rad));

  // Calculate new longitude
  double new_lon_rad = lon_rad + std::atan2(
                                     std::sin(heading_rad) * std::sin(DISTANCE_TO_HEADING_WAYPOINT / EARTH_RADIUS) * std::cos(lat_rad),
                                     std::cos(DISTANCE_TO_HEADING_WAYPOINT / EARTH_RADIUS) - std::sin(lat_rad) * std::sin(new_lat_rad));

  // Convert the new latitude and longitude back to degrees
  double new_lat_deg = rad_to_deg(new_lat_rad);
  double new_lon_deg = rad_to_deg(new_lon_rad);

  mts_course_waypoint_coord->set_vertex(new_lat_deg, new_lon_deg);
}

bool UAV_Model::commandAndSetCourse(double heading, bool isAllowed)
{

  if (!m_in_air)
  {
    m_warning_system_ptr->queue_monitorWarningForXseconds("UAV is not in air! Cannot send heading", WARNING_DURATION);
    return false;
  }

  m_target_course = heading;

  if (isCopter())
  {
    if (!isAllowed)
    {
      m_warning_system_ptr->queue_monitorWarningForXseconds("Helm must be active to command heading", WARNING_DURATION);
      return false;
    }

    heading = angle360(heading);
    mavsdk::MavlinkPassthrough::CommandLong command_mode;
    command_mode.command = MAV_CMD_CONDITION_YAW;
    command_mode.target_sysid = m_system_ptr->get_system_id();
    command_mode.target_compid = MAV_COMP_ID_AUTOPILOT1;
    command_mode.param1 = heading;
    command_mode.param2 = 30.0; // yaw rate deg/s
    command_mode.param3 = 0.0;  // shortest direction
    command_mode.param4 = 0.0;  // absolute heading
    command_mode.param5 = 0.0;
    command_mode.param6 = 0.0;
    command_mode.param7 = 0.0;

    auto result = m_mavPass_ptr->send_command_long(command_mode);
    if (result != mavsdk::MavlinkPassthrough::Result::Success)
    {
      std::stringstream ss;
      ss << "Copter yaw command error: " << result << " with heading " << heading;
      m_warning_system_ptr->queue_monitorWarningForXseconds(ss.str(), WARNING_DURATION);
      return false;
    }

    MOOSTraceFromCallback("Copter yaw command succeeded\n");
    return true;
  }

  if (/*isGuidedMode() &&*/ isAllowed)
  {
    return commandChangeHeading_Guided(heading, HEADING_TYPE::HEADING_TYPE_COURSE_OVER_GROUND);
  }
  else
  {

    std::stringstream ss;
    ss << "Helm must be active to command heading";
    m_warning_system_ptr->queue_monitorWarningForXseconds(ss.str(), WARNING_DURATION);
    return false;
  }

  Logger::error("UAV_Model: commandAndSetHeading using goto location with waypointHeading");
  setHeadingWyptFromHeading(heading);

  // Command the plane to the new location
  return commandGoToLocationXY(mts_course_waypoint_coord);
}

///////////////////////////////////////////////////////
///// Threading
///////////////////////////////////////////////////////
void UAV_Model::startCommandSender()
{
  if (!m_running)
  {
    m_running = true;
    m_thread = std::thread([this]()
                           { runCommandsender(); });
  }
}

void UAV_Model::runCommandsender()
{

  // At startUP
  subscribeToTelemetry();
  pollAllParametersAsync();

  // Perform commands
  while (m_running)
  {
    std::unique_ptr<CommandBase> cmd;
    {
      std::unique_lock lock(m_queue_mutex);
      m_thread_cv.wait(lock, [this]()
                       { return !m_command_queue.empty() || !m_running || m_sendValuesEnabled; });

      if (!m_running)
        break;

      if (!m_command_queue.empty())
      {
        cmd = std::move(m_command_queue.front());
        m_command_queue.pop();
      }
    }

    if (cmd)
    {
      cmd->execute(*this);
      pollAllParametersAsync();

      Logger::info("UAV_Model THREAD: isGuidedMode: " + boolToString(isGuidedMode()) + " is_hold_heading_guided_set: " + boolToString(m_is_hold_course_guided_set));
      if (!isGuidedMode())
      {
        m_is_hold_course_guided_set = false;
      }
    }

    if (m_sendValuesEnabled && sendDesiredValues)
    {
      sendDesiredValues(*this, false);
    }

    auto sleep = !m_command_queue.empty() ? 10 : 100;
    std::this_thread::sleep_for(std::chrono::milliseconds(sleep));
  }
}

void UAV_Model::registerSendDesiredValuesFunction(std::function<void(UAV_Model &, bool)> func)
{
  std::lock_guard lock(m_sendDesiredValues_mutex);
  this->sendDesiredValues = func;
}

void UAV_Model::enableSendDesiredValues(bool enable)
{
  m_sendValuesEnabled = enable;
  m_thread_cv.notify_one();
};

void UAV_Model::stopCommandSender()
{
  m_running = false;
  m_thread_cv.notify_all();
  if (m_thread.joinable())
    m_thread.join();
}

///////////////////////////////////////////////////////
///// Mission Start
///////////////////////////////////////////////////////

mavsdk::MissionRaw::MissionItem make_mission_item_wp(
    float latitude_deg1e7,
    float longitude_deg1e7,
    int32_t altitude_m,
    float param1,
    MAV_FRAME frame,
    MAV_CMD command,
    float p2,
    float p3)
{
  // WARNING this is done in consideration of CLEAN!! mission
  static uint32_t seq_num = 0;
  mavsdk::MissionRaw::MissionItem new_item{};
  new_item.seq = seq_num;
  new_item.frame = static_cast<uint32_t>(frame);
  new_item.command = static_cast<uint32_t>(command);
  new_item.param1 = param1;
  new_item.param2 = p2;
  new_item.param3 = p3;
  new_item.x = latitude_deg1e7 * 1e7;
  new_item.y = longitude_deg1e7 * 1e7;
  new_item.z = altitude_m;
  new_item.mission_type = MAV_MISSION_TYPE_MISSION;
  new_item.autocontinue = 1;

  if (seq_num == 1)
  {
    new_item.current = 1;
  }

  seq_num++;

  return new_item;
}

bool create_missionPlan(std::vector<mavsdk::MissionRaw::MissionItem> &mission_plan, double lat_deg_home, double lon_deg_home)
{

  // in case of ardupilot we want to set lat lon to waypoint 0
  // Home position (same as original, set to lat/lon home)
  mission_plan.push_back(make_mission_item_wp( // 0
      lat_deg_home,                            // lat home
      lon_deg_home,                            // lon home
      100,                                     // alt home
      0,
      MAV_FRAME_GLOBAL_RELATIVE_ALT,
      MAV_CMD_NAV_WAYPOINT));

  // Waypoint 1
  mission_plan.push_back(make_mission_item_wp( // 1 takeoff
      lat_deg_home + 0.003429,                 // 382.4 meters north
      lon_deg_home - 0.000534,                 // 49 meters west
      static_cast<int32_t>(41),  // Changed from 41.03 - function expects int32_t altitude_m
      15,
      MAV_FRAME_GLOBAL_RELATIVE_ALT,
      MAV_CMD_NAV_TAKEOFF));

  // Waypoint 2
  mission_plan.push_back(make_mission_item_wp( // 2
      lat_deg_home + 0.003677,                 // 409.2 meters north
      lon_deg_home - 0.003845,                 // 341.4 meters west
      120.00,
      0,
      MAV_FRAME_GLOBAL_RELATIVE_ALT,
      MAV_CMD_NAV_WAYPOINT));

  // Waypoint 3
  mission_plan.push_back(make_mission_item_wp( // 3
      lat_deg_home - 0.003201,                 // 356.8 meters south
      lon_deg_home - 0.002996,                 // 265.2 meters west
      200.00,
      0,
      MAV_FRAME_GLOBAL_RELATIVE_ALT,
      MAV_CMD_NAV_WAYPOINT));

  // Waypoint 4
  mission_plan.push_back(make_mission_item_wp( // 4
      lat_deg_home - 0.002869,                 // 320.4 meters south
      lon_deg_home - 0.000656,                 // 57.2 meters west
      210.00,
      0,
      MAV_FRAME_GLOBAL_RELATIVE_ALT,
      MAV_CMD_NAV_WAYPOINT));

  // Waypoint 5
  mission_plan.push_back(make_mission_item_wp( // 5
      lat_deg_home + 0.004198,                 // 444.2 meters north
      lon_deg_home - 0.001480,                 // 131.6 meters west
      130.00,
      0,
      MAV_FRAME_GLOBAL_RELATIVE_ALT,
      MAV_CMD_NAV_WAYPOINT));

  // Waypoint 6 (Speed Change)
  mission_plan.push_back(make_mission_item_wp( // 6
      lat_deg_home - 0.002869,                 // relative to waypoint 4
      lon_deg_home - 0.000656,                 // relative to waypoint 4
      110.00,
      SPEED_TYPE_AIRSPEED,
      MAV_FRAME_GLOBAL_RELATIVE_ALT,
      MAV_CMD_DO_CHANGE_SPEED,
      6)); // speed 6 m/s

  // Waypoint 7 (repeat of WP5)
  mission_plan.push_back(make_mission_item_wp( // 7
      lat_deg_home + 0.004198,                 // same as waypoint 5
      lon_deg_home - 0.001480,
      100.00,
      0,
      MAV_FRAME_GLOBAL_RELATIVE_ALT,
      MAV_CMD_NAV_WAYPOINT));

  // Waypoint 8
  mission_plan.push_back(make_mission_item_wp( // 8
      lat_deg_home + 0.002396,                 // 267.2 meters north
      lon_deg_home - 0.000352,                 // 31.1 meters west
      41.00,
      0,
      MAV_FRAME_GLOBAL_RELATIVE_ALT,
      MAV_CMD_NAV_WAYPOINT));

  // Landing waypoint (back to home)
  mission_plan.push_back(make_mission_item_wp( // 9
      lat_deg_home,                            // return to home position
      lon_deg_home,
      0.00,
      1, // Minimum abort altitude
      MAV_FRAME_GLOBAL_RELATIVE_ALT,
      MAV_CMD_NAV_LAND,
      PRECISION_LAND_MODE_OPPORTUNISTIC));
  return true;
}
