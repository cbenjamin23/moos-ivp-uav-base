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
#include "Logger.h"
//------------------------------------------------------------------------
// Constructor

UAV_Model::UAV_Model(std::shared_ptr<WarningSystem> ws) : m_mavsdk_ptr{std::make_shared<mavsdk::Mavsdk>(mavsdk::Mavsdk::Configuration{mavsdk::Mavsdk::ComponentType::GroundStation})},

                                                          callbackMOOSTrace{nullptr},
                                                          callbackReportRunW{nullptr},
                                                          callbackRetractRunW{nullptr},
                                                          callbackReportEvent{nullptr},

                                                          m_is_hold_heading_guided_set{false},
                                                          m_health_all_ok{false},
                                                          m_is_armed{false},
                                                          m_in_air{false},
                                                          m_target_altitudeAGL{120.0},
                                                          m_target_airspeed{0.0},
                                                          m_target_heading{0.0},
                                                          m_last_sent_altitudeAGL{double(NAN)},
                                                          mts_position{mavsdk::Telemetry::Position()},
                                                          mts_attitude_ned{mavsdk::Telemetry::EulerAngle()},
                                                          m_velocity_ned{mavsdk::Telemetry::VelocityNed()},
                                                          mts_battery{mavsdk::Telemetry::Battery()},
                                                          mts_flight_mode{mavsdk::Telemetry::FlightMode::Unknown},
                                                          mts_home_coord{XYPoint(0, 0)},
                                                          mts_current_loiter_coord{XYPoint(0, 0)},
                                                          mts_next_waypoint_coord{XYPoint(0, 0)},
                                                          mts_heading_waypoint_coord{XYPoint(0, 0)},
                                                          mts_polled_params{PolledParameters()}

{
  // Initalize the configuration variables

  m_warning_system_ptr = ws;
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

  // Poll Cruise speed and set to target airspeed
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

  return true;
}

bool UAV_Model::setUpMission(bool onlyRegisterHome)
{

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

  if (download_result.first != mavsdk::MissionRaw::Result::Success)
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

  auto start_result = m_mission_raw_ptr->start_mission();

  if (start_result != mavsdk::MissionRaw::Result::Success)
  {
    m_warning_system_ptr->queue_monitorWarningForXseconds("Failed to start mission", WARNING_DURATION);
    return false;
  }

  MOOSTraceFromCallback("Mission started\n");
  return true;
}

bool UAV_Model::sendArmCommandIfHealthyAndNotArmed_async() const
{
  if (m_health_all_ok && !m_is_armed)
  {
    commandArmAsync();
    return true;
  }

  m_warning_system_ptr->queue_monitorWarningForXseconds("UAV is not healthy or is already armed", WARNING_DURATION);
  return false;
}

bool UAV_Model::subscribeToTelemetry()
{

  m_telemetry_ptr->subscribe_armed([&](bool is_armed)
                                   { m_is_armed = is_armed; });

  m_telemetry_ptr->subscribe_health_all_ok([&, this](bool is_health_all_ok)
                                           { this->m_health_all_ok = is_health_all_ok; });

  m_telemetry_ptr->subscribe_position([&](mavsdk::Telemetry::Position position)
                                      {
                                        mts_position = position;

                                        m_in_air = (position.relative_altitude_m >= IN_AIR_HIGHT_THRESHOLD); });

  m_telemetry_ptr->subscribe_attitude_euler([&](mavsdk::Telemetry::EulerAngle attitude_ned)
                                            { mts_attitude_ned = attitude_ned; });

  m_telemetry_ptr->subscribe_velocity_ned([&](mavsdk::Telemetry::VelocityNed vel)
                                          { m_velocity_ned = vel; });

  m_telemetry_ptr->subscribe_battery([&](mavsdk::Telemetry::Battery battery)
                                     { mts_battery = battery; });

  m_telemetry_ptr->subscribe_flight_mode([&](mavsdk::Telemetry::FlightMode flight_mode)
                                         { mts_flight_mode = flight_mode; });

  // gives wrong data
  // m_telemetry_ptr->subscribe_in_air([&](bool in_air) {
  //   m_in_air = in_air;
  // });

  return true;
}

///////////////////////////////////
/////////////  POLLING  //////////
///////////////////////////////////

void UAV_Model::pollAllParametersAsync()
{

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

  if (m_flight_mode == mavsdk::Telemetry::FlightMode::Mission                                                                   //   Mission mode is ardupilots AUTO mode
      || m_flight_mode == mavsdk::Telemetry::FlightMode::Hold // Also loiter mode
      || m_flight_mode == mavsdk::Telemetry::FlightMode::Land 
      || m_flight_mode == mavsdk::Telemetry::FlightMode::Offboard       // Previous guided mode
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

  if (m_is_hold_heading_guided_set && !alt_hold)
  {
    mavsdk::Action::Result result = m_action_ptr->set_flight_mode_auto();

    if (result != mavsdk::Action::Result::Success)
    {
      std::stringstream ss;
      ss << "Failed to exit Guided after hold is activated: " << result;
      m_warning_system_ptr->queue_monitorWarningForXseconds(ss.str(), WARNING_DURATION);
      return false;
    }

    m_is_hold_heading_guided_set = false;
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

bool UAV_Model::commandReturnToLaunchAsync() const
{

  if (!haveAutorythyToChangeMode())
  {
    std::stringstream ss;
    ss << "Cannot change mode. Do not have autorithy. Flight mode in " << mts_flight_mode;
    m_warning_system_ptr->queue_monitorWarningForXseconds(ss.str(), WARNING_DURATION);
    return false;
  }

  m_action_ptr->return_to_launch_async([&, this](mavsdk::Action::Result result)
                                       {
    if (result != mavsdk::Action::Result::Success) {
        std::stringstream ss;
        ss << "Return to launch failed: " << result;
        m_warning_system_ptr->queue_monitorWarningForXseconds(ss.str(), WARNING_DURATION);
        return;
    } });

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

bool UAV_Model::commandAndSetAirSpeed(double speed)
{
  if (commandSpeed(speed, SPEED_TYPE::SPEED_TYPE_AIRSPEED))
  {
    // setParameterAsync(Parameters::AIRSPEED_TARGET_CRUISE, speed);
    m_target_airspeed = speed;
    return true;
  };
  m_warning_system_ptr->queue_monitorWarningForXseconds("Failed to set airspeed to " + doubleToString(speed), WARNING_DURATION);
  return false;
}

bool UAV_Model::commandArmAsync() const
{

  m_action_ptr->arm_async([&, this](mavsdk::Action::Result result)
                          {
    // MOOSTrace("Arming result: %d\n", result);
    if (result != mavsdk::Action::Result::Success) {
        std::stringstream ss;
        ss << "Arming failed: " << result << '\n';
        std::cout << ss.str();
        m_warning_system_ptr->queue_monitorWarningForXseconds(ss.str(), WARNING_DURATION);
    } });

  return true;
}

bool UAV_Model::commandDisarmAsync() const
{

  m_action_ptr->disarm_async([&, this](mavsdk::Action::Result result)
                             {
      // MOOSTrace("Disarming result: %d\n", result);
      if (result != mavsdk::Action::Result::Success) {
          std::stringstream ss;
          ss << "Disarming failed: " << result << '\n';
          std::cout << ss.str();
          m_warning_system_ptr->queue_monitorWarningForXseconds(ss.str(), WARNING_DURATION);
      } });

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
  if (!commandGuidedMode(true))
    return false;

  m_target_altitudeAGL = altitudeAGL_m;
  return commandChangeAltitude_Guided(altitudeAGL_m, true);
}

bool UAV_Model::commandSpeed(double speed_m_s, SPEED_TYPE speed_type) const
{

  if (!m_in_air)
  {
    m_warning_system_ptr->queue_monitorWarningForXseconds("UAV is not in air! Cannot send speed", WARNING_DURATION);
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

bool UAV_Model::commandChangeAltitude_Guided(double altitude_m, bool relativeAlt, double vrate_ms) const
{

  if (!m_in_air)
  {
    m_warning_system_ptr->queue_monitorWarningForXseconds("UAV is not in air! Cannot send altitude", WARNING_DURATION);
    return false;
  }

  if (!isGuidedMode())
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

  if (!isGuidedMode())
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

  m_is_hold_heading_guided_set = true;
  MOOSTraceFromCallback("command Change Heading succeeded\n");

  return true;
}

bool UAV_Model::commandChangeSpeed_Guided(double speed_m_s, SPEED_TYPE speed_type) const
{

  if (!m_in_air)
  {
    m_warning_system_ptr->queue_monitorWarningForXseconds("UAV is not in air! Cannot send speed", WARNING_DURATION);
    return false;
  }

  if (!isGuidedMode())
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

  mts_heading_waypoint_coord->set_vertex(new_lat_deg, new_lon_deg);
}

bool UAV_Model::commandAndSetHeading(double heading, bool isAllowed)
{

  if (!m_in_air)
  {
    m_warning_system_ptr->queue_monitorWarningForXseconds("UAV is not in air! Cannot send heading", WARNING_DURATION);
    return false;
  }

  m_target_heading = heading;

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
  return commandGoToLocationXY(mts_heading_waypoint_coord);
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
    
      Logger::info("UAV_Model THREAD: isGuidedMode: " + boolToString(isGuidedMode()) + " is_hold_heading_guided_set: " + boolToString(m_is_hold_heading_guided_set));
      if (!isGuidedMode())
      {
        m_is_hold_heading_guided_set = false;
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
      41.03,
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
