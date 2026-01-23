/************************************************************/
/*    NAME: Steve Carter Feujo Nomeny                       */
/*    ORGN: NTNU, MIT                                       */
/*    FILE: ArduBridge.cpp                                  */
/*    DATE: September 9th, 2024                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"

// #include "NodeMessage.h"
#include "XYMarker.h"
#include "XYVector.h"

#include "ArduBridge.h"
#include "definitions.h"

#include "Logger.h"

//---------------------------------------------------------
// Constructor()

ArduBridge::ArduBridge()
    : m_do_fly_to_waypoint{false},
      m_do_takeoff{false},
      m_cli_arg{},
      m_do_change_speed_pair{std::make_pair(false, 0)},
      m_do_change_course_pair{std::make_pair(false, 0)},
      m_do_change_altitude_pair{std::make_pair(false, 0)},
      m_do_reset_speed{false},
      m_do_return_to_launch{false},
      m_do_autoland{false},
      m_do_loiter_pair{false, "default"},
      m_do_arm{false},
      m_do_helm_survey{false},
      m_do_helm_voronoi{false},
      m_is_simulation{false},
      m_command_groundSpeed{false},
      m_warning_system_ptr{std::make_shared<WarningSystem>(
          [this](const std::string msg)
          { this->reportRunWarning(msg); },
          [this](const std::string msg)
          { this->retractRunWarning(msg); })},
      m_autopilot_mode{AutopilotHelmMode::HELM_PARKED}
{
  m_uav_model.registerWarningSystem(m_warning_system_ptr);
  m_uav_model.setCallbackReportEvent([this](const std::string msg)
                                     { this->reportEvent(msg); });
  m_uav_model.setCallbackMOOSTrace([this](const std::string msg)
                                   { MOOSTrace(msg); });

  m_uav_prefix = "UAV";

  m_vcolor = "yellow";

  m_waypointsXY_mission = {{-390, 10}, {55, 381}, {333, 35}, {-100, -290}};

  initializeStateTransitionFunctions();

  // Get the home directory from the environment variable
  auto home_dir = getenv("HOME");
  if (home_dir == nullptr)
  {
    Logger::error("Error: Could not get the home directory.");
    std::cerr << "Error: Could not get the home directory." << std::endl;
    return;
  }
  std::string save_path = std::string(home_dir) + "/moos-ivp-uav/missions/pArduBrigeLog_" + m_vname + ".log";
  Logger::configure(save_path);
}

//---------------------------------------------------------
// Destructor

ArduBridge::~ArduBridge()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool ArduBridge::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for (p = NewMail.begin(); p != NewMail.end(); p++)
  {
    CMOOSMsg &msg = *p;
    std::string key = msg.GetKey();

#if 0 // Keep these around just for template
    std::string comm  = msg.GetCommunity();
    std::string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif

    if (msg.GetSource() == m_sAppName)
      continue;
    

    if (key == "DESIRED_HEADING") // DESIRED_COURSE comes in as DESIRED_HEADING from pHelm for legacy reasons
    {
      double course = msg.GetDouble();
      m_helm_desiredValues->updateDesiredCourse(course);
    }
    else if (key == "DESIRED_SPEED")
    {
      double speed = msg.GetDouble();
      m_helm_desiredValues->updateDesiredSpeed(speed);
    }
    else if (key == "DESIRED_ALTITUDE")
    {
      double alt = msg.GetDouble();
      m_helm_desiredValues->updateDesiredAltitude(alt);
    }
    else if (key == "NEXT_WAYPOINT")
    {
      Logger::info("OnNewMail NEXT_WAYPOINT: " + msg.GetString());
      std::string wp_str = msg.GetString();
      double lat, lon, x, y;
      std::string vname;
      if (parseCoordinateString(wp_str, lat, lon, x, y, vname))
      {
        // Logger::info("OnNewMail Parsed info: " + doubleToString(lat) + " " + doubleToString(lon) + " " + doubleToString(x) + " " + doubleToString(y) + " " + vname);

        if (vname == m_vname || vname == "all")
        {
          Logger::info("OnNewMail Accepted Wailpoint");
          m_uav_model.setNextWaypointLatLon(XYPoint(lat, lon));

          m_tonext_waypointXY = XYPoint(x, y);
        }
      }
      else
      {
        m_warning_system_ptr->queue_monitorWarningForXseconds("Invalid waypoint string: " + wp_str, WARNING_DURATION);
      }
    }
    else if (key == "HELM_STATUS")
    {
      static bool helm_status_on = false;
      Logger::info("OnNewMail HELM_STATUS: " + msg.GetString());
      
      setBooleanOnString(helm_status_on, msg.GetString());

      if (!helm_status_on && m_autopilot_mode != AutopilotHelmMode::HELM_INACTIVE_LOITERING)
      {
        // m_autopilot_mode = AutopilotHelmState::HELM_INACTIVE;
        reportEvent("Helm is set to OFF");
        m_warning_system_ptr->queue_monitorWarningForXseconds("Helm is turned off. Will loiter at current position", WARNING_DURATION);
        m_do_loiter_pair = std::make_pair(true, "here");
        goToHelmMode(AutopilotHelmMode::HELM_INACTIVE);
      }
      else if (helm_status_on && !isHelmOn())
      {
        // m_autopilot_mode = AutopilotHelmState::HELM_ACTIVE;
        goToHelmMode(AutopilotHelmMode::HELM_ACTIVE);
      }
    }
    else if (key == "MOOS_MANUAL_OVERRIDE")
    {
      Logger::info("OnNewMail MOOS_MANUAL_OVERRIDE: " + msg.GetString() + " from " + msg.GetSource());
      if (msg.GetString() == "true")
      {
        m_warning_system_ptr->queue_monitorWarningForXseconds("Helm is parked. Will return to launch", WARNING_DURATION);
        m_do_return_to_launch = true;
        goToHelmMode(AutopilotHelmMode::HELM_PARKED, true);
      }
    }
    else if (key == "AUTOPILOT_MODE")
    {
      Logger::info("OnNewMail AUTOPILOT_MODE: " + msg.GetString() + " from " + msg.GetSource());
      goToHelmMode(stringToHelmMode(msg.GetString()), true);
    }
    else if (key == "CHANGE_SPEED")
    {
      Logger::info("OnNewMail CHANGE_SPEED: " + msg.GetString() + " from " + msg.GetSource());
      double speed_change = msg.GetDouble();
      m_do_change_speed_pair = std::make_pair(true, speed_change);
    }
    else if (key == "CHANGE_COURSE")
    {
      Logger::info("OnNewMail CHANGE_COURSE: " + msg.GetString() + " from " + msg.GetSource());
      double course_change = msg.GetDouble();
      m_do_change_course_pair = std::make_pair(true, course_change);
    }
    else if (key == "CHANGE_ALTITUDE")
    {
      Logger::info("OnNewMail CHANGE_ALTITUDE: " + msg.GetString() + " from " + msg.GetSource());
      double altitude_change = msg.GetDouble();
      m_do_change_altitude_pair = std::make_pair(true, altitude_change);

      std::string update_str = "altitude=" + doubleToString(altitude_change + m_uav_model.getTargetAltitudeAGL());
      Notify("CONST_ALTITUDE_UPDATE", update_str);
    }
    else if (key == "ARM_UAV")
    {
      setBooleanOnString(m_do_arm, msg.GetString());
    }
    else if (key == "DEAD_MAN_POST_INTERRUPT")
    {
      m_warning_system_ptr->queue_monitorWarningForXseconds(
          "No heartbeats from GCS. Returning to launch", WARNING_DURATION);
      reportEvent("No heartbeats from GCS. Returning to launch");
      m_do_return_to_launch = true;
    }
    else if (key == "ARDU_COMMAND")
    {
      std::string command = msg.GetString();
      Logger::info("OnNewMail ARDU_COMMAND: " + command + " from " + msg.GetSource());

      bool handled = false;
      if (command == "VIZ_HOME")
      {
        visualizeHomeLocation();
        handled = true;
      }
      else if (command == "FLY_WAYPOINT")
      {
        m_do_fly_to_waypoint = true;
        handled = true;
      }
      else if (command == "DO_TAKEOFF")
      {
        m_do_takeoff = true;
        handled = true;
      }
      else if (command == "RESET_SPEED_MIN")
      {
        m_do_reset_speed = true;
        handled = true;
      }
      else if (command == "RETURN_TO_LAUNCH" || command == "RETURN")
      {
        m_do_return_to_launch = true;
        handled = true;
      }
      else if (command == "LOITER")
      {
        std::string val = (msg.GetSource() == "pHelmIvP") ? "default" : "here";
        m_do_loiter_pair = std::make_pair(true, val);
        handled = true;
      }
      else if (command == "SURVEY")
      {
        m_do_helm_survey = true;
        handled = true;
      }
      else if (command == "DO_VORONOI")
      {
        m_do_helm_voronoi = true;
        handled = true;
      }
      else if (command == "AUTOLAND")
      {
        m_do_autoland = true;
        handled = true;
      }

      if (!handled)
      {
        Logger::warning("Unhandled ARDU Command: " + command);
        m_warning_system_ptr->queue_monitorWarningForXseconds("Unhandled ARDU Command: " + command, WARNING_DURATION);
      }
    }

    else if (key != "APPCAST_REQ")
    { // handled by AppCastingMOOSApp
      Logger::warning("Unhandled Mail: " + key);
      reportRunWarning("Unhandled Mail: " + key);
    }
  }

  return (true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool ArduBridge::OnConnectToServer()
{
  registerVariables();

  m_warning_system_ptr->queue_monitorCondition("Helm is set in Park Mode", [this]()
                                               { return (m_autopilot_mode == AutopilotHelmMode::HELM_PARKED); });

  Notify("AUTOPILOT_MODE", helmModeToString(m_autopilot_mode), m_curr_time);
  return (true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool ArduBridge::Iterate()
{
  // Logger::info("Iterate Start");

  AppCastingMOOSApp::Iterate();

  //////////////////////////////////////////////////////////////
  //////  Blocking Functions - These functions will block until they are done
  /////   - Takeoff
  /////   - change speed        (needs to be done immidiately)
  /////   - change course      (needs to be done immidiately)
  /////   - change altitude     (needs to be done immidiately)
  /////   - reset speed to min  (needs to be done immidiately)
  //////////////////////////////////////////////////////////////

  /// Takeoff
  if (m_do_takeoff)
  {
    tryDoTakeoff();
    m_do_takeoff = false;
  }

  /// - change speed
  if (m_do_change_speed_pair.first)
  {
    double new_speed = m_uav_model.getTargetAirSpeed() + m_do_change_speed_pair.second;
    //  m_uav_model.commandAndSetAirSpeed(new_speed);

    if (m_uav_model.commandAndSetAirSpeed(new_speed))
    {
      postSpeedUpdateToBehaviors(new_speed);
      reportEvent("Changed speed to " + doubleToString(new_speed));
    }

    if (m_command_groundSpeed)
    {
      m_uav_model.commandGroundSpeed(new_speed);
    }

    m_do_change_speed_pair.second = 0;
    m_do_change_speed_pair.first = false;
  }

  /// - change course
  if (m_do_change_course_pair.first)
  {
    double new_course = angle360(m_uav_model.getTargetCourse() + m_do_change_course_pair.second);

    if (isHelmCommanding())
    {
      m_warning_system_ptr->queue_monitorWarningForXseconds("Helm is commanding values. Restart Helm or wait for NothingToDo", WARNING_DURATION);
    }
    else if (m_uav_model.commandAndSetCourse(new_course, isHelmOn_NothingTodo()))
    {
      reportEvent("Changed course to " + doubleToString(new_course));
    };

    m_do_change_course_pair.first = false;
    m_do_change_course_pair.second = 0;
  }

  /// - change altitude
  if (m_do_change_altitude_pair.first)
  {
    double new_altitude = m_uav_model.getTargetAltitudeAGL() + m_do_change_altitude_pair.second;
    bool success = m_uav_model.commandAndSetAltitudeAGL(new_altitude);

    if (!isHelmOn() && !success)
    {
      if (m_autopilot_mode == AutopilotHelmMode::HELM_INACTIVE_LOITERING)
      {
        success = tryloiterAtPos(m_uav_model.getCurrentLoiterLatLon());
      }
      else if (m_autopilot_mode == AutopilotHelmMode::HELM_INACTIVE)
      {
        success = tryFlyToWaypoint();
      }

      if (!success)
      {
        m_warning_system_ptr->queue_monitorWarningForXseconds("Failed to immidiately change altitude (in helm off state)", WARNING_DURATION);
      }
    }

    reportEvent("Changed altitude to " + doubleToString(new_altitude));
    m_do_change_altitude_pair.second = 0;
    m_do_change_altitude_pair.first = false;
  }

  /// - reset speed to min
  if (m_do_reset_speed)
  {
    auto min_speed = m_uav_model.getMinAirSpeed();
    if (m_uav_model.commandAndSetAirSpeed(min_speed))
    {
      postSpeedUpdateToBehaviors(min_speed);
      reportEvent("Changed speed to " + doubleToString(min_speed));
    }

    if (m_command_groundSpeed)
    {
      m_uav_model.commandGroundSpeed(m_uav_model.getMinAirSpeed());
    }
    m_do_reset_speed = false;
  }

  //////////////////////////////////////////////////////////////
  //////  Send Desired Values to UAV   - Async
  //////////////////////////////////////////////////////////////
  static bool sendValEnabled = false;

  if (isHelmCommanding() && !sendValEnabled)
  {
    Logger::info("Iterate: Enabling sendDesiredValues");
    m_uav_model.enableSendDesiredValues();
    sendValEnabled = true;
  }
  else if (!isHelmCommanding() && sendValEnabled)
  {
    Logger::info("Iterate: Disabling sendDesiredValues");
    m_uav_model.enableSendDesiredValues(false);
    sendValEnabled = false;
  }

  //////////////////////////////////////////////////////////////
  //////  Arm UAV           - Async
  //////////////////////////////////////////////////////////////
  if (m_do_arm)
  {
    m_uav_model.sendArmCommandIfHealthyAndNotArmed_async();
    m_do_arm = false;
  }

  //////////////////////////////////////////////////////////////
  //////  Fly to waypoint   - Async w/polling
  //////////////////////////////////////////////////////////////
  if (m_do_fly_to_waypoint)
  {
    static bool running_ftw = false;
    if (!running_ftw)
    {
      flyToWaypoint_async();
      running_ftw = true;
    }

    auto result = future_poll_result(m_fly_to_waypoint_promfut.fut);
    if (result.has_value())
    {
      auto goToMode = AutopilotHelmMode::HELM_INACTIVE;
      if (result.value().success)
      {
        goToMode = isHelmOn() ? AutopilotHelmMode::HELM_TOWAYPT : AutopilotHelmMode::HELM_INACTIVE;

        auto loiter = m_uav_model.getNextWaypointLatLon();

        // visualize loiter loc if helm is not on drive
        auto visualize = !isHelmOn();
        visualizeLoiterLocation(loiter, visualize);
      }
      else
      {
        m_warning_system_ptr->queue_monitorWarningForXseconds("FAIL: " + result.value().message, result.value().display_time);
      }

      goToHelmMode(goToMode);

      running_ftw = false;
      m_do_fly_to_waypoint = false;
      m_fly_to_waypoint_promfut.reset();
    }
  }

  //////////////////////////////////////////////////////////////
  //////  Return to Launch  - Async w/polling
  //////////////////////////////////////////////////////////////
  if (m_do_return_to_launch)
  {
    Logger::info("Iterate: Returning to launch");
    static bool running_rtl = false;
    if (!running_rtl)
    {
      rtl_async();
      running_rtl = true;
    }

    auto result = future_poll_result(m_rtl_promfut.fut);
    if (result.has_value())
    {
      Logger::info("Iterate: Command sent with result: " + result.value().message + " success: " + boolToString(result.value().success));

      auto goToMode = AutopilotHelmMode::HELM_INACTIVE;
      if (result.value().success)
      {
        goToMode = isHelmOn() ? AutopilotHelmMode::HELM_RETURNING : AutopilotHelmMode::HELM_INACTIVE;
      }
      else
      {
        m_warning_system_ptr->queue_monitorWarningForXseconds("FAIL: " + result.value().message, WARNING_DURATION);
      }

      goToHelmMode(goToMode);

      running_rtl = false;
      m_do_return_to_launch = false;
      m_rtl_promfut.reset();
    }
  }

  //////////////////////////////////////////////////////////////
  //////  Autoland          - Async w/polling
  //////////////////////////////////////////////////////////////
  if (m_do_autoland)
  {
    Logger::info("Iterate: Autoland");
    static bool running_autoland = false;
    if (!running_autoland)
    {
      autoland_async();
      running_autoland = true;
    }

    auto result = future_poll_result(m_autoland_promfut.fut);
    if (result.has_value())
    {
      Logger::info("Iterate: Autoland command sent with result: " + result.value().message + " success: " + boolToString(result.value().success));

      auto goToMode = AutopilotHelmMode::HELM_INACTIVE;
      if (result.value().success)
      {
        goToMode = AutopilotHelmMode::HELM_INACTIVE;
      }
      else
      {
        m_warning_system_ptr->queue_monitorWarningForXseconds("FAIL: " + result.value().message, WARNING_DURATION);
      }

      goToHelmMode(goToMode);

      running_autoland = false;
      m_do_autoland = false;
      m_autoland_promfut.reset();
    }
  }

  //////////////////////////////////////////////////////////////
  //////  Loiter            - Async w/polling
  //////////////////////////////////////////////////////////////

  static int loiter_tries = 0;
  // pos is >100 awai from lotiter, send again
  if (m_autopilot_mode == AutopilotHelmMode::HELM_INACTIVE_LOITERING && !m_do_loiter_pair.first)
  {
    auto pos = transformLatLonToXY({m_uav_model.getLatitude(), m_uav_model.getLongitude()});
    auto loiterCoord = transformLatLonToXY(m_uav_model.getCurrentLoiterLatLon());
    auto dist = hypot(pos.get_vx() - loiterCoord.get_vx(), pos.get_vy() - loiterCoord.get_vy());
    Logger::info("Iterate: UAV is loitering. Checking if UAV is far from loiter location / dist: " + doubleToString(dist));
    if (dist > 100 && loiter_tries < 3)
    {
      Logger::info("Iterate: UAV is far from loiter location. Sending loiter command again");
      loiter_tries++;
      m_uav_model.pushCommand([this](UAV_Model &uav)
                              { uav.commandLoiterAtPos(uav.getCurrentLoiterLatLon()); });
    }
  }

  if (m_do_loiter_pair.first)
  {
    static bool running_loiter = false;
    if (!running_loiter)
    {
      // Will report warning if command fails
      auto curr_loiter = XYPoint(m_uav_model.getLatitude(), m_uav_model.getLongitude());
      auto location = (m_do_loiter_pair.second == "here") ? curr_loiter : XYPoint(0, 0);
      loiterAtPos_async(location);
      running_loiter = true;
    }

    auto result = future_poll_result(m_loiter_at_pos_promfut.fut);
    if (result.has_value())
    {
      if (result.value().success)
      {
        goToHelmMode(AutopilotHelmMode::HELM_INACTIVE_LOITERING);
        visualizeLoiterLocation(m_uav_model.getCurrentLoiterLatLon());
        loiter_tries = 0;
      }
      else
      {
        goToHelmMode(AutopilotHelmMode::HELM_INACTIVE);
        m_warning_system_ptr->queue_monitorWarningForXseconds("FAIL: " + result.value().message, WARNING_DURATION);
      }
      running_loiter = false;
      m_do_loiter_pair.first = false;
      m_do_loiter_pair.second = "default";
      m_loiter_at_pos_promfut.reset();
    }
  }

  //////////////////////////////////////////////////////////////
  //////  Do Helm Survey
  //////////////////////////////////////////////////////////////
  if (m_do_helm_survey)
  {

    // Handled by pGridSearchPlanner
    // Notify("SURVEY_UPDATE", generateMissionPathSpec(m_waypointsXY_mission));

    bool sendCommand=true;

    if (!isHelmOn()){
      m_warning_system_ptr->queue_monitorWarningForXseconds("Helm is not active, Cannot do survey", WARNING_DURATION);
      sendCommand=false;
    }
    else if(!m_uav_model.commandGuidedMode())
    {
      m_warning_system_ptr->queue_monitorWarningForXseconds("Failed to enter guided mode", WARNING_DURATION);
      sendCommand=false;
    }
   
    if (sendCommand)
    {
      m_uav_model.commandGuidedMode();
      goToHelmMode(AutopilotHelmMode::HELM_SURVEYING);
    }



    m_do_helm_survey = false;
  }

  //////////////////////////////////////////////////////////////
  //////  Do Helm Voronoi mission
  //////////////////////////////////////////////////////////////

  if (m_do_helm_voronoi)
  {
   
    bool sendCommand=true;
    if (!isHelmOn())
    {
      m_warning_system_ptr->queue_monitorWarningForXseconds("Helm is not active, Cannot do Voronoi", WARNING_DURATION);
      sendCommand=false;
    }
    else if (!m_uav_model.commandGuidedMode())
    {
      m_warning_system_ptr->queue_monitorWarningForXseconds("Failed to enter guided mode", WARNING_DURATION);
      sendCommand=false;
    }   
    if (sendCommand)
    {
      goToHelmMode(AutopilotHelmMode::HELM_VORONOI);
    }
    m_do_helm_voronoi = false;
  }

  postTelemetryUpdate(m_uav_prefix);

  m_warning_system_ptr->checkConditions(); // Check for warnings and remove/raise them as needed

  AppCastingMOOSApp::PostReport();

  // Logger::info("Iterate Complete \n\n");
  return (true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool ArduBridge::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  std::stringstream ss;

  ss << "Starting up ArduBridge with mission file name: " << GetMissionFileName() << std::endl;
  ss << "App name is: " << GetAppName() << std::endl;

  std::cout << ss.str() << std::endl;
  Logger::info(ss.str());

  std::string ardupilot_url;
  std::pair<bool, std::string> url_protocol_pair{false, ""};

  STRING_LIST sParams;

  m_MissionReader.EnableVerbatimQuoting(false);
  if (!m_MissionReader.GetConfiguration(GetAppName(), sParams))
  {
    std::cout << "No config block found for " << GetAppName() << std::endl;

    Logger::error("No config block found for " + GetAppName());
    reportConfigWarning("No config block found for " + GetAppName());
  }

  STRING_LIST::iterator p;
  for (p = sParams.begin(); p != sParams.end(); p++)
  {
    std::string orig = *p;
    std::string line = *p;
    std::string param = tolower(biteStringX(line, '='));
    std::string value = line;

    bool handled = false;
    if (param == "vname")
    {
      m_vname = value;
      MOOSToLower(m_vname);
      handled = true;
    }
    else if (param == "vcolor")
    {
      m_vcolor = value;
      handled = true;
    }
    else if (param == "logger")
    {
      Logger::enable(value == "true");
      handled = true;
    }
    else if (param == "is_sim" && isBoolean(value))
    {
      handled = setBooleanOnString(m_is_simulation, value);
    }
    else if ((param == "command_groundspeed" || param == "cmd_gs") && isBoolean(value))
    {
      handled = setBooleanOnString(m_command_groundSpeed, value);
    }
    else if (param == "ardupiloturl" || param == "url")
    {
      ardupilot_url = value;
      handled = true;
    }
    else if (param == "prefix")
    {
      handled = setNonWhiteVarOnString(m_uav_prefix, value);
    }
    else if (param == "url_protocol")
    {
      if (value == "tcp")
      {
        url_protocol_pair.first = true;
        url_protocol_pair.second = "tcp://";
      }
      else if (value == "udp")
      {
        url_protocol_pair.first = true;
        url_protocol_pair.second = "udp://";
      }
      else if (value == "serial")
      {
        url_protocol_pair.first = true;
        url_protocol_pair.second = "serial:///dev/";
      }

      handled = url_protocol_pair.first;
      std::cout << "URL protocol set to: " << url_protocol_pair.second << std::endl;
      Logger::info("URL protocol set to: " + url_protocol_pair.second);
    }
    if (!handled)
      reportUnhandledConfigWarning(orig);
  }

  // look for latitude, longitude global variables
  double latOrigin, longOrigin;
  if (!m_MissionReader.GetValue("LatOrigin", latOrigin))
  {
    MOOSTrace("pArduBridge: LatOrigin not set in " + GetMissionFileName() + " file.\n");

    m_geo_ok = false;
  }
  else if (!m_MissionReader.GetValue("LongOrigin", longOrigin))
  {
    MOOSTrace("pArduBridge: LongOrigin not set in " + GetMissionFileName() + " file\n");
    m_geo_ok = false;
  }
  else
  {
    m_geo_ok = true;
    // initialize m_geodesy
    if (!m_geodesy.Initialise(latOrigin, longOrigin))
    {
      MOOSTrace("pArduBridge: Geodesy init failed.\n");
      m_geo_ok = false;
    }
  }

  ardupilot_url = url_protocol_pair.second + ardupilot_url;

  std::cout << "ArduPilot URL is: " << ardupilot_url << std::endl;

  if (!m_cli_arg.parse(ardupilot_url))
  {
    if (!url_protocol_pair.first)
    {
      reportConfigWarning("URL protocol not set - Need to restart with a valid URL prefix");
      std::cout << "URL protocol not set - Need to restart with a valid URL prefix" << std::endl;
      Logger::error("URL protocol not set - Need to restart with a valid URL prefix");
    }
    else
    {
      reportConfigWarning("Invalid ArduPilot URL specified - Need to restart with a valid URL");
      std::cout << "Invalid ArduPilot URL specified - Need to restart with a valid URL" << std::endl;
      Logger::error("Invalid ArduPilot URL specified - Need to restart with a valid URL");
    }
  }
  else
  {
    // Connect to autopilot
    if (!m_uav_model.connectToUAV(ardupilot_url))
    {
      std::cout << "Failed to connect to ArduPilot" << std::endl;
      return (false);
    }
  }

  if (!m_uav_model.setUpMission(!m_is_simulation))
  {
    std::cout << "Mission setup failed" << std::endl;
    return (false);
  };

  if (m_vname.empty())
  {
    std::cout << "Vehicle name not set. " << std::endl;
    return (false);
  }

  m_uav_model.startCommandSender();

  m_uav_model.registerSendDesiredValuesFunction([this](UAV_Model &uav, bool forceSend)
                                                {
                                                  static unsigned int failed_attempts = 0;
                                                  if (!m_helm_desiredValues->isValid() && !forceSend)
                                                  {
                                                    if (failed_attempts++ < 3 && !forceSend)
                                                      return
                                                    
                                                    m_warning_system_ptr->queue_monitorWarningForXseconds("No valid setpoints to send", 2);
                                                    return;
                                                  }
                                                  
                                                  failed_attempts = 0;
                                                  this->sendDesiredValuesToUAV(uav, forceSend); });
  Logger::info("Registered function for sending desired variables");

  m_warning_system_ptr->checkConditions(); // Check for warnings and remove/raise them as needed

  postSpeedUpdateToBehaviors(m_uav_model.getTargetAirSpeed());

  visualizeHomeLocation();

  registerVariables();
  return (true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void ArduBridge::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("FLY_WAYPOINT", 0);
  Register("DO_TAKEOFF", 0);
  Register("RETURN_TO_LAUNCH", 0);
  Register("AUTOLAND", 0);
  Register("LOITER", 0);
  Register("SURVEY", 0);
  Register("RESET_SPEED_MIN", 0);
  // For pMarineViewer
  Register("VIZ_HOME", 0);

  Register("ARDU_COMMAND", 0);

  Register("ARM_UAV", 0);     // on,off
  Register("HELM_STATUS", 0); // on,off

  Register("CHANGE_SPEED", 0);
  Register("CHANGE_COURSE", 0);
  Register("CHANGE_ALTITUDE", 0);

  // To be sent to Ardupilot
  Register("DESIRED_HEADING", 0); // DESIRED_COURSE comes in as DESIRED_HEADING from pHelm for legacy reasons
  Register("DESIRED_SPEED", 0);
  Register("DESIRED_ALTITUDE", 0);

  // Other variables
  Register("NEXT_WAYPOINT", 0);
  Register("AUTOPILOT_MODE", 0);

  // Register("MOOS_MANUAL_OVERIDE", 0);
  Register("MOOS_MANUAL_OVERRIDE", 0);
  Register("DEAD_MAN_POST_INTERRUPT", 0);
}

//------------------------------------------------------------
// Procedure: buildReport()

bool ArduBridge::buildReport()
{
  const int sdigits = 2;
  m_msgs << "============================================" << std::endl;
  m_msgs << "File: pArduBridge                           " << std::endl;
  m_msgs << "============================================" << std::endl;

  m_msgs << " -------- Configuration Settings -----------" << std::endl;
  m_msgs << "ArduPilot URL: " << m_cli_arg.get_path() << std::endl;
  m_msgs << "ArduPilot Port: " << m_cli_arg.get_port() << std::endl;
  m_msgs << "ArduPilot Protocol: " << protocol2str.at(m_cli_arg.get_protocol()) << std::endl;
  std::string sim_mode = m_is_simulation ? "SITL" : "No Simulation";
  m_msgs << "Simulation Mode: " << sim_mode << std::endl;

  m_msgs << "-------------------------------------------" << std::endl;

  auto uav_isArmed = m_uav_model.isArmed();
  auto uav_isHealthy = m_uav_model.isHealthy();
  auto uav_isInAir = m_uav_model.isInAir();
  auto uav_flightMode = m_uav_model.getFlightMode();

  auto uav_minAirSpeed = m_uav_model.getMinAirSpeed();
  auto uav_maxAirSpeed = m_uav_model.getMaxAirSpeed();

  m_msgs << "UAV States: " << std::endl;
  m_msgs << "------------------ " << std::endl;
  m_msgs << "           Is Armed: " << boolToString(uav_isArmed) << std::endl;
  m_msgs << "         Is Healthy: " << boolToString(uav_isHealthy) << std::endl;
  m_msgs << "             In Air: " << boolToString(uav_isInAir) << std::endl;
  m_msgs << "        Flight Mode: " << uav_flightMode << std::endl;

  auto guidedHold = m_uav_model.isHoldCourseGuidedSet();
  m_msgs << "    Helm On BUSY: " << boolToString(isHelmCommanding()) << std::endl;
  m_msgs << "   Helm GUIDED HOLD: " << boolToString(guidedHold) << std::endl;
  // m_msgs << "do_loiter_pair.second: " << m_do_loiter_pair.second << std::endl;
  m_msgs << std::endl;

  m_msgs << "UAV Parameters: " << std::endl;
  m_msgs << "------------------ " << std::endl;
  m_msgs << "       Min AirSpeed: " << doubleToStringX(uav_minAirSpeed, sdigits) << " m/s" << std::endl;
  m_msgs << "       Max AirSpeed: " << doubleToStringX(uav_maxAirSpeed, sdigits) << " m/s" << std::endl;

  double lat = m_uav_model.getLatitude();
  double lon = m_uav_model.getLongitude();

  auto uav_altitude_msl = m_uav_model.getAltitudeMSL();
  auto uav_targetCOG = m_uav_model.getTargetCourse();
  auto uav_targetAirspeed = m_uav_model.getTargetAirSpeed();
  auto uav_targetAltitudeAGL = m_uav_model.getTargetAltitudeAGL();

  auto uav_SOG = m_uav_model.getSOG();
  auto uav_heading = m_uav_model.getHeading();
  auto uav_COG = m_uav_model.getCOG();
  auto uav_altitude_agl = m_uav_model.getAltitudeAGL();

  auto uav_home_latlon = m_uav_model.getHomeLatLon();
  auto uav_next_waypoint_latlon = m_uav_model.getNextWaypointLatLon();
  auto uav_course_waypoint_latlon = m_uav_model.getCourseWaypointLatLon();

  auto XY = transformLatLonToXY({lat, lon});

  m_msgs << "State Information: " << std::endl;
  m_msgs << "------------------ " << std::endl;
  m_msgs << "       Helm Autonomy Mode: " << helmModeToString(m_autopilot_mode) << std::endl;
  m_msgs << "  (Latitude , Longditute): " << lat << " , " << lon << std::endl;
  m_msgs << "                  (X , Y): " << XY.x() << " , " << XY.y() << std::endl;
  // m_msgs << "    Target Altitude (AGL): " << m_uav_model.getTargetAltitudeAGL() << " m" << "    Last Sent (AGL): " << m_uav_model.getLastSentTargetAltitudeAGL() << " m" << std::endl;
  // m_msgs << "           Altitude (AGL): " << m_uav_model.getAltitudeAGL() << " m (Depth/Z: " << -m_uav_model.getAltitudeAGL() << " m)" << std::endl;
  m_msgs << "           Altitude (MSL): " << doubleToStringX(uav_altitude_msl, sdigits) << " m" << std::endl;
  // m_msgs << "          Target Airspeed: " << m_uav_model.getTargetAirSpeed() << " m/s" << std::endl;
  // m_msgs << "          AirSpeed (SOG) : " << m_uav_model.getSOG() << " m/s)" << std::endl;
  // m_msgs << "     Target Course (COG): " << doubleToStringX(uav_targetCOG, sdigits) << " deg" << std::endl;
  m_msgs << "                  Heading: " << uav_heading << " deg" << std::endl;

  m_msgs << "-------------------------------------------" << std::endl;

  ACTable actb(4);
  actb << "States | Measurments | Helm | Targets";
  actb.addHeaderLines();
  actb << "Speed:" << doubleToStringX(uav_SOG, sdigits) << doubleToStringX(m_helm_desiredValues->readDesiredSpeed(), sdigits) << doubleToStringX(uav_targetAirspeed, sdigits);
  actb << "COG:" << doubleToStringX(uav_COG, sdigits) << doubleToStringX(m_helm_desiredValues->readDesiredCourse(), sdigits) << doubleToStringX(uav_targetCOG, sdigits);
  actb << "Altitude:" << doubleToStringX(uav_altitude_agl, sdigits) << doubleToStringX(m_helm_desiredValues->readDesiredAltitudeAGL(), sdigits) << doubleToStringX(uav_targetAltitudeAGL, sdigits);
  ;

  m_msgs << actb.getFormattedString() << std::endl;
  m_msgs << "-------------------------------------------" << std::endl;

  ACTable actabd(3);
  actabd << "Waypoint | Lat | Lon";
  actabd.addHeaderLines();
  actabd << "Home Coord:" << uav_home_latlon.x() << uav_home_latlon.y();
  actabd << "Next Wypt Coord:" << uav_next_waypoint_latlon.x() << uav_next_waypoint_latlon.y();
  actabd << "Course Wypt Coord:" << uav_course_waypoint_latlon.x() << uav_course_waypoint_latlon.y();
  m_msgs << actabd.getFormattedString() << std::endl;
  m_msgs << "-------------------------------------------" << std::endl;

  ACTable actab(2);
  actab << "Debug | Value ";
  actab.addHeaderLines();
  actab << "Do set fly waypoint:" << boolToString(m_do_fly_to_waypoint);
  actab << "Do takeoff:" << boolToString(m_do_takeoff);
  actab << "command groundSpeed:" << boolToString(m_command_groundSpeed);
  m_msgs << actab.getFormattedString();

  return (true);
}

//---------------------------------------------------------
// Procedure: sendSetPointsToUAV()

void ArduBridge::sendDesiredValuesToUAV(UAV_Model &uav, bool forceSend)
{

  auto desired_course = m_helm_desiredValues->getDesiredCourse();
  auto desired_speed = m_helm_desiredValues->getDesiredSpeed();
  auto desired_altitude_agl = m_helm_desiredValues->getDesiredAltitude();

  if (forceSend)
  {
    desired_course = m_helm_desiredValues->readDesiredCourse();
    desired_speed = m_helm_desiredValues->readDesiredSpeed();
    desired_altitude_agl = m_helm_desiredValues->readDesiredAltitudeAGL();
  }

  if (desired_course.has_value())
  {
    Logger::info("Sending desired course: " + doubleToString(desired_course.value()));
    uav.commandAndSetCourse(desired_course.value());
  }

  if (desired_speed.has_value())
  {
    Logger::info("Sending desired speed: " + doubleToString(desired_speed.value()));
    uav.commandAndSetAirSpeed(desired_speed.value());

    if (m_command_groundSpeed)
    {
      uav.commandGroundSpeed(desired_speed.value());
    }
  }

  if (desired_altitude_agl.has_value())
  {
    Logger::info("Sending desired altitude: " + doubleToString(desired_altitude_agl.value()));
    uav.commandAndSetAltitudeAGL(desired_altitude_agl.value());
  }
}

std::string ArduBridge::generateMissionPathSpec(const std::vector<XYPoint> &points) const
{
  XYSegList seglist;
  for (auto point : points)
  {
    seglist.add_vertex(point);
  }
  std::string update_str = "points = ";
  update_str += seglist.get_spec();
  return update_str;
}

void ArduBridge::postTelemetryUpdate(const std::string &prefix)
{

  static std::function<bool(const std::string &, double, double)> NotifyIfNonNan = [this](const std::string &key, double value, double time = (-1.0))
  {
    if (!std::isnan(value))
    {
      return Notify(key, value, time);
    }
    return false;
  };

  double lat = m_uav_model.getLatitude();
  double lon = m_uav_model.getLongitude();

  if (!lat || !lon)
  {
    m_warning_system_ptr->queue_monitorWarningForXseconds("NAN Values at lat or long", 5);
    return;
  }

  NotifyIfNonNan(prefix + "_LAT", lat, m_curr_time);
  NotifyIfNonNan(prefix + "_LON", lon, m_curr_time);

  auto uav_isHoldCourseGuided = m_uav_model.isHoldCourseGuidedSet();

  auto uav_SOG = m_uav_model.getSOG();
  // auto uav_heading = m_uav_model.getHeading();
  auto uav_COG = m_uav_model.getCOG();
  auto uav_altitude_agl = m_uav_model.getAltitudeAGL();

  auto XY = transformLatLonToXY({lat, lon});

  NotifyIfNonNan(prefix + "_X", XY.x(), m_curr_time);
  NotifyIfNonNan(prefix + "_Y", XY.y(), m_curr_time);

  visualizeCourseHoldTarget(isHelmCommanding() || uav_isHoldCourseGuided);

  NotifyIfNonNan(prefix + "_SPEED", uav_SOG, m_curr_time);

  NotifyIfNonNan(prefix + "_ALTITUDE", uav_altitude_agl, m_curr_time);
  NotifyIfNonNan(prefix + "_DEPTH", -uav_altitude_agl, m_curr_time);

  NotifyIfNonNan(prefix + "_HEADING", uav_COG, m_curr_time); // NAV_COURSE comes in as NAV_HEADING for pNodeReporter
}

void ArduBridge::postSpeedUpdateToBehaviors(double speed)
{

  std::string update_str = "speed=" + doubleToString(speed);
  Notify("SURVEY_UPDATE", update_str);
  Notify("TOWAYPT_UPDATE", update_str);
  Notify("RETURN_UPDATE", update_str);
}

void ArduBridge::visualizeHomeLocation()
{
  auto home_latlon = m_uav_model.getHomeLatLon();
  double lat = home_latlon.x();
  double lon = home_latlon.y();

  if (!lat || !lon)
  {
    m_warning_system_ptr->queue_monitorWarningForXseconds("Cannot Visualize Home: NAN Values at lat or long", 5);
    return;
  }

  auto XY = transformLatLonToXY({lat, lon});

  XYMarker marker(XY.x(), XY.y());
  marker.set_label("Home_" + m_vname);
  marker.set_type("gateway");
  marker.set_width(MARKER_WIDTH);
  std::string spec = marker.get_spec() + ",color=" + m_vcolor + ",scale=" + doubleToString(MARKER_WIDTH);
  Notify("VIEW_MARKER", spec);

  reportEvent("Set marker at home location: " + spec);
}

void ArduBridge::visualizeLoiterLocation(const XYPoint &loiter_coord, bool visualize)
{

  XYPoint point = transformLatLonToXY(loiter_coord);
  if (point == XYPoint(0, 0) && visualize)
  {
    m_warning_system_ptr->queue_monitorWarningForXseconds("Cannot Visualize Loiter: NAN Values at lat or long", 5);
    return;
  }
  XYMarker marker(point.x(), point.y());

  marker.set_label("Loiter_point_" + m_vname);
  marker.set_type("gateway");
  marker.set_width(MARKER_WIDTH);
  marker.set_active(visualize);
  std::string spec = marker.get_spec(); //+ ",scale=" + doubleToString(MARKER_WIDTH);
  Notify("VIEW_MARKER", spec);

  reportEvent("Set marker at loiter location: " + spec);
}

void ArduBridge::visualizeCourseWaypoint(const XYPoint &course_coord, bool visualize)
{

  XYPoint point = transformLatLonToXY(course_coord);
  if (point == XYPoint(0, 0) && visualize)
  {
    m_warning_system_ptr->queue_monitorWarningForXseconds("No Course waypoiny set: NAN Values at lat or long", 5);
    return;
  }

  point.set_label("Hold Course point");
  point.set_vertex_size(COURSE_POINT_SIZE);
  point.set_active(visualize);
  std::string spec = point.get_spec() + ",color=" + m_vcolor;
  Notify("VIEW_POINT", spec);
}

void ArduBridge::visualizeCourseVector(double x, double y, double magnitude, double angle, bool visualize)
{

  XYVector vector(x, y, magnitude * 2, angle);
  vector.set_active(visualize);

  vector.set_label("_");
  // vector.set_edge_size(4);

  std::string spec = vector.get_spec() + ",color=" + m_vcolor;
  Notify("VIEW_VECTOR", spec);

  // reportEvent("Set vector at wbldith: " + spec);
}

void ArduBridge::visualizeCourseHoldTarget(bool visualize)
{
  static bool is_visible = false;

  if (!is_visible && !visualize)
    return; // Already removed

  is_visible = visualize;

  double lat = m_uav_model.getLatitude();
  double lon = m_uav_model.getLongitude();

  auto uav_targetAirspeed = m_uav_model.getTargetAirSpeed();
  auto uav_targetCourse = m_uav_model.getTargetCourse();

  if ((!lat || !lon) && !visualize)
    return; // If invald coord, return unless the goal is to remove from visualization

  auto XY = transformLatLonToXY({lat, lon});

  visualizeCourseVector(XY.x(), XY.y(), uav_targetAirspeed, uav_targetCourse, visualize);
}

bool ArduBridge::parseCoordinateString(const std::string &input, double &lat, double &lon, double &x, double &y, std::string &vname) const
{
  std::vector<std::string> key_value_pairs = parseString(input, ',');

  for (const std::string &pair : key_value_pairs)
  {
    std::string key = biteStringX(const_cast<std::string &>(pair), '=');
    std::string value = pair; // `pair` now only contains the value after `biteStringX`

    if (key == "lat")
    {
      lat = std::stod(value);
    }
    else if (key == "lon")
    {
      lon = std::stod(value);
    }
    else if (key == "x")
    {
      x = std::stod(value);
    }
    else if (key == "y")
    {
      y = std::stod(value);
    }
    else if (key == "vname")
    {
      vname = value;
      MOOSToLower(vname);
    }
    else
    {
      Logger::error("parseCoordinateString: Unknown key: " + key);
      return false; // If there's an unknown key, return false
    }
  }

  return true;
}

XYPoint ArduBridge::transformLatLonToXY(const XYPoint &lat_lon)
{
  double lat = lat_lon.x();
  double lon = lat_lon.y();
  double nav_x, nav_y;
  if (m_geo_ok)
  {
    m_geodesy.LatLong2LocalGrid(lat, lon, nav_y, nav_x);
  }
  else
  {
    m_warning_system_ptr->queue_monitorWarningForXseconds("Geodesy not initialized", 5);
    return XYPoint(0, 0);
  }

  return XYPoint(nav_x, nav_y);
}

// Helper funcitons | only used in simulation
bool ArduBridge::tryDoTakeoff()
{
  if (isHelmOn())
  {
    m_warning_system_ptr->queue_monitorWarningForXseconds("HELM is active when trying to give control to UAV Ardupilot Start mission", WARNING_DURATION);
  }
  else
  {
    // send the takeoff command
    return m_uav_model.startMission();
  }
  return false;
}

bool ArduBridge::tryFlyToWaypoint()
{
  // send the fly to waypoint command
  XYPoint wp = m_uav_model.getNextWaypointLatLon();
  if (wp == XYPoint(0, 0))
  {
    m_warning_system_ptr->queue_monitorWarningForXseconds("No waypoint set", WARNING_DURATION);
    return false;
  }

  if (isHelmOn())
  {

    std::string update_str = "points=" + xypointToString(m_tonext_waypointXY);
    Notify("TOWAYPT_UPDATE", update_str);

    // m_warning_system_ptr->monitorWarningForXseconds("HELM is active when trying to give control to UAV Ardupilot", WARNING_DURATION);
  }
  else
  { // Helm is inactive. Send the waypoint directly to the UAV

    if (!m_uav_model.commandGoToLocationXY(wp))
    {
      return false;
    }
    // m_uav_model.setNextWaypointLatLon(XYPoint(0, 0)); // clear the waypoint
    visualizeLoiterLocation(wp);
  }

  return true;
}

void ArduBridge::flyToWaypoint_async()
{
  XYPoint wp = m_uav_model.getNextWaypointLatLon();
  if (wp == XYPoint(0, 0))
  {
    // m_warning_system_ptr->queue_monitorWarningForXseconds("No waypoint set", WARNING_DURATION);
    m_fly_to_waypoint_promfut.prom.set_value(ResultPair{false, "No waypoint set"});
    return;
  }

  if (isHelmOn())
  {

    if (!m_uav_model.isGuidedMode())
    {
      m_uav_model.pushCommand([this](UAV_Model &uav)
                              {
        
        m_warning_system_ptr->queue_monitorWarningForXseconds("Commanding Flight Mode Guided to UAV...", 3);
        uav.commandGuidedMode(); });
    }

    std::string update_str = "points=" + xypointToString(m_tonext_waypointXY);
    Notify("TOWAYPT_UPDATE", update_str);

    m_uav_model.setLoiterLocationLatLon(wp); // set the waypoint
    m_fly_to_waypoint_promfut.prom.set_value(ResultPair{true, ""});
    return;
  }

  // Helm is Park from here:

  m_uav_model.pushCommand([wp, this](UAV_Model &uav)
                          {

      // Helm is inactive. Send the waypoint directly to the UAV
      bool success = uav.commandGoToLocationXY(wp);
      if(success){
        uav.setLoiterLocationLatLon(wp); // clear the waypoint
        Logger::info("UAV_Model THREAD: Successfully sent waypoint to UAV");  
      }
      
      m_fly_to_waypoint_promfut.prom.set_value(
          ResultPair{success, success ? "" : "Failed sending command GotoWYP"}
      );

      return; });
}

bool ArduBridge::tryRTL()
{

  if (isHelmOn())
  {
    XYPoint home = transformLatLonToXY(m_uav_model.getHomeLatLon());
    if (home == XYPoint(0, 0))
    {
      m_warning_system_ptr->queue_monitorWarningForXseconds("Cannot Return to launch: NAN Values at lat or long", 5);
      return false;
    }

    std::string update_str = "points=" + xypointToString(home);
    Notify("RETURN_UPDATE", update_str);
    // m_warning_system_ptr->monitorWarningForXseconds("HELM is active when trying to give control to UAV Ardupilot", WARNING_DURATION);
  }
  else
  {
    return m_uav_model.commandReturnToLaunchAsync();
  }

  return true;
}

void ArduBridge::rtl_async()
{

  if (isHelmOn())
  {
    XYPoint home = transformLatLonToXY(m_uav_model.getHomeLatLon());
    if (home == XYPoint(0, 0))
    {
      m_rtl_promfut.prom.set_value(ResultPair{false, "Cannot Return to launch: NAN Values at lat or long", 5});
      return;
    }

    std::string update_str = "points=" + xypointToString(home);
    Notify("RETURN_UPDATE", update_str);

    m_rtl_promfut.prom.set_value(ResultPair{true, ""});
    return;
  }

  m_uav_model.pushCommand([this](UAV_Model &uav)
                          {

      // Helm is inactive. Send the waypoint directly to the UAV
      uav.commandReturnToLaunchAsync();

      m_rtl_promfut.prom.set_value(ResultPair{true, ""}); });

  return;
}

void ArduBridge::autoland_async()
{
  m_uav_model.pushCommand([this](UAV_Model &uav)
                          {
      // Send autoland command directly to the UAV
      bool success = uav.commandAutoland();
      
      m_autoland_promfut.prom.set_value(
          ResultPair{success, success ? "" : "Failed sending AUTOLAND command"}
      );
      
      return; });
}

bool ArduBridge::tryloiterAtPos(const XYPoint &loiter_coord, bool holdCurrentAltitude)
{
  // if(m_autopilot_mode != AutopilotHelmState::HELM_INACTIVE){

  XYPoint loiter_latlon = loiter_coord;
  if (loiter_latlon == XYPoint(0, 0))
  {
    loiter_latlon = {m_uav_model.getLatitude(), m_uav_model.getLongitude()}; // current position
  }

  if (m_autopilot_mode == AutopilotHelmMode::HELM_TOWAYPT)
  { // if UAV is flying to waypoint, loiter at the waypoint
    loiter_latlon = m_uav_model.getCurrentLoiterLatLon();
  }

  if (m_autopilot_mode == AutopilotHelmMode::HELM_RETURNING)
  {
    loiter_latlon = m_uav_model.getHomeLatLon();
  }

  if (!m_uav_model.commandLoiterAtPos(loiter_latlon, holdCurrentAltitude))
  {
    return false;
  }

  if (loiter_latlon == m_uav_model.getNextWaypointLatLon())
  {
    m_uav_model.setNextWaypointLatLon(XYPoint(0, 0));
  }

  visualizeLoiterLocation(loiter_latlon);
  return true;
}

void ArduBridge::loiterAtPos_async(const XYPoint &loiter_coord, bool holdCurrentAltitude)
{

  m_uav_model.pushCommand([loiter_coord, holdCurrentAltitude, ap_mode = m_autopilot_mode, this](UAV_Model &uav)
                          {

      XYPoint loiter_latlon = loiter_coord;

      if (loiter_latlon == XYPoint(0, 0)){
      
        if (ap_mode == AutopilotHelmMode::HELM_TOWAYPT){ 
          // if UAV is flying to waypoint, loiter at the waypoint
          loiter_latlon = m_uav_model.getCurrentLoiterLatLon();
        }
        else if (ap_mode == AutopilotHelmMode::HELM_RETURNING)
        {
          loiter_latlon = m_uav_model.getHomeLatLon();
        }
      }

      if (!uav.commandLoiterAtPos(loiter_latlon, holdCurrentAltitude))
      {
        m_loiter_at_pos_promfut.prom.set_value(ResultPair{false, "Failed sending command"});
        return;
      }

      if (loiter_latlon == uav.getNextWaypointLatLon())
      {
        uav.setNextWaypointLatLon(XYPoint(0, 0));
      }

      m_loiter_at_pos_promfut.prom.set_value(ResultPair{true, ""}); });
}

void ArduBridge::goToHelmMode(AutopilotHelmMode to_state, bool fromGCS)
{

  if (m_autopilot_mode == to_state)
  {
    return;
  }

  Logger::info("Changing Helm mode to: " + helmModeToString(to_state));

  Notify("AUTOPILOT_MODE", helmModeToString(to_state));
  auto from_state = m_autopilot_mode;
  m_autopilot_mode = to_state;

  auto transition = std::make_pair(from_state, to_state);
  if (m_statetransition_functions.find(transition) != m_statetransition_functions.end())
  {
    m_statetransition_functions[transition]();
  }

  switch (to_state)
  {
  case AutopilotHelmMode::HELM_PARKED:
    Notify("MOOS_MANUAL_OVERRIDE", "true");
    visualizeCourseHoldTarget(false);
    break;

  case AutopilotHelmMode::HELM_INACTIVE_LOITERING:
  case AutopilotHelmMode::HELM_INACTIVE:
    visualizeCourseHoldTarget(false);
    break;

  case AutopilotHelmMode::HELM_TOWAYPT:
  {
    Notify("MOOS_MANUAL_OVERRIDE", "false");
    std::string update_str = "points=" + xypointToString(m_tonext_waypointXY);
    Notify("TOWAYPT_UPDATE", update_str);
  }
  break;
  case AutopilotHelmMode::HELM_ACTIVE:
  case AutopilotHelmMode::HELM_RETURNING:
  case AutopilotHelmMode::HELM_SURVEYING:
  case AutopilotHelmMode::HELM_VORONOI:
    Notify("MOOS_MANUAL_OVERRIDE", "false");

  default:
    break;
  }
}

void ArduBridge::initializeStateTransitionFunctions()
{

  m_statetransition_functions[std::make_pair(AutopilotHelmMode::HELM_TOWAYPT, AutopilotHelmMode::HELM_INACTIVE_LOITERING)] = [this]()
  {
    visualizeCourseWaypoint(XYPoint(0, 0), false);
  };
}

std::string ArduBridge::xypointToString(const XYPoint &point) const
{
  return doubleToString(point.x()) + "," + doubleToString(point.y());
}