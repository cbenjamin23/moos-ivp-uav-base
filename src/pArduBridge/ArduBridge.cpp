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


//---------------------------------------------------------
// Constructor()

ArduBridge::ArduBridge()
    : m_do_fly_to_waypoint{false},
      m_do_takeoff{false},
      m_cli_arg{},
      m_do_change_speed_pair{std::make_pair(false, 0)},
      m_do_change_heading_pair{std::make_pair(false, 0)},
      m_do_change_altitude_pair{std::make_pair(false, 0)},
      m_do_reset_speed{false},
      m_do_return_to_launch{false},
      m_do_loiter{false},
      m_do_arm{false},
      m_do_helm_survey{false},
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

    if (key == "DESIRED_HEADING")
    {
      double heading = msg.GetDouble();
      m_setpoint_manager.updateDesiredHeading(heading);
    }
    else if (key == "DESIRED_SPEED")
    {
      double speed = msg.GetDouble();
      m_setpoint_manager.updateDesiredSpeed(speed);
    }
    else if (key == "DESIRED_ALTITUDE")
    {
      double alt = msg.GetDouble();
      m_setpoint_manager.updateDesiredAltitude(alt);
    }
    else if (key == "FLY_WAYPOINT")
    {
      setBooleanOnString(m_do_fly_to_waypoint, msg.GetString());
    }
    else if (key == "NEXT_WAYPOINT")
    {
      std::string wp_str = msg.GetString();
      double lat, lon, x, y;
      std::string vname;
      if (parseCoordinateString(wp_str, lat, lon, x, y, vname))
      {
        if (vname == m_vname || vname == "all")
        {
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
      std::string overide = msg.GetString();
      if (overide == "OFF" && m_autopilot_mode != AutopilotHelmMode::HELM_INACTIVE_LOITERING)
      {
        // m_autopilot_mode = AutopilotHelmState::HELM_INACTIVE;
        reportEvent("Helm is set to OFF");
        goToHelmMode(AutopilotHelmMode::HELM_INACTIVE);
      }
      else if (overide == "ON" && !isHelmDrive())
      {
        // m_autopilot_mode = AutopilotHelmState::HELM_ACTIVE;
        goToHelmMode(AutopilotHelmMode::HELM_ACTIVE);
      }
    }
    else if (key == "MOOS_MANUAL_OVERRIDE")
    {
      if (msg.GetString() == "true")
      {
        goToHelmMode(AutopilotHelmMode::HELM_PARKED, true);
      }
    }
    else if (key == "AUTOPILOT_MODE")
    {
      goToHelmMode(stringToHelmMode(msg.GetString()), true);
    }
    else if (key == "DO_TAKEOFF")
    {
      setBooleanOnString(m_do_takeoff, msg.GetString());
    }
    else if (key == "CHANGE_SPEED")
    {
      double speed_change = msg.GetDouble();
      m_do_change_speed_pair = std::make_pair(true, speed_change);
    }
    else if (key == "CHANGE_HEADING")
    {
      double heading_change = msg.GetDouble();
      m_do_change_heading_pair = std::make_pair(true, heading_change);
    }
    else if (key == "CHANGE_ALTITUDE")
    {
      double altitude_change = msg.GetDouble();
      m_do_change_altitude_pair = std::make_pair(true, altitude_change);

      std::string update_str = "altitude=" + doubleToString(altitude_change + m_uav_model.getTargetAltitudeAGL());
      Notify("CONST_ALTITUDE_UPDATE", update_str);
    }
    else if (key == "ARM_UAV")
    {
      setBooleanOnString(m_do_arm, msg.GetString());
    }
    else if (key == "RESET_SPEED_MIN")
    {
      setBooleanOnString(m_do_reset_speed, msg.GetString());
    }
    else if (key == "RETURN_TO_LAUNCH" || key == "RETURN")
    {
      setBooleanOnString(m_do_return_to_launch, msg.GetString());
    }
    else if (key == "LOITER")
    {
      setBooleanOnString(m_do_loiter, msg.GetString());
    }
    else if (key == "SURVEY")
    {
      setBooleanOnString(m_do_helm_survey, msg.GetString());
    }
    else if (key == "DEAD_MAN_POST_INTERRUPT")
    {
      m_warning_system_ptr->queue_monitorWarningForXseconds(
          "No heartbeats from GCS. Returning to launch", WARNING_DURATION);
      reportEvent("No heartbeats from GCS. Returning to launch");
      m_do_return_to_launch = true;
    }
    else if (key == "VIZ_HOME")
    {
      visualizeHomeLocation();
    }
    else if (key != "APPCAST_REQ")
    { // handled by AppCastingMOOSApp
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
  AppCastingMOOSApp::Iterate();
  static bool poll_uav_parameters = true;

  if (isHelmCommanding())
  {
    sendSetpointsToUAV();
  }

  if (m_do_takeoff)
  {
    tryDoTakeoff();
    m_do_takeoff = false;
    poll_uav_parameters = true;
  }

  if (m_do_fly_to_waypoint)
  {
    // if (tryFlyToWaypoint() && isHelmDrive())
    // { // Will report warning if command fails
    //   goToHelmMode(AutopilotHelmMode::HELM_TOWAYPT);
    //   m_uav_model.setLoiterLocationLatLon(m_uav_model.getNextWaypointLatLon());
    // }
    // else
    // {
    //   goToHelmMode(AutopilotHelmMode::HELM_INACTIVE);
    // }
    static bool running = false;
    if(!running){
      tryFlyToWaypoint_async();
      running = true;
    }

    auto result = future_try_get(m_fly_to_waypoint_promfut.fut);
    if (result.has_value())
    {
      if (result.value().success)
      {
        
        if (isHelmDrive()){
          goToHelmMode(AutopilotHelmMode::HELM_TOWAYPT);
        } else {
          goToHelmMode(AutopilotHelmMode::HELM_INACTIVE);
        }

        // m_uav_model.setLoiterLocationLatLon(m_uav_model.getNextWaypointLatLon());
        visualizeLoiterLocation(m_uav_model.getNextWaypointLatLon());

      }
      else
      {
        goToHelmMode(AutopilotHelmMode::HELM_INACTIVE);
        m_warning_system_ptr->queue_monitorWarningForXseconds("FAIL: " + result.value().message, WARNING_DURATION);
      }
      running = false;
      std::cout << "COMMANDED FLY TO WAYPOITN" << std::endl;
      m_do_fly_to_waypoint = false;
      poll_uav_parameters = true;

      m_fly_to_waypoint_promfut.reset();
    }
    

  }

  if (m_do_arm)
  {
    m_uav_model.sendArmCommandIfHealthyAndNotArmed();
    m_do_arm = false;
  }

  if (m_do_return_to_launch)
  {
    if (tryRTL() && isHelmDrive()) // Non-blocking
    { // Will report warning if command fails
      goToHelmMode(AutopilotHelmMode::HELM_RETURNING);
    }
    else
    {
      goToHelmMode(AutopilotHelmMode::HELM_INACTIVE);
    }

    m_do_return_to_launch = false;
  }

  if (m_do_loiter)
  {
    // Will report warning if command fails
    // if (tryloiterAtPos())
    //   goToHelmMode(AutopilotHelmMode::HELM_INACTIVE_LOITERING);


    static bool running = false;
    if(!running){
      tryLoiterAtPos_async();
      running = true;
    }

    auto result = future_try_get(m_loiter_at_pos_promfut.fut);
    if (result.has_value())
    {
      if (result.value().success)
      {
        goToHelmMode(AutopilotHelmMode::HELM_INACTIVE_LOITERING);
        visualizeLoiterLocation(m_uav_model.getCurrentLoiterLatLon());
      }
      else
      {
        goToHelmMode(AutopilotHelmMode::HELM_INACTIVE);
        m_warning_system_ptr->queue_monitorWarningForXseconds("FAIL: " + result.value().message, WARNING_DURATION);
      }
      running = false;
      m_do_loiter = false;
      m_loiter_at_pos_promfut.reset();
    }
  

  }

  if (m_do_helm_survey)
  {

    Notify("SURVEY_UPDATE", generateMissionPathSpec(m_waypointsXY_mission));

    if (isHelmDrive())
    {
      goToHelmMode(AutopilotHelmMode::HELM_SURVEYING);
    }
    else
    {
      m_warning_system_ptr->queue_monitorWarningForXseconds("Helm is not active, Cannot do survey", WARNING_DURATION);
    }
    m_do_helm_survey = false;
  }

  if (m_do_change_speed_pair.first)
  {
    double new_speed = m_uav_model.getTargetAirSpeed() + m_do_change_speed_pair.second;
    //  m_uav_model.commandAndSetAirSpeed(new_speed);

    if (m_uav_model.commandAndSetAirSpeed(new_speed))
    {
      postSpeedUpdateToBehaviors(new_speed);
      reportEvent("Trying to changed speed to " + doubleToString(new_speed));
    }

    if (m_command_groundSpeed)
    {
      m_uav_model.commandGroundSpeed(new_speed);
    }

    m_do_change_speed_pair.second = 0;
    m_do_change_speed_pair.first = false;

    poll_uav_parameters = true;
  }

  if (m_do_change_heading_pair.first)
  {
    double new_heading = angle360(m_uav_model.getTargetHeading() + m_do_change_heading_pair.second);

    if (isHelmCommanding())
    {
      m_warning_system_ptr->queue_monitorWarningForXseconds("Helm is commanding values. Restart Helm or wait for NothingToDo", WARNING_DURATION);
    }
    else if (m_uav_model.commandAndSetHeading(new_heading, isHelmNothingTodo()))
    {
      reportEvent("Changed heading to " + doubleToString(new_heading));
      poll_uav_parameters = true;
    };

    m_do_change_heading_pair.first = false;
    m_do_change_heading_pair.second = 0;
  }

  if (m_do_change_altitude_pair.first)
  {
    double new_altitude = m_uav_model.getTargetAltitudeAGL() + m_do_change_altitude_pair.second;
    double time_now = MOOSTime();
    bool success = m_uav_model.commandAndSetAltitudeAGL(new_altitude);

    if (!isHelmDrive() && !success)
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

    poll_uav_parameters = true;
  }

  if (m_do_reset_speed)
  {
    if (m_uav_model.commandAndSetAirSpeed(m_uav_model.getMinAirSpeed()))
      postSpeedUpdateToBehaviors(m_uav_model.getMinAirSpeed());

    if (m_command_groundSpeed)
    {
      m_uav_model.commandGroundSpeed(m_uav_model.getMinAirSpeed());
    }
    m_do_reset_speed = false;
    poll_uav_parameters = true;
  }

  postTelemetryUpdate(m_uav_prefix);

  m_warning_system_ptr->checkConditions(); // Check for warnings and remove/raise them as needed

  if (poll_uav_parameters)
  {
    m_uav_model.pollAllParametersAsync();
    poll_uav_parameters = false;
  }

  AppCastingMOOSApp::PostReport();
  return (true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool ArduBridge::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  std::cout << "Starting up ArduBridge with mission file name: " << GetMissionFileName() << std::endl;
  std::cout << "App name is: " << GetAppName() << std::endl;

  std::string ardupilot_url;
  std::pair<bool, std::string> url_protocol_pair{false, ""};

  STRING_LIST sParams;

  m_MissionReader.EnableVerbatimQuoting(false);
  if (!m_MissionReader.GetConfiguration(GetAppName(), sParams))
  {
    std::cout << "No config block found for " << GetAppName() << std::endl;
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
      handled = true;
    }
    else if (param == "vcolor")
    {
      m_vcolor = value;
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
    }
    else
    {
      reportConfigWarning("Invalid ArduPilot URL specified - Need to restart with a valid URL");
      std::cout << "Invalid ArduPilot URL specified - Need to restart with a valid URL" << std::endl;
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

  m_uav_model.start();


  // m_uav_model.subscribeToTelemetry();
  m_warning_system_ptr->checkConditions(); // Check for warnings and remove/raise them as needed

  postSpeedUpdateToBehaviors(m_uav_model.getTargetAirSpeed());

  // if (m_is_simulation) // Should be set in the param file for simulation
  // {
  //   m_uav_model.setTargetAirSpeed(12); // target speed in simulation is constant 11.2 when default throttle is 50%
  // }

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

  Register("CHANGE_SPEED", 0);
  Register("ARM_UAV", 0);
  Register("RESET_SPEED_MIN", 0);

  Register("CHANGE_HEADING", 0);

  Register("CHANGE_ALTITUDE", 0);
  Register("RETURN_TO_LAUNCH", 0);
  Register("RETURN", 0);
  Register("LOITER", 0);
  Register("SURVEY", 0);
  Register("NEXT_WAYPOINT", 0);

  Register("HELM_STATUS", 0);

  // Register("MOOS_MANUAL_OVERIDE", 0);
  Register("MOOS_MANUAL_OVERRIDE", 0);

  Register("AUTOPILOT_MODE", 0);

  Register("DEAD_MAN_POST_INTERRUPT", 0);

  // To be sent to Ardupilot
  Register("DESIRED_HEADING", 0);
  Register("DESIRED_SPEED", 0);
  Register("DESIRED_ALTITUDE", 0);

  // For PMarineViewer
  Register("VIZ_HOME", 0);
}

//------------------------------------------------------------
// Procedure: buildReport()

bool ArduBridge::buildReport()
{
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

  m_msgs << "UAV Parameters: " << std::endl;
  m_msgs << "------------------ " << std::endl;
  m_msgs << "       Min AirSpeed:" << uav_minAirSpeed << " m/s" << std::endl;
  m_msgs << "       Max AirSpeed:" << uav_maxAirSpeed << " m/s" << std::endl;

  double lat = m_uav_model.getLatitude();
  double lon = m_uav_model.getLongitude();

  auto uav_altitude_msl = m_uav_model.getAltitudeMSL();
  auto uav_targetHeading = m_uav_model.getTargetHeading();
  auto uav_targetAirspeed = m_uav_model.getTargetAirSpeed();

  double nav_x, nav_y;
  if (m_geo_ok)
  {
    m_geodesy.LatLong2LocalGrid(lat, lon, nav_y, nav_x);
  }

  m_msgs << "State Information: " << std::endl;
  m_msgs << "------------------ " << std::endl;
  m_msgs << "       Helm Autonomy Mode: " << helmModeToString(m_autopilot_mode) << std::endl;
  m_msgs << "  (Latitude , Longditute): " << lat << " , " << lon << std::endl;
  m_msgs << "                  (X , Y): " << nav_x << " , " << nav_y << std::endl;
  // m_msgs << "    Target Altitude (AGL): " << m_uav_model.getTargetAltitudeAGL() << " m" << "    Last Sent (AGL): " << m_uav_model.getLastSentTargetAltitudeAGL() << " m" << std::endl;
  // m_msgs << "           Altitude (AGL): " << m_uav_model.getAltitudeAGL() << " m (Depth/Z: " << -m_uav_model.getAltitudeAGL() << " m)" << std::endl;
  m_msgs << "           Altitude (MSL): " << uav_altitude_msl << " m" << std::endl;
  // m_msgs << "          Target Airspeed: " << m_uav_model.getTargetAirSpeed() << " m/s" << std::endl;
  // m_msgs << "          AirSpeed (SOG) : " << m_uav_model.getSOG() << " m/s)" << std::endl;
  m_msgs << "     Target Heading (COG): " << uav_targetHeading << " deg" << std::endl;
  // m_msgs << "                  Heading: " << m_uav_model.getHeading() << " deg" << std::endl;

  m_msgs << "-------------------------------------------" << std::endl;

  auto hi = "sfafdesfar";

  ACTable actb(4);
  actb << "States | Measurments | Helm | Targets";
  actb.addHeaderLines();
  actb << "Speed:" << m_uav_model.getSOG() << m_setpoint_manager.readDesiredSpeed() << uav_targetAirspeed;
  actb << "Heading:" << m_uav_model.getHeading() << m_setpoint_manager.readDesiredHeading() << m_uav_model.getTargetHeading();
  actb << "Altitude:" << m_uav_model.getAltitudeAGL() << m_setpoint_manager.readDesiredAltitudeAGL() << m_uav_model.getTargetAltitudeAGL();

  m_msgs << actb.getFormattedString() << std::endl;
  m_msgs << "-------------------------------------------" << std::endl;

  ACTable actabd(3);
  actabd << "Waypoint | Lat | Lon";
  actabd.addHeaderLines();
  actabd << "Home Coord:" << m_uav_model.getHomeLatLon().x() << m_uav_model.getHomeLatLon().y();
  actabd << "Next Wypt Coord:" << m_uav_model.getNextWaypointLatLon().x() << m_uav_model.getNextWaypointLatLon().y();
  actabd << "Heading Wypt Coord:" << m_uav_model.getHeadingWaypointLatLon().x() << m_uav_model.getHeadingWaypointLatLon().y();
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

void ArduBridge::sendSetpointsToUAV(bool forceSend)
{

  if (!m_setpoint_manager.isValid())
  {
    m_warning_system_ptr->queue_monitorWarningForXseconds("No valid setpoints to send", 2);
    return;
  }
  auto desired_heading = m_setpoint_manager.getDesiredHeading();
  auto desired_speed = m_setpoint_manager.getDesiredSpeed();
  auto desired_altitude_agl = m_setpoint_manager.getDesiredAltitude();

  if (forceSend)
  {
    desired_heading = m_setpoint_manager.readDesiredHeading();
    desired_speed = m_setpoint_manager.readDesiredSpeed();
    desired_altitude_agl = m_setpoint_manager.readDesiredAltitudeAGL();
  }

  if (desired_heading.has_value())
  {
    m_uav_model.commandAndSetHeading(desired_heading.value());
  }

  if (desired_speed.has_value())
  {
    m_uav_model.commandAndSetAirSpeed(desired_speed.value());

    if (m_command_groundSpeed)
    {
      m_uav_model.commandGroundSpeed(desired_speed.value());
    }
  }

  if (desired_altitude_agl.has_value())
  {
    // m_uav_model.setTargetAltitudeAGL(desired_altitude_agl.value());
    m_uav_model.commandAndSetAltitudeAGL(desired_altitude_agl.value());
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

  if (m_geo_ok)
  {
    double nav_x, nav_y;
    m_geodesy.LatLong2LocalGrid(lat, lon, nav_y, nav_x);

    NotifyIfNonNan(prefix + "_X", nav_x, m_curr_time);
    NotifyIfNonNan(prefix + "_Y", nav_y, m_curr_time);

    visualizeHdgHoldTarget(isHelmDrive() || m_uav_model.isHoldHeadingGuidedSet());
    // if (m_uav_model.isGuidedMode())
    //   visualizeHdgVector(nav_x, nav_y, m_uav_model.getTargetAirSpeed(), m_uav_model.getTargetHeading());
  }

  NotifyIfNonNan(prefix + "_SPEED", m_uav_model.getSOG(), m_curr_time);

  NotifyIfNonNan(prefix + "_ALTITUDE", m_uav_model.getAltitudeAGL(), m_curr_time);
  NotifyIfNonNan(prefix + "_DEPTH", -m_uav_model.getAltitudeAGL(), m_curr_time);

  // NotifyIfNonNan(prefix+"_Z", -m_uav_model.getDepth(), m_curr_time);

  // NotifyIfNonNan(prefix+"_ROLL", m_uav_model.getPitch(), m_curr_time);
  // NotifyIfNonNan(prefix+"_PITCH", m_uav_model.getPitch(), m_curr_time);
  NotifyIfNonNan(prefix + "_HEADING", m_uav_model.getHeading(), m_curr_time);

  // NotifyIfNonNan(prefix+"_HEADING_OVER_GROUND", hog, m_curr_time);
  // NotifyIfNonNan(prefix + "_SPEED_OVER_GROUND", m_uav_model.getAirSpeed(), m_curr_time);

  static bool prev_in_air = false;
  if (m_uav_model.isInAir() != prev_in_air)
  {

    prev_in_air = m_uav_model.isInAir();
    Notify("DEPLOY", boolToString(prev_in_air));
  }
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
  double lat = m_uav_model.getHomeLatLon().x();
  double lon = m_uav_model.getHomeLatLon().y();

  if (!lat || !lon)
  {
    m_warning_system_ptr->queue_monitorWarningForXseconds("Cannot Visualize Home: NAN Values at lat or long", 5);
    return;
  }

  double nav_x, nav_y;
  if (m_geo_ok)
  {
    m_geodesy.LatLong2LocalGrid(lat, lon, nav_y, nav_x);
  }
  else
  {
    m_warning_system_ptr->queue_monitorWarningForXseconds("Geodesy not initialized", 5);
    return;
  }

  XYMarker marker(nav_x, nav_y);
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
  std::string spec = marker.get_spec() + ",color=" + m_vcolor + ",scale=" + doubleToString(MARKER_WIDTH);
  Notify("VIEW_MARKER", spec);

  reportEvent("Set marker at loiter location: " + spec);
}

void ArduBridge::visualizeHeadingWaypoint(const XYPoint &heading_coord, bool visualize)
{

  XYPoint point = transformLatLonToXY(heading_coord);
  if (point == XYPoint(0, 0) && visualize)
  {
    m_warning_system_ptr->queue_monitorWarningForXseconds("No Heading waypoiny set: NAN Values at lat or long", 5);
    return;
  }

  point.set_label("Hold Heading point");
  point.set_vertex_size(HEADING_POINT_SIZE);
  point.set_active(visualize);
  std::string spec = point.get_spec() + ",color=" + m_vcolor;
  Notify("VIEW_POINT", spec);

  //  reportEvent("Set constant Heading point at with: " + spec);
}

void ArduBridge::visualizeHdgVector(double x, double y, double magnitude, double angle, bool visualize)
{

  XYVector vector(x, y, magnitude * 2, angle);
  vector.set_active(visualize);

  vector.set_label("_");
  // vector.set_edge_size(4);

  std::string spec = vector.get_spec() + ",color=" + m_vcolor;
  Notify("VIEW_VECTOR", spec);

  // reportEvent("Set vector at wbldith: " + spec);
}

void ArduBridge::visualizeHdgHoldTarget(bool visualize)
{

  if (!m_uav_model.isHoldHeadingGuidedSet() && visualize)
    return;

  double lat = m_uav_model.getLatitude();
  double lon = m_uav_model.getLongitude();

  if (!lat || !lon || !m_geo_ok)
    return;

  double nav_x, nav_y;
  m_geodesy.LatLong2LocalGrid(lat, lon, nav_y, nav_x);

  visualizeHdgVector(nav_x, nav_y, m_uav_model.getTargetAirSpeed(), m_uav_model.getTargetHeading(), visualize);
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
    }
    else
    {
      std::cerr << "Unknown key: " << key << std::endl;
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

// Helper funcitons
bool ArduBridge::tryDoTakeoff()
{
  if (isHelmDrive())
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

  if (isHelmDrive())
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

void ArduBridge::tryFlyToWaypoint_async()
{

  XYPoint wp = m_uav_model.getNextWaypointLatLon();
  if (wp == XYPoint(0, 0))
  {
    // m_warning_system_ptr->queue_monitorWarningForXseconds("No waypoint set", WARNING_DURATION);
    m_fly_to_waypoint_promfut.prom.set_value(ResultPair{false, "No waypoint set"});
    return;
  }

  if (isHelmDrive())
  {
    std::string update_str = "points=" + xypointToString(m_tonext_waypointXY);
    Notify("TOWAYPT_UPDATE", update_str);
    m_fly_to_waypoint_promfut.prom.set_value(ResultPair{true, ""});
    return;
  }

  
  // Helm is Park from here:

  // static std::future<void> future; // To avoid the destructor of being called
  m_fly_to_waypoint_promfut.fut = std::async(std::launch::async, [wp, this]() {
    
    // Helm is inactive. Send the waypoint directly to the UAV

    if (!m_uav_model.commandGoToLocationXY(wp))
    {
      return ResultPair{false, "Failed sending command"};
    }

    std::cout << "Successfully sent waypoint to UAV" << std::endl;
    return ResultPair{true, ""};
    }
  );

}



bool ArduBridge::tryRTL()
{

  if (isHelmDrive())
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

void ArduBridge::tryLoiterAtPos_async(const XYPoint &loiter_coord, bool holdCurrentAltitude){

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

  m_loiter_at_pos_promfut.fut = std::async(std::launch::async, [loiter_latlon, holdCurrentAltitude, this]() {
    
    if (!m_uav_model.commandLoiterAtPos(loiter_latlon, holdCurrentAltitude))
    {
      return ResultPair{false, "Failed sending command"};
    }

    if (loiter_latlon == m_uav_model.getNextWaypointLatLon())
    {
      m_uav_model.setNextWaypointLatLon(XYPoint(0, 0));
    }

    return ResultPair{true, ""};
    }
  );


}


void ArduBridge::goToHelmMode(AutopilotHelmMode to_state, bool fromGCS)
{

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
    visualizeHdgHoldTarget(false);
    break;

  case AutopilotHelmMode::HELM_INACTIVE_LOITERING:
  case AutopilotHelmMode::HELM_INACTIVE:
    visualizeHdgHoldTarget(false);
    break;
  case AutopilotHelmMode::HELM_ACTIVE:
  case AutopilotHelmMode::HELM_SURVEYING:
  case AutopilotHelmMode::HELM_RETURNING:
  case AutopilotHelmMode::HELM_TOWAYPT:
    Notify("MOOS_MANUAL_OVERRIDE", "false");

  default:
    break;
  }
}

void ArduBridge::initializeStateTransitionFunctions()
{

  m_statetransition_functions[std::make_pair(AutopilotHelmMode::HELM_TOWAYPT, AutopilotHelmMode::HELM_INACTIVE_LOITERING)] = [this]()
  {
    visualizeHeadingWaypoint(XYPoint(0, 0), false);
  };
}

std::string ArduBridge::xypointToString(const XYPoint &point) const
{
  return doubleToString(point.x()) + "," + doubleToString(point.y());
}