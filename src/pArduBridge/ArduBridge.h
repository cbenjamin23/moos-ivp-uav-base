/************************************************************/
/*    NAME: Steve Carter Feujo Nomeny                       */
/*    ORGN: NTNU, MIT                                       */
/*    FILE: ArduBridge.h                                    */
/*    DATE: September 9th, 2024                             */
/************************************************************/

#ifndef ArduBridge_HEADER
#define ArduBridge_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

#include "MOOSGeodesy.h"
#include <mavsdk/cli_arg.h>

#include "WarningSystem.h"
#include "UAV_Model.h"
#include "SetpointManager.h"

#include <map>
#include <functional>
#include <utility> // for std::pair

#include <future>

class ArduBridge : public AppCastingMOOSApp
{
public:
  ArduBridge();
  ~ArduBridge();

protected: // Standard MOOSApp functions to overload
  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();

protected: // Standard AppCastingMOOSApp function to overload
  bool buildReport();

protected:
  void registerVariables();

  // Notify to DB
  void postTelemetryUpdate(const std::string &prefix);

  void postSpeedUpdateToBehaviors(double speed);
  // Send command to UAV
  void sendSetpointsToUAV(bool forceSend = false);

private: // Configuration variables
  std::string m_uav_prefix;

  static inline std::map<mavsdk::CliArg::Protocol, std::string> protocol2str = {
      {mavsdk::CliArg::Protocol::None, "None"},
      {mavsdk::CliArg::Protocol::Udp, "Udp"},
      {mavsdk::CliArg::Protocol::Tcp, "Tcp"},
      {mavsdk::CliArg::Protocol::Serial, "Serial"}};
  mavsdk::CliArg m_cli_arg;

  bool m_geo_ok;
  CMOOSGeodesy m_geodesy;

private: // Autopilot Helm states
  enum class AutopilotHelmMode
  {
    HELM_PARKED, // Helm is parked

    // Inactive states where the Helm is not in control but still in Drive
    HELM_INACTIVE,
    HELM_INACTIVE_LOITERING,

    // Active states where the Helm is in control
    HELM_ACTIVE, // Helm has nothing to do
    HELM_TOWAYPT,
    HELM_SURVEYING,
    HELM_RETURNING,
    HELM_UNKOWN,
  };

  // map containing the stateName
  const std::vector<std::pair<AutopilotHelmMode, std::string>> stateStringPairs = {
      {AutopilotHelmMode::HELM_PARKED, "HELM_PARKED"},
      {AutopilotHelmMode::HELM_INACTIVE, "HELM_INACTIVE"},
      {AutopilotHelmMode::HELM_INACTIVE_LOITERING, "HELM_INACTIVE_LOITERING"},
      {AutopilotHelmMode::HELM_ACTIVE, "HELM_ACTIVE"},
      {AutopilotHelmMode::HELM_TOWAYPT, "HELM_TOWAYPT"},
      {AutopilotHelmMode::HELM_SURVEYING, "HELM_SURVEYING"},
      {AutopilotHelmMode::HELM_RETURNING, "HELM_RETURNING"},
      {AutopilotHelmMode::HELM_UNKOWN, "HELM_UNKOWN"},
  };

  std::string helmModeToString(AutopilotHelmMode state) const
  {
    for (const auto &[stateEnum, stateStr] : stateStringPairs)
    {
      if (stateEnum == state)
      {
        return stateStr;
      }
    }
    return "HELM_UNKOWN";
  }

  AutopilotHelmMode stringToHelmMode(const std::string &stateStr) const
  {
    for (const auto &[stateEnum, stateStrVal] : stateStringPairs)
    {
      if (stateStrVal == stateStr)
      {
        return stateEnum;
      }
    }
    return AutopilotHelmMode::HELM_UNKOWN;
  }

  using StateTransition = std::pair<AutopilotHelmMode, AutopilotHelmMode>;
  std::map<StateTransition, std::function<void()>> m_statetransition_functions;
  void initializeStateTransitionFunctions();

private:
  const double MARKER_WIDTH = 10.0;
  const double HEADING_POINT_SIZE = 5;

  void visualizeHomeLocation();
  void visualizeLoiterLocation(const XYPoint &loiter_coord, bool visualize = true);
  void visualizeHeadingWaypoint(const XYPoint &heading_coord, bool visualize = true);
  void visualizeHdgVector(double x, double y, double magnitude, double angle, bool visualize = true);

  void visualizeHdgHoldTarget(bool visualize = true);

  // bool evaluateBoolFromString(const std::string& str) const { bool b; setBooleanOnString(b,str); return b;}
  bool parseCoordinateString(const std::string &input, double &lat, double &lon, double &x, double &y, std::string &vname) const;

  // AutopilotHelmMode getTransitionAutopilotHelmState() const;

private: // Helperfunctions
  std::string generateMissionPathSpec(const std::vector<XYPoint> &points) const;

  std::string xypointToString(const XYPoint &point) const;
  XYPoint transformLatLonToXY(const XYPoint &lat_lon);
  bool isHelmDrive() const { return (m_autopilot_mode != AutopilotHelmMode::HELM_INACTIVE && m_autopilot_mode != AutopilotHelmMode::HELM_INACTIVE_LOITERING && m_autopilot_mode != AutopilotHelmMode::HELM_PARKED); };
  bool isHelmCommanding() const { return (m_autopilot_mode == AutopilotHelmMode::HELM_SURVEYING || m_autopilot_mode == AutopilotHelmMode::HELM_TOWAYPT || m_autopilot_mode == AutopilotHelmMode::HELM_RETURNING); };
  bool isHelmNothingTodo() const { return (m_autopilot_mode == AutopilotHelmMode::HELM_ACTIVE); };

  void goToHelmMode(AutopilotHelmMode state, bool fromGCS = false);

  bool tryDoTakeoff();
  bool tryFlyToWaypoint();
  bool tryRTL(); // Non-blocking
  bool tryloiterAtPos(const XYPoint &loiter_coord = XYPoint(0, 0), bool holdCurrentAltitude = false);
  

protected: // async

template <typename T>
 class PromiseFuture{
  public:
  std::promise<T> prom;
  std::future<T> fut = prom.get_future();

  void reset() { 
    prom = std::promise<T>();
    fut = prom.get_future();
  }
 };

struct ResultPair {
    bool success;
    std::string message;
};

/// @brief Future and promise for async functions
template <typename T>
std::optional<T> future_try_get(std::future<T>& future) {
    if (future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
        return future.get(); // Retrieve the value if it's ready
    }
    return std::nullopt; // Return an empty optional if not ready
}

  void tryFlyToWaypoint_async();
  PromiseFuture<ResultPair> m_fly_to_waypoint_promfut;

  void tryLoiterAtPos_async(const XYPoint &loiter_coord = XYPoint(0, 0), bool holdCurrentAltitude = false);
  PromiseFuture<ResultPair> m_loiter_at_pos_promfut;



private: // State variables
  // For UAV

  std::string m_vname;
  std::string m_vcolor;
  bool m_is_simulation;
  bool m_command_groundSpeed;

  std::shared_ptr<WarningSystem> m_warning_system_ptr;
  UAV_Model m_uav_model;
  SetpointManager m_setpoint_manager;

  std::pair<bool, double> m_do_change_speed_pair;
  std::pair<bool, double> m_do_change_heading_pair;
  std::pair<bool, double> m_do_change_altitude_pair;
  bool m_do_reset_speed;
  bool m_do_fly_to_waypoint;
  bool m_do_takeoff;
  bool m_do_arm;
  bool m_do_return_to_launch;
  bool m_do_loiter;
  bool m_do_helm_survey;

  AutopilotHelmMode m_autopilot_mode;

  // Helm utility
  XYPoint m_tonext_waypointXY;
  std::vector<XYPoint> m_waypointsXY_mission;
};

#endif
