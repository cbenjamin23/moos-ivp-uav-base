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
#include <cli_arg.h>



#include "WarningSystem.h"
#include "UAV_Model.h"
#include "SetpointManager.h"




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
    void postTelemetryUpdate(const std::string& prefix );


    // Send command to UAV
    void sendSetpointsToUAV(bool forceSend = false);

  private: // Configuration variables
    std::string  m_uav_prefix;

    static inline std::map<mavsdk::CliArg::Protocol, std::string> protocol2str = {
      {mavsdk::CliArg::Protocol::None, "None"},
      {mavsdk::CliArg::Protocol::Udp, "Udp"},
      {mavsdk::CliArg::Protocol::Tcp, "Tcp"},
      {mavsdk::CliArg::Protocol::Serial, "Serial"}
    };
    mavsdk::CliArg  m_cli_arg;

  
    bool         m_geo_ok;
    CMOOSGeodesy m_geodesy;

  
  private: // Autopilot Helm states

    enum class AutopilotHelmState{
      HELM_INACTIVE,
      HELM_INACTIVE_LOITERING,
      HELM_ACTIVE,
      HELM_TOWAYPT,
      HELM_SURVEYING,
      HELM_RETURNING,
      HELM_UNKOWN,
    };

  // map containing the stateName

    const std::vector<std::pair<AutopilotHelmState, std::string>> stateStringPairs = {
      {AutopilotHelmState::HELM_INACTIVE, "HELM_INACTIVE"},
      {AutopilotHelmState::HELM_INACTIVE_LOITERING, "HELM_INACTIVE_LOITERING"},
      {AutopilotHelmState::HELM_ACTIVE, "HELM_ACTIVE"},
      {AutopilotHelmState::HELM_TOWAYPT, "HELM_TOWAYPT"},
      {AutopilotHelmState::HELM_SURVEYING, "HELM_SURVEYING"},
      {AutopilotHelmState::HELM_RETURNING, "HELM_RETURNING"},
      {AutopilotHelmState::HELM_UNKOWN, "HELM_UNKOWN"},
    };

    std::string helmStateToString(AutopilotHelmState state) const {
      for (const auto& [stateEnum, stateStr] : stateStringPairs) {
          if (stateEnum == state) {
              return stateStr;
          }
      }
      return "HELM_UNKOWN";
    }

    AutopilotHelmState stringToHelmState(const std::string& stateStr) const {
      for (const auto& [stateEnum, stateStrVal] : stateStringPairs) {
          if (stateStrVal == stateStr) {
              return stateEnum;
          }
      }
      return AutopilotHelmState::HELM_UNKOWN;  
    }

    

  private:

    const double MARKER_WIDTH = 14.0;
    const double HEADING_POINT_SIZE = 5;

    void visualizeHomeLocation();
    void visualizeLoiterLocation(const XYPoint& loiter_coord, bool visualize = true);
    void visualizeHeadingWaypoint(const XYPoint& heading_coord, bool visualize = true);

    // bool evaluateBoolFromString(const std::string& str) const { bool b; setBooleanOnString(b,str); return b;}
    bool parseCoordinateString(const std::string& input, double& lat, double& lon, double& x, double& y , std::string& vname) const;
  
  
    AutopilotHelmState getTransitionAutopilotHelmState() const;  
  
  private: // Helperfunctions

  std::string generateMissionPathSpec(const std::vector<XYPoint>& points) const;

  std::string xypointToString(const XYPoint& point) const;
  XYPoint transformLatLonToXY(const XYPoint& lat_lon);
  bool isHelmON(){return m_autopilot_mode != AutopilotHelmState::HELM_INACTIVE && m_autopilot_mode != AutopilotHelmState::HELM_INACTIVE_LOITERING;};
  
  void goToHelmState(AutopilotHelmState state);


  bool tryDoTakeoff();
  bool tryFlyToWaypoint();
  bool tryRTL();
  bool tryloiterAtPos(const XYPoint& loiter_coord = XYPoint(0, 0), bool holdCurrentAltitude = false);
  
  private: // State variables
    // For UAV

    std::string   m_vname;
    bool          m_is_simulation;
    bool          m_is_helm_parked;
    bool          m_command_groundSpeed;


    std::shared_ptr<WarningSystem> m_warning_system_ptr;
    UAV_Model       m_uav_model;
    SetpointManager m_setpoint_manager;
    
    bool  m_do_fly_to_waypoint;
    bool  m_do_takeoff;
    std::pair<bool, double> m_do_change_speed_pair;
    std::pair<bool, double> m_do_change_altitude_pair;
    bool  m_do_reset_speed;

    bool  m_do_arm;
    bool  m_do_return_to_launch;
    bool  m_do_loiter;
    bool  m_do_helm_survey;
  
    AutopilotHelmState m_autopilot_mode;


    // Helm utility

    XYPoint   m_tonext_waypointXY;
    std::vector<XYPoint> m_waypointsXY_mission;

};



#endif 

