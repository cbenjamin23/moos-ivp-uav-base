/************************************************************/
/*    NAME: Steve Carter Feujo Nomeny                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: ArduBridge.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef ArduBridge_HEADER
#define ArduBridge_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"


#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/mission_raw/mission_raw.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/param/param.h>
#include <chrono>
#include <functional>
#include <future>
#include <iostream>
#include <ratio>
#include <thread>

using namespace mavsdk;


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

    bool ConnectToUAV(std::string url);

  private: // Configuration variables

    std::string m_ArduPilot_url;

  private: // State variables
    // For UAV
    std::shared_ptr<Mavsdk>     m_mavsdk_ptr;
    std::unique_ptr<MissionRaw> m_mission_raw_ptr;
    std::unique_ptr<Action>     m_action_ptr;
    std::unique_ptr<Telemetry>  m_telemetry_ptr;

    bool  m_health_all_ok;
    bool  m_do_fly_to_waypoint;
    bool  m_do_takeoff;

    double m_lat_deg_home;
    double m_lon_deg_home;
};

MissionRaw::MissionItem make_mission_item_wp(
    float latitude_deg1e7,
    float longitude_deg1e7,
    int32_t altitude_m,
    float param1,
    MAV_FRAME frame,
    MAV_CMD command,
    float p2 = 0,
    float p3 = 0);
    
bool create_missionPlan(std::vector<mavsdk::MissionRaw::MissionItem>& mission_plan, float lat_deg_home = -35.359833, float lon_deg_home = 149.164703);



#endif 

