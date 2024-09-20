/************************************************************/
/*    NAME: Steve Carter Feujo Nomeny                       */
/*    ORGN: NTNU, MIT                                       */
/*    FILE: UAV_Model.h                                     */
/*    DATE: September 9th, 2024                             */
/************************************************************/

#pragma once

#include <string>
#include "NodeRecord.h"



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

class UAV_Model
{
public:
    UAV_Model(WarningSystem &ws);
    virtual  ~UAV_Model() {}

    bool   connectToUAV(std::string url);
    
    bool   goToLocation(double latitude_deg, double longitude_deg, float absolute_altitude_m, float yaw_deg) const;
    bool   startMission() const;
    bool   sendArmCommandIfHealthyAndNotArmed();

    bool   gatherTelemetry();

    // Setters
    void   setCallbackMOOSTrace(const std::function<void(const std::string&)>& callback) {callbackMOOSTrace = callback ;}
    void   setCallbackReportRunW(const std::function<void(const std::string&)>& callback) {callbackReportRunW = callback ;}
    void   setCallbackRetractRunW(const std::function<void(const std::string&)>& callback) {callbackRetractRunW = callback ;}
    void   registerWarningSystem(WarningSystem &ws) {m_warning_system = ws;}


    // bool   setParam(std::string, double);
    // bool   initPosition(const std::string&);

    // Getters
    XYPoint   getHomeCoord() const {return(m_home_coord);}
    double    getLatitude() const {return(m_position.latitude_deg);}
    double    getLongitude() const {return(m_position.longitude_deg);}
    double    getSpeed() const {return( sqrt(m_velocity_ned.north_m_s*m_velocity_ned.north_m_s
                                               + m_velocity_ned.east_m_s*m_velocity_ned.east_m_s)
                                               + m_velocity_ned.down_m_s*m_velocity_ned.down_m_s );}

    double    getSpeedXY() const {return( sqrt(m_velocity_ned.north_m_s*m_velocity_ned.north_m_s
                                               + m_velocity_ned.east_m_s*m_velocity_ned.east_m_s) );}
    double    getHeading() const {return(m_attitude_ned.yaw_deg);}
    double    getAltitudeAGL() const {return(m_position.relative_altitude_m);} 
    double    getAltitudeMSL() const {return(m_position.absolute_altitude_m);}
    
    double    getRoll() const {return(m_attitude_ned.roll_deg);}
    double    getPitch() const {return(m_attitude_ned.pitch_deg);}


 protected:

    WarningSystem& m_warning_system;
    // Callbacks for debug messages
    std::function<void(const std::string&)> callbackMOOSTrace;
    std::function<void(const std::string&)> callbackReportRunW;
    std::function<void(const std::string&)> callbackRetractRunW;    

    void MOOSTraceFromCallback(const std::string& msg) const {if(callbackMOOSTrace) callbackMOOSTrace(msg);};
    void reportRunWarningFromCallback(const std::string& msg) const {if(callbackReportRunW) callbackReportRunW(msg);};
    void retractRunWarningFromCallback(const std::string& msg) const {if(callbackRetractRunW) callbackRetractRunW(msg);};


 protected:
    std::shared_ptr<mavsdk::Mavsdk>     m_mavsdk_ptr;
    std::unique_ptr<mavsdk::MissionRaw> m_mission_raw_ptr;
    std::unique_ptr<mavsdk::Action>     m_action_ptr;
    std::unique_ptr<mavsdk::Telemetry>  m_telemetry_ptr;

    bool       m_health_all_ok;
    bool       m_is_armed;
  
    
    // Telemetry
    mavsdk::Telemetry::Position m_position;       // NAV_X, NAV_Y           
    mavsdk::Telemetry::EulerAngle m_attitude_ned;
    mavsdk::Telemetry::VelocityNed m_velocity_ned;
    
    mavsdk::Telemetry::Battery m_battery;

    double     m_rudder;


    XYPoint   m_home_coord;



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
    
bool create_missionPlan(std::vector<mavsdk::MissionRaw::MissionItem>& mission_plan, double lat_deg_home = -35.359833, double lon_deg_home = 149.164703);




