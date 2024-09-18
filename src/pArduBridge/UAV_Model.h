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

class UAV_Model
{
public:
  UAV_Model();
  virtual  ~UAV_Model() {}

  void   resetTime(double time);

  bool   connectToUAV(std::string url, const std::function<void(const std::string&)>& callbackDebug = nullptr);
  
  bool   goToLocation(double latitude_deg, double longitude_deg, float absolute_altitude_m, float yaw_deg, const std::function<void(const std::string&)>& callbackDebug = nullptr) const;
  bool   startMission(const std::function<void(const std::string&)>& callbackDebug = nullptr) const;
  bool   sendArmCommandIfHealthyAndNotArmed(const std::function<void(const std::string&)>& callbackDebug = nullptr);

  // Setters
  bool   setParam(std::string, double);

  void   setRudder(double v)          {m_rudder = v;}
  void   setRudder(double, double);
  void   setThrust(double v)          {m_thrust = v;}
  void   setElevator(double v)        {m_elevator = v;}

  bool   initPosition(const std::string&);

  // Getters
  double     getThrust() const       {return(m_thrust);}
  NodeRecord getNodeRecord() const   {return(m_record);}


 protected:


 protected:

    std::shared_ptr<mavsdk::Mavsdk>     m_mavsdk_ptr;
    std::unique_ptr<mavsdk::MissionRaw> m_mission_raw_ptr;
    std::unique_ptr<mavsdk::Action>     m_action_ptr;
    std::unique_ptr<mavsdk::Telemetry>  m_telemetry_ptr;

    bool       m_health_all_ok;
    bool       m_is_armed;
  
  
    NodeRecord m_record;       // NAV_X, NAV_Y           
  
    double     m_rudder;


    bool       m_paused;
    double     m_thrust;
    double     m_elevator;





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
    
bool create_missionPlan(std::vector<mavsdk::MissionRaw::MissionItem>& mission_plan, float lat_deg_home = -35.359833, float lon_deg_home = 149.164703);




