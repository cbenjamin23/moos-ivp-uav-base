/************************************************************/
/*    NAME: Steve Nomeny and Filip Stromstad                */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: GenRescue.h                                     */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef GenRescue_HEADER
#define GenRescue_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "XYPoint.h"


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

int return_x_value(std::string point);
int return_y_value(std::string point);
std::string return_id_value(std::string point);

using namespace mavsdk;

namespace path {

  enum class PathAlgorithm
  {
    GREEDY_SHORTEST_PATH,
    LOOK_AHEAD,
    ADVERSARY_PATH_PLANNING,
    INVALID
  };

  static std::map<std::string,PathAlgorithm> path_algorithm_map = {
    {"greedy", PathAlgorithm::GREEDY_SHORTEST_PATH},
    {"look_ahead", PathAlgorithm::LOOK_AHEAD},
    {"adversary", PathAlgorithm::ADVERSARY_PATH_PLANNING},
    {"invalid", PathAlgorithm::INVALID}
  }; 
  
  static PathAlgorithm stringToEnum(const std::string &path_algorithm)
  {
    auto it = path_algorithm_map.find(path_algorithm);
    if (it != path_algorithm_map.end())
    {
      return it->second;
    }
    return PathAlgorithm::INVALID;
  }

  static std::string enumToString(const PathAlgorithm& path_algorithm)
  {
    for (const auto &pair : path::path_algorithm_map)
    {
      if (pair.second == path_algorithm)
      {
        return pair.first;
      }
    }
    return "unknown";
  }
} // namespace


struct ShipStatus {
  XYPoint position;
  double speed;
  ShipStatus(const XYPoint& position, double speed) : position(position), speed(speed) {}
  ShipStatus() : position(XYPoint(0,0)), speed(0) {}
};


class GenRescue : public AppCastingMOOSApp
{
 public:
   GenRescue();
   ~GenRescue();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();
   std::vector<XYPoint> solveGreedyShortestPath() const;

   std::vector<XYPoint> solveGreedyNLegs(const XYPoint& currentLocation, std::vector<XYPoint> unvisitedPoints, int N) const;
   std::vector<XYPoint> twoStepLookahead(const XYPoint& ship_position, const std::vector<XYPoint>& points) const;
   std::vector<XYPoint> updateOurPath(const ShipStatus& myShip, const ShipStatus& adversaryShip, const std::vector<XYPoint>& adverseryPath, std::vector<double>& adversaryETA) const;
   std::vector<XYPoint> adversaryPathPlanning() ;

   std::string generatePathSpec(std::vector<XYPoint> points) const;
   std::vector<XYPoint>::const_iterator findWaypointVisited() const;
   void removeParkPoint(std::vector<XYPoint>& points, const XYPoint& parkPoint);

 private: // Configuration variables
   uint                 m_visit_radius;
   std::string          m_vname;
   std::string          m_scout_name;
   std::string          m_adversaryName;
   bool                 m_pav90;
   int                  m_path_update_period;

   double               m_position_update_period;
   double               m_still_radius;

 private: // State variables
   std::vector<XYPoint> m_points_visited;
   std::vector<XYPoint> m_unvisited_points;
   uint                 m_num_received_points;
   ShipStatus           m_ship_status;
   ShipStatus           m_advesary_ship_status;

   bool                 m_generate_path;
   bool                 m_deploy;
   uint                 m_look_ahead_steps;
   path::PathAlgorithm        m_path_algorithm;

   std::vector<std::string>  m_known_swimmers;

   XYPoint              m_park_point;
   XYPoint              m_skip_point;
   bool                 m_skip_next_point;

   // For UAV
  std::unique_ptr<Mavsdk> m_mavsdk_ptr;
  std::unique_ptr<MissionRaw> m_mission_raw_ptr;
  std::unique_ptr<Action> m_action_ptr;
  std::unique_ptr<Telemetry> m_telemetry_ptr;

   bool                m_do_fly_to_waypoint;
   bool                m_do_takoff;

  double m_lat_deg_home;
  double m_lon_deg_home;
  

 private: // debug vars

   int m_node_report_received = 0;
   int m_node_report_from_adversary_received = 0;



  
};

double calculateDistance(const XYPoint& point1, const XYPoint& point2);
double calculateETA(const ShipStatus& myShip, const XYPoint& point);


#endif 
