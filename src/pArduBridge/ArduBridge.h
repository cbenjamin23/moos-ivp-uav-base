/************************************************************/
/*    NAME: Steve Carter Feujo Nomeny                       */
/*    ORGN: NTNU, MIT                                       */
/*    FILE: ArduBridge.h                                    */
/*    DATE: September 9th, 2024                             */
/************************************************************/

#ifndef ArduBridge_HEADER
#define ArduBridge_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

#include "UAV_Model.h"
#include <cli_arg.h>



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


  private: // Configuration variables

    mavsdk::CliArg  m_cli_arg;

  private: // State variables
    // For UAV

    UAV_Model m_uav_model;

    bool  m_do_fly_to_waypoint;
    bool  m_do_takeoff;

    double m_lat_deg_home;
    double m_lon_deg_home;
};



#endif 

