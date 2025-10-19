/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: BoatTracker.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef BoatTracker_HEADER
#define BoatTracker_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "XYPoint.h"
#include "XYFormatUtilsPoint.h"
#include "NodeRecord.h"              
#include "NodeRecordUtils.h"
#include "GeomUtils.h"

class BoatTracker : public AppCastingMOOSApp
{
 public:
   BoatTracker();
   ~BoatTracker();

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

 private: // State variables

 bool m_got_here;
 double m_boat_x;
  double m_boat_y;
  XYPoint m_boat_point;
  double m_plane_x;
  double m_plane_y;
  XYPoint m_plane_point;
  bool m_refresh;
  std::string m_update_msg;
  double m_margin;
};

#endif 
