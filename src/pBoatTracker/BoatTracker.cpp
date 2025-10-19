/************************************************************/
/*    NAME:                                               */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: BoatTracker.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "BoatTracker.h"

#include "XYPoint.h"
#include "XYFormatUtilsPoint.h"
#include "NodeRecord.h"              
#include "NodeRecordUtils.h"
#include "GeomUtils.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

BoatTracker::BoatTracker()
{

  m_got_here = false;
  //m_boat_point = (0,0);
  //m_plane_point = (0,0);

}

//---------------------------------------------------------
// Destructor

BoatTracker::~BoatTracker()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool BoatTracker::OnNewMail(MOOSMSG_LIST &NewMail)
{

  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();

    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 

#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif

     if(key == "NODE_REPORT") {
       //set values 
        m_got_here = true;
       NodeRecord record = string2NodeRecord(sval);

       if (comm == "formula2Boat") {
        
        // Extract X/Y coordinates from the report
        m_boat_x = record.getX();
        m_boat_y = record.getY();

        m_boat_point.set_vx(m_boat_x);
        m_boat_point.set_vy(m_boat_y);
      }

      else if (comm == "t1_ranger") {

        // Extract X/Y coordinates from the report
        m_plane_x = record.getX();
        m_plane_y = record.getY();

        m_plane_point.set_vx(m_plane_x);
        m_plane_point.set_vy(m_plane_y);
      }

       //update refresh
       m_refresh = true;
     }

     else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool BoatTracker::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool BoatTracker::Iterate()
{
  AppCastingMOOSApp::Iterate();
  // Do your thing here!

  if (m_refresh == true) {
    m_margin = distPointToPoint(m_boat_point, m_plane_point);

    if (m_margin >= 10) {
    // Format a BHV_Waypoint update string

    string update_str = "points = ";
    update_str += doubleToString(m_boat_x);
    update_str += ",";
    update_str += doubleToString(m_boat_y); 

    m_update_msg = update_str;
      // Post it to the updates variable your BHV listens to
      Notify("TOWAYPT_UPDATE", m_update_msg); 
    }
    m_refresh = false;
  }
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool BoatTracker::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    if(param == "foo") {
      handled = true;
    }
    else if(param == "bar") {
      handled = true;
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);

  }
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void BoatTracker::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  // Register("FOOBAR", 0);
  Register("NODE_REPORT", 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool BoatTracker::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "m_boat_x = " << m_boat_x << endl;
  m_msgs << "m_boat_y = " << m_boat_y << endl;
  m_msgs << "m_update_msg = " << m_update_msg << endl;
  m_msgs << "m_got_here = " << boolToString(m_got_here) << endl;
  m_msgs << "m_plane_x = " << m_plane_x << endl;
  m_msgs << "m_plane_y = " << m_plane_y << endl;
  m_msgs << "============================================" << endl;
  
  ACTable actab(4);
  actab << "Alpha | Bravo | Charlie | Delta";
  actab.addHeaderLines();
  actab << "one" << "two" << "three" << "four";
  m_msgs << actab.getFormattedString();

  return(true);
}




