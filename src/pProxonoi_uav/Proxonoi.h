/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: Proxonoi.h                                           */
/*    DATE: December 25th, 2019                                  */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#ifndef PROXONOI_HEADER
#define PROXONOI_HEADER

#include <string>
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"
#include "XYPolygon.h"
#include "XYSegList.h"
#include "NodeRecord.h"

class Proxonoi : public AppCastingMOOSApp
{
public:
  Proxonoi();
  ~Proxonoi() {};

protected: // Standard MOOSApp functions to overload
  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();

protected: // Standard AppCastingMOOSApp function to overload
  bool buildReport();

protected:
  void registerVariables();

  bool handleConfigOpRegion(std::string);
  bool handleStringSetPointMethod(std::string);

  void handleMailNodeReport(std::string);
  bool handleMailProxPolyView(std::string);
  bool handleMailProxClear();
  bool handleMailProxSetIgnoreList(std::string);
  // bool handleMailNodeMessage(std::string);

  bool updateSplitLines();
  bool updateVoronoiPoly();
  XYPoint calculateGridSearchSetpoint() const;

  XYPoint updateViewGridSearchSetpoint();
  void postCentroidSetpoint();

  void checkRemoveVehicleStaleness();

  void shareProxPolyArea();
  void shareProxPoly();


private: // Configuration variables
  std::string m_ownship;
  std::string m_vcolor;
  std::string m_setpt_method;

  double m_reject_range;
  bool m_post_poly;
  bool m_post_region;

  std::string m_contact_local_coords;
  bool m_use_geodesy;
  CMOOSGeodesy m_geodesy;

  std::string m_region_up_var;
  std::string m_ignore_list_up_var;

  double m_node_record_stale_treshold;

private: // State variables
  double m_osx;
  double m_osy;
  bool m_osx_tstamp;
  bool m_osy_tstamp;
  std::set<std::string> m_name_reject;
  std::set<std::string> m_name_always_reject;
  std::string m_last_posted_spec;
  int m_skip_count;

  bool m_os_in_prox_region;

  XYPolygon m_prox_region;
  XYPolygon m_prox_poly;

  bool m_poly_erase_pending;

  std::map<std::string, NodeRecord> m_map_node_records;
  std::map<std::string, XYSegList> m_map_split_lines;
  std::map<std::string, double> m_map_ranges;
};

#endif
