/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: Proxonoi.cpp                                         */
/*    DATE: December 25th, 2019                                  */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#include <iterator>
#include <unistd.h>
#include "MBUtils.h"
#include "ACTable.h"
#include "Proxonoi.h"
#include "VoronoiUtils.h"
#include "XYFormatUtilsPoly.h"
#include "NodeRecordUtils.h"
#include "NodeMessage.h"

#include <cmath>
#include "AngleUtils.h"
#include "GeomUtils.h"

#include "XYFormatUtilsConvexGrid.h"
#include "Logger.h"

//---------------------------------------------------------
// Constructor

Proxonoi::Proxonoi()
{
  // Config variables
  m_reject_range = 10000;
  m_contact_local_coords = "verbatim";
  m_use_geodesy = false;

  m_post_region = false;
  m_post_poly = false;

  m_region_up_var = "PROX_UP_REGION";
  m_ignore_list_up_var = "PROX_SET_IGNORE_LIST";

  // State Variables
  m_course = 0;
  m_osx = 0;
  m_osy = 0;
  m_osx_tstamp = false;
  m_osy_tstamp = false;
  m_poly_erase_pending = false;
  m_last_posted_spec = "";
  m_skip_count = 0;

  m_vcolor = "white";
  m_setpt_method = "center";

  m_node_record_stale_treshold = 10;
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool Proxonoi::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for (p = NewMail.begin(); p != NewMail.end(); p++)
  {
    CMOOSMsg &msg = *p;
    std::string key = msg.GetKey();
    double dval = msg.GetDouble();
    std::string sval = msg.GetString();
    double mtime = msg.GetTime();
    std::string comm = msg.GetCommunity();

#if 0 // Keep these around just for template
    std::string msrc  = msg.GetSource();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif

    bool handled = false;
    if (key == "NAV_X")
    {
      m_osx = dval;
      m_osx_tstamp = mtime;
      handled = true;
    }
    else if (key == "NAV_Y")
    {
      m_osy = dval;
      m_osy_tstamp = mtime;
      handled = true;
    }
    else if (key == "NAV_HEADING")
    {
      m_course = dval;
      handled = true;
    }
    else if (key == "PROX_CLEAR")
      handled = handleMailProxClear();
    else if (key == "NODE_REPORT")
    {
      handleMailNodeReport(sval);
      handled = true;
    }
    else if (key == "PROX_POLY_VIEW")
      handled = handleMailProxPolyView(sval);
    else if (key == m_ignore_list_up_var)
      handled = handleMailProxSetIgnoreList(sval);
    else if (key == m_region_up_var)
    {
      handleMailProxClear();
      handled = handleConfigOpRegion(sval);
    }
    else if (key == "PROX_SETPT_METHOD")
      handled = handleStringSetPointMethod(sval);
    else if (key == "VIEW_GRID")
      handled = handleMailViewGrid(sval);
    else if (key == "VIEW_GRID_DELTA")
      handled = handleMailViewGridUpdate(sval);

    else if (key != "APPCAST_REQ") // handled by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);

    if (!handled)
      reportRunWarning("Unhandled Mail: " + key);
  }

  return (true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool Proxonoi::OnConnectToServer()
{
  // registerVariables();
  return (true);
}

//---------------------------------------------------------
// Procedure: Iterate()

bool Proxonoi::Iterate()
{
  AppCastingMOOSApp::Iterate();

  //===========================================================
  // Part 1: Update the split line based on the information of
  // nearby contacts.
  //===========================================================
  updateSplitLines();

  //===========================================================
  // Part 2: Using the split lines, carve down the voronoi poly
  //===========================================================
  updateVoronoiPoly();

  checkRemoveVehicleStaleness();

  //===========================================================
  // Update the Setpoints
  //===========================================================

  if (m_prox_region.is_convex())
  {

    XYPoint center_reg = m_prox_region.get_center_pt();
    center_reg.set_label("center_reg");
    // center_reg.set_label_color("off");
    center_reg.set_color("vertex", "red");
    center_reg.set_vertex_size(10);
    Notify("VIEW_POINT", center_reg.get_spec());

    XYPoint centroid_reg = m_prox_region.get_centroid_pt();
    centroid_reg.set_label("centroid_reg");
    // centroid_reg.set_label_color("off");
    centroid_reg.set_color("vertex", "yellow");
    centroid_reg.set_vertex_size(10);
    Notify("VIEW_POINT", centroid_reg.get_spec());
  }

  auto setpt_grid = updateViewGridSearchSetpoint();

  XYPoint setpt;
  if (m_setpt_method == "gridsearch")
    setpt = setpt_grid;

  postGridSearchSetpointFiltered(setpt);
  
  postCentroidSetpoint();

  //===========================================================

  // Part 3.  Post it;
  if (m_prox_poly.is_convex())
  {
    std::string spec = m_prox_poly.get_spec();
    bool new_spec = (spec != m_last_posted_spec);

    if (m_post_poly && !m_poly_erase_pending && new_spec)
    {
      Notify("VIEW_POLYGON", spec);
      m_last_posted_spec = spec;
    }
    Notify("PROXONOI_POLY", spec);
  }
  else
  {
    m_prox_poly.set_active(false);
    std::string spec = m_prox_poly.get_spec();
    bool new_spec = (spec != m_last_posted_spec);
    if (m_post_poly && new_spec)
    {
      Notify("VIEW_POLYGON", spec);
      m_last_posted_spec = spec;
    }

    Notify("PROXONOI_POLY", spec);
    m_poly_erase_pending = false;
  }

  if (m_poly_erase_pending)
  {
    m_prox_poly.set_active(false);
    std::string spec = m_prox_poly.get_spec();
    Notify("VIEW_POLYGON", spec);
    m_poly_erase_pending = false;
  }

  if ((m_skip_count % 200) == 0)
  {
    if (m_prox_region.is_convex())
    {
      std::string spec = m_prox_region.get_spec();
      bool new_spec = (spec != m_last_posted_spec);
      if (m_post_region && new_spec)
      {
        Notify("VIEW_POLYGON", spec);
        m_last_posted_spec = spec;
      }
      Notify("PROXONOI_REGION", spec);
    }
    m_skip_count = 0;
  }
  else
    m_skip_count += 1;

  if ((m_iteration % 1000) == 0)
  {
    shareProxPolyArea();
    shareProxPoly();
  }

  AppCastingMOOSApp::PostReport();
  return (true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool Proxonoi::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  Notify("PROXONOI_PID", getpid());

  m_ownship = m_host_community;
  if (m_ownship == "")
  {
    std::cout << "Vehicle Name (MOOS community) not provided" << std::endl;
    return (false);
  }

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if (!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for (p = sParams.begin(); p != sParams.end(); p++)
  {
    std::string orig = *p;
    std::string line = *p;
    std::string param = tolower(biteStringX(line, '='));
    std::string value = line;

    bool handled = false;
    if (param == "region")
      handled = handleConfigOpRegion(value);
    else if (param == "post_poly")
      handled = setBooleanOnString(m_post_poly, value);
    else if (param == "post_region")
      handled = setBooleanOnString(m_post_region, value);
    else if ((param == "reject_range") && (value == "nolimit"))
    {
      m_reject_range = -1;
      handled = true;
    }
    else if (param == "ignore_name")
    {
      m_name_reject.insert(tolower(value));
      handled = true;
    }
    else if (param == "always_ignore_name")
    {
      m_name_always_reject.insert(tolower(value));
      m_name_reject.insert(tolower(value));
      handled = true;
    }
    else if (param == "reject_range")
      handled = setDoubleOnString(m_reject_range, value);
    else if (param == "setpt_method")
      handled = handleStringSetPointMethod(value);
    else if (param == "vehicle_stale_treshold")
      handled = setDoubleOnString(m_node_record_stale_treshold, value);
    else if (param == "region_update_var")
    {
      if (value != "")
      {
        m_region_up_var = toupper(value);
        handled = true;
      }
    }
    else if (param == "ignore_list_update_var")
    {
      if (value != "")
      {
        m_ignore_list_up_var = toupper(value);
        handled = true;
      }
    }
    else if (param == "vcolor")
    {
      m_vcolor = value;
      handled = true;
    }

    if (!handled)
      reportUnhandledConfigWarning(orig);
  }

  registerVariables();
  return (true);
}

//---------------------------------------------------------
// Procedure: registerVariables

void Proxonoi::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NODE_REPORT", 0);
  Register("PROX_POLY_VIEW", 0);
  Register("PROX_CLEAR", 0);
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
  Register("NAV_HEADING", 0);
  Register(m_ignore_list_up_var, 0);
  Register(m_region_up_var, 0);

  Register("PROX_SETPT_METHOD", 0);

  // From GCS
  Register("VIEW_GRID", 0);
  Register("VIEW_GRID_DELTA", 0);
}
bool Proxonoi::handleMailViewGrid(std::string str)
{
  str = stripBlankEnds(str);
  auto grid = string2ConvexGrid(str);
  if (!grid.valid())
  {
    reportRunWarning("Received invalid grid: " + str);
    Logger::warning("Received invalid grid: " + str);
    return false;
  }
  m_convex_region_grid = grid;

  return true;
}

bool Proxonoi::handleMailViewGridUpdate(std::string str)
{
  m_convex_region_grid.processDelta(str);
  return true;
}

//---------------------------------------------------------
// Procedure: handleConfigOpRegion

bool Proxonoi::handleConfigOpRegion(std::string opstr)
{
  XYPolygon op_region = string2Poly(opstr);
  op_region.set_label("prox_opregion");

  if (!op_region.is_convex())
    return (false);

  // If all good, then clear the states
  handleMailProxClear();

  m_prox_region = op_region;
  return (true);
}

//---------------------------------------------------------
// Procedure: handleStringSetPointMethod

bool Proxonoi::handleStringSetPointMethod(std::string method)
{
  method = tolower(method);
  if ((method == "gridsearch") ||
      (method == "centroid") ||
      (method == "center"))
    m_setpt_method = method;
  else
    return (false);

  return (true);
}

//---------------------------------------------------------
// Procedure: handleMailNodeReport
//   Example: NAME=alpha,TYPE=KAYAK,UTC_TIME=1267294386.51,
//            X=29.66,Y=-23.49, LAT=43.825089,LON=-70.330030,
//            SPD=2.00,HDG=119.06,YAW=119.05677,DEPTH=0.00,
//            LENGTH=4.0,MODE=DRIVE

void Proxonoi::handleMailNodeReport(std::string report)
{
  NodeRecord new_node_record = string2NodeRecord(report);

  // Part 1: Decide if we want to override X/Y with Lat/Lon based on
  // user configuration and state of the node record.
  bool override_xy_with_latlon = true;
  if (m_contact_local_coords == "verbatim")
    override_xy_with_latlon = false;
  if (!m_use_geodesy)
    override_xy_with_latlon = false;
  if (m_contact_local_coords == "lazy_lat_lon")
  {
    if (new_node_record.isSetX() && new_node_record.isSetY())
      override_xy_with_latlon = false;
  }

  if (!new_node_record.isSetLatitude() || !new_node_record.isSetLongitude())
    override_xy_with_latlon = false;

  // Part 2: If we can override xy with latlon and configured to do so
  // then find the X/Y from MOOSGeodesy and Lat/Lon and replace.
  if (override_xy_with_latlon)
  {
    double nav_x, nav_y;
    double lat = new_node_record.getLat();
    double lon = new_node_record.getLon();

#ifdef USE_UTM
    m_geodesy.LatLong2LocalUTM(lat, lon, nav_y, nav_x);
#else
    m_geodesy.LatLong2LocalGrid(lat, lon, nav_y, nav_x);
#endif
    new_node_record.setX(nav_x);
    new_node_record.setY(nav_y);
  }

  std::string vname = new_node_record.getName();

  // If incoming node name matches ownship, just ignore the node report
  // but don't return false which would indicate an error.

  if (vname == m_ownship || (m_name_reject.count(tolower(vname)) > 0))
    return;

  bool newly_known_vehicle = false;
  if (m_map_node_records.count(vname) == 0)
    newly_known_vehicle = true;

  // If we are (a) not currently tracking the given vehicle, and (b)
  // a reject_range is enabled, and (c) the contact is outside the
  // reject_range, then ignore this contact.
  double cnx = new_node_record.getX();
  double cny = new_node_record.getY();
  double range = hypot(m_osx - cnx, m_osy - cny);

  if (newly_known_vehicle && (m_reject_range > 0) && (range > m_reject_range))
    return;

  //  if(!new_node_record.valid()) {
  //  Notify("PROXONOI_WARNING", "Bad Node Report Received");
  //  reportRunWarning("Bad Node Report Received");
  //  return;
  //}

  m_map_node_records[vname] = new_node_record;
  m_map_ranges[vname] = range;
}

//---------------------------------------------------------
// Procedure: handleMailProxPolyView()

bool Proxonoi::handleMailProxPolyView(std::string msg)
{
  msg = tolower(stripBlankEnds(msg));
  if (msg == "toggle")
    m_post_poly = !m_post_poly;
  else if ((msg == "false") || (msg == "off"))
    m_post_poly = false;
  else if ((msg == "true") || (msg == "on"))
    m_post_poly = true;
  else
    return (false);

  if (m_post_poly == false)
    m_poly_erase_pending = true;

  return (true);
}

//---------------------------------------------------------
// Procedure: handleMailProxClear()

bool Proxonoi::handleMailProxClear()
{
  // ========================================================
  // Part 1: Reset the prox poly to the entire region. But we
  // want to retain the label of the prox_poly on the new one
  // ========================================================
  std::string prox_label = m_prox_poly.get_label();
  m_prox_poly = m_prox_region;
  m_prox_poly.set_label(prox_label);

  m_prox_poly.set_label("vpoly_" + m_ownship);

  // ========================================================
  // Part 2: Reset all the data structures
  // ========================================================

  m_map_node_records.clear();
  m_map_split_lines.clear();
  m_map_ranges.clear();

  // ========================================================
  // Part 3: Mark the prox poly as needing to be erased
  // ========================================================
  m_poly_erase_pending = true;

  return (true);
}

//------------------------------------------------------------
// Procedure: updateSplitLines()

bool Proxonoi::updateSplitLines()
{
  std::map<std::string, NodeRecord>::iterator p;
  for (p = m_map_node_records.begin(); p != m_map_node_records.end(); p++)
  {
    std::string vname = p->first;
    NodeRecord record = p->second;
    double cnx = record.getX();
    double cny = record.getY();

    // If the contact is in the op_region, then create a split line
    // otherwise the splitline associated with the contaxt is null
    XYSegList segl;
    if (m_prox_region.contains(cnx, cny))
    {
      double sx1, sy1, sx2, sy2;
      vsplit(m_osx, m_osy, cnx, cny, sx1, sy1, sx2, sy2);
      segl.add_vertex(sx1, sy1);
      segl.add_vertex(sx2, sy2);
    }
    m_map_split_lines[vname] = segl;
  }
  return (true);
}

//------------------------------------------------------------
// Procedure: shareProxPolyArea()

void Proxonoi::shareProxPolyArea()
{
  if (!m_prox_poly.valid())
    return;

  NodeMessage msg(m_ownship, "all", "PROX_POLY_AREA");
  msg.setDoubleVal(m_prox_poly.area());
  msg.setColor("off");

  Notify("NODE_MESSAGE_LOCAL", msg.getSpec());
}

//------------------------------------------------------------
// Procedure: shareProxPoly()

void Proxonoi::shareProxPoly()
{
  if (!m_prox_poly.valid())
    return;

  NodeMessage msg(m_ownship, "all", "PROX_POLY_NEIGHBOR");

  Logger::info("Sharing Prox Poly (3spec): " + m_prox_poly.get_spec(3));

  msg.setStringVal(m_prox_poly.get_spec(3));
  msg.setColor("off");

  Notify("NODE_MESSAGE_LOCAL", msg.getSpec(3));
}

//------------------------------------------------------------
// Procedure: updateVoronoiPoly()

bool Proxonoi::updateVoronoiPoly()
{
  m_prox_poly = XYPolygon(); // Init to a null poly
                             // m_prox_poly.add_vertex(0,0);
  // m_prox_poly.add_vertex(1,0);
  // m_prox_poly.add_vertex(0,1);
  m_prox_poly.set_label("vpoly_" + m_ownship);

  // Sanity check 1: if op_region is null return false
  if (!m_prox_region.is_convex())
    return (false);

  // Sanity check 2: if no ownship position return false
  if (!m_osx_tstamp || !m_osy_tstamp)
    return (false);

  // Sanity check 3: if ownship not in op_region, return false
  if (!m_prox_region.contains(m_osx, m_osy))
  {
    if (m_os_in_prox_region)
      m_poly_erase_pending = true;

    m_os_in_prox_region = false;
    return (false);
  }
  else
    m_os_in_prox_region = true;

  // Passed sanity checks, init voronoi poly to entire op_region
  m_prox_poly = m_prox_region;
  m_prox_poly.set_label("vpoly_" + m_ownship);

  // Special case: if no contact info, vornoi poly is the
  // entire op_region and we return true
  if (m_map_node_records.size() == 0)
    return (true);

  // Proceed with building the voronoi poly
  std::map<std::string, XYSegList>::iterator p;
  for (p = m_map_split_lines.begin(); p != m_map_split_lines.end(); p++)
  {
    std::string vname = p->first;
    XYSegList segl = p->second;
    m_prox_poly = polychop(m_prox_poly, m_osx, m_osy, segl);
  }

  // Possibly combine very close vertices. In this case, vertices
  // within one meter of each other.
  bool can_simplify = true;
  while (can_simplify)
    can_simplify = m_prox_poly.simplify(1);

  // Set the viewable components of the prox poly
  m_prox_poly.set_label("vpoly_" + m_ownship);
  m_prox_poly.set_color("edge", "white");
  m_prox_poly.set_color("vertex", "blue");
  m_prox_poly.set_color("fill", "pink");
  m_prox_poly.set_transparency(0.15);

  return (true);
}

//-----------------------------------------------------
// Procedure: handleMailProxSetIgnoreList(std::string);
bool Proxonoi::handleMailProxSetIgnoreList(std::string msg)
{

  // First clear the states
  handleMailProxClear();

  // Make a new set which will replace the old set.
  std::set<std::string> new_name_reject;
  // parse message
  std::vector<std::string> svector = parseString(msg, ',');
  for (unsigned int i = 0; i < svector.size(); i++)
  {
    new_name_reject.insert(svector[i]);
  }

  // Add in the names we allways ignore
  std::set<std::string>::iterator it;
  for (it = m_name_always_reject.begin(); it != m_name_always_reject.end(); it++)
  {
    new_name_reject.insert(*it);
  }

  m_name_reject = new_name_reject;

  return (true);
}

//------------------------------------------------------------
// Procedure: buildReport()

bool Proxonoi::buildReport()
{
  //=================================================================
  // Part 1: Header Content
  //=================================================================
  std::string reject_range = "off";
  if (m_reject_range > 0)
    reject_range = doubleToStringX(m_reject_range, 2);

  std::string in_region = boolToString(m_os_in_prox_region);
  std::string erase_pending = boolToString(m_poly_erase_pending);

  m_msgs << "Reject Range:   " << reject_range << std::endl;
  m_msgs << "In Prox Region: " << in_region << std::endl;
  m_msgs << "Erase Pending:  " << erase_pending << std::endl;
  m_msgs << "Vehicle Treshold: " << m_node_record_stale_treshold << std::endl;
  m_msgs << "\n";

  double area = m_prox_poly.area();
  if (area > 10000)
    area /= 1000;

  m_msgs << "Ownship Area:       " << doubleToStringX(area, 0) << std::endl;
  m_msgs << "Ownship Position:   (" << m_osx << ", " << m_osy << ")" << std::endl;
  m_msgs << "Setpoint Method:   " << m_setpt_method << std::endl;

  m_msgs << "\n\n";
  //=================================================================
  // Part 4: Contact Status Summary
  //=================================================================
  m_msgs << "Contact Status Summary:" << std::endl;
  m_msgs << "-----------------------" << std::endl;

  ACTable actab(3, 2);
  actab.setColumnJustify(1, "right");
  actab.setColumnJustify(2, "right");
  actab << "Contact | Range | TimeSinceRec";
  actab.addHeaderLines();

  std::map<std::string, NodeRecord>::iterator q;
  for (q = m_map_node_records.begin(); q != m_map_node_records.end(); q++)
  {
    std::string vname = q->first;
    std::string range = doubleToString(m_map_ranges[vname], 1);

    std::string time_str = "-";

    double time_to_treshold = (MOOSTime() - q->second.getTimeStamp());
    if (time_to_treshold > m_node_record_stale_treshold)
      time_str = "stale";
    else
      time_str = doubleToStringX(time_to_treshold, 1);

    actab << vname << range << time_str;
  }
  m_msgs << actab.getFormattedString();

  return (true);
}

XYPoint Proxonoi::updateViewGridSearchSetpoint()
{
  XYPoint nullpt;
  XYPoint gridSearchSetPt = calculateGridSearchSetpoint();

  if (!gridSearchSetPt.valid())
  {
    Logger::warning("GridSearch Setpoint not valid");
    reportRunWarning("GridSearch Setpoint not valid");
    gridSearchSetPt = calculateCircularSetPt();
  }
  else
    retractRunWarning("GridSearch Setpoint not valid");

  // std::string label = "gridSearchSetPt_" + m_ownship;
  std::string label = "g_" + m_ownship;
  gridSearchSetPt.set_label(label);
  // gridSearchSetPt.set_label_color("off");
  gridSearchSetPt.set_color("vertex", m_vcolor);
  gridSearchSetPt.set_vertex_size(10);
  // gridSearchSetPt.set_duration(5);
  Notify("VIEW_POINT", gridSearchSetPt.get_spec());

  return gridSearchSetPt;
}

//------------------------------------------------------------
void Proxonoi::postCentroidSetpoint()
{

  XYPoint centroidSetPt = m_prox_poly.get_centroid_pt();
  if (!centroidSetPt.valid())
  {
    Logger::warning("Centroid Setpoint not valid");
    return;
  }

  double area = m_prox_poly.area();
  if (area > 10000)
    area /= 1000;
  unsigned int uint_area = (unsigned int)(area);

  std::string label = "centroid_" + m_ownship + " (" + uintToString(uint_area) + ")";
  centroidSetPt.set_label("centroidSetPt_" + m_ownship);
  centroidSetPt.set_label_color("white");
  centroidSetPt.set_msg(label);
  centroidSetPt.set_color("vertex", "white");
  centroidSetPt.set_vertex_size(10);
  // centroidSetPt.set_duration(5);
  Notify("VIEW_POINT", centroidSetPt.get_spec());

  auto center = m_prox_poly.get_center_pt();
  if (center.valid())
  {
    center.set_label("center");
    center.set_color("vertex", "white");
    center.set_vertex_size(4);
    Notify("VIEW_POINT", center.get_spec());
  }
}

void Proxonoi::checkRemoveVehicleStaleness()
{

  double curr_time = MOOSTime();

  for (auto p = m_map_node_records.begin(); p != m_map_node_records.end(); p++)
  {
    std::string vname = p->first;
    double time_received = p->second.getTimeStamp();

    if (vname == m_ownship)
      continue;

    auto timediff = (curr_time - time_received);
    // Logger::info("Checking Poly Staleness: " + vname + " " + doubleToStringX(timediff, 1));

    // Check if the time received is older than the threshold
    if ((curr_time - time_received) < m_node_record_stale_treshold)
      continue;

    m_map_ranges.erase(vname);
    m_map_split_lines.erase(vname);
    m_map_node_records.erase(vname);
    Logger::info("Checking Poly Staleness: Erased " + vname + " time: " + doubleToStringX(time_received, 2) + " curr_time: " + doubleToStringX(curr_time, 2) +
                 " treshold: " + doubleToStringX(m_node_record_stale_treshold, 2) +
                 " diff: " + doubleToStringX(timediff, 2));
  }
}

XYPoint Proxonoi::calculateGridSearchSetpoint()
{
  XYPoint null_pt;

  if (!m_prox_region.valid())
    return (null_pt);
  if (!m_prox_poly.valid())
    return (null_pt);

  //--------------------------------------------------------------------------------------------
  //--------------------------------------------------------------------------------------------
  auto [forward_center, forward_weight] = calculateSearchCenter(m_prox_poly, m_convex_region_grid, -20, 20);
  auto [left_center, left_weight] = calculateSearchCenter(m_prox_poly, m_convex_region_grid, -90, -20);
  auto [right_center, right_weight] = calculateSearchCenter(m_prox_poly, m_convex_region_grid, 20, 90);

  bool forward_free = !isPointInDiscoverdGridCell(forward_center);
  bool left_free = !isPointInDiscoverdGridCell(left_center);
  bool right_free = !isPointInDiscoverdGridCell(right_center);

  forward_center.set_label("f" + m_ownship);
  forward_center.set_color("vertex", "yellow");
  forward_center.set_vertex_size(5);
  forward_center.set_msg("f");
  Notify("VIEW_POINT", forward_center.get_spec());

  left_center.set_label("l" + m_ownship);
  left_center.set_color("vertex", "green");
  left_center.set_vertex_size(5);
  left_center.set_msg("l");
  Notify("VIEW_POINT", left_center.get_spec());

  right_center.set_label("r" + m_ownship);
  right_center.set_color("vertex", "red");
  right_center.set_msg("r");
  right_center.set_vertex_size(5);
  Notify("VIEW_POINT", right_center.get_spec());

  static std::string prev_sector = "forward";
  double threshold = 1.4; // 40% hysteresis
  XYPoint searchCenter;

  if (prev_sector == "forward")
  {

    if ((left_weight > forward_weight * threshold  && left_free)|| (left_free && !forward_free))
    {
      searchCenter = left_center;
      prev_sector = "left";
    }
    else if ((right_weight > forward_weight * threshold && right_free) || (right_free && !forward_free))
    {
      searchCenter = right_center;
      prev_sector = "right";
    }
    else
    {
      searchCenter = forward_center;
    }
  }
  else if (prev_sector == "left")
  {
    if ((forward_weight > left_weight * threshold && forward_free) || (forward_free && !left_free) )
    {
      searchCenter = forward_center;
      prev_sector = "forward";
    }
    else if ((right_weight > left_weight * threshold && right_free)|| (right_free && !left_free))
    {
      searchCenter = right_center;
      prev_sector = "right";
    }
    else
    {
      searchCenter = left_center;
    }
  }
  else
  { // right
    if ((forward_weight > right_weight * threshold && forward_free) || (forward_free && !right_free))
    {
      searchCenter = forward_center;
      prev_sector = "forward";
    }
    else if ((left_weight > right_weight * threshold && left_free) || (left_free && !right_free) )
    {
      searchCenter = left_center;
      prev_sector = "left";
    }
    else
    {
      searchCenter = right_center;
    }
  }

  if (!searchCenter.valid())
    return null_pt;

  searchCenter.set_label("searcCenter" + m_ownship);
  searchCenter.set_label_color("off");
  searchCenter.set_color("vertex", "blue");
  searchCenter.set_vertex_size(8);
  searchCenter.set_msg("searchCenter");
  Notify("VIEW_POINT", searchCenter.get_spec());

  //--------------------------------------------------------------------------------------------
  //--------------------------------------------------------------------------------------------

  XYPoint cpt{m_osx, m_osy};

  // Smooth the searchCenter to avoid abrupt jumps (optional)
  static XYPoint prev_searchCenter = searchCenter;
  if (prev_searchCenter.valid())
  {
    double alpha = 0.3; // Smoothing factor (0 to 1)
    double new_x = alpha * searchCenter.get_vx() + (1 - alpha) * prev_searchCenter.get_vx();
    double new_y = alpha * searchCenter.get_vy() + (1 - alpha) * prev_searchCenter.get_vy();
    searchCenter.set_vx(new_x);
    searchCenter.set_vy(new_y);
  }
  prev_searchCenter = searchCenter;

  auto ref_pt = searchCenter;

  // Logger::info("Relative angle calculated: " + doubleToString(rel_ang, 2));

  static XYPoint target_pt;
  XYPoint circular_point = calculateCircularSetPt();

  // double dist = distPointToPoint(circular_point, ref_pt);
  double distance_from_circle_point = distPointToPoint(circular_point, ref_pt);
  auto heading_from_circle_point = relAng(circular_point, ref_pt);
  XYPoint final_pt = projectPoint(heading_from_circle_point, distance_from_circle_point, circular_point);

  double dist_to_target = 0;
  if (target_pt.valid())
    dist_to_target = distPointToPoint(cpt, target_pt);
  else
    dist_to_target = distPointToPoint(cpt, final_pt);

  // Logger::info("Distance to target calculated: " + doubleToString(dist_to_target, 2));
  // create a function that goes to 1000 as dist goes to 30
  // double mag = 1000 * (1 - (dist / 30));
  double mag = 150 * (1 - (dist_to_target / 150));
  mag = std::max(0.0, std::min(mag, 200.0));

  // final_pt = projectPoint(heading, mag + default_dist, ref_pt);
  final_pt = projectPoint(heading_from_circle_point, mag, final_pt);
  // Logger::info("Final point calculated: " + final_pt.get_spec());

  if (!m_prox_poly.contains(final_pt))
  {
    final_pt = m_prox_poly.closest_point_on_poly(final_pt);
    Logger::warning("Calculated weighted center is outside the polygon");
  }

  target_pt = final_pt;
  return (final_pt);
}

std::pair<XYPoint, double> Proxonoi::calculateSearchCenter(const XYPolygon &pol, const XYConvexGrid &grid, double min_signed_diff, double max_signed_diff) const
{

  std::pair<XYPoint, double> null_pair;
  if (!pol.valid() || !grid.size())
  {
    Logger::error("Invalid polygon or empty grid");
    return null_pair;
  }
  double max_visits = grid.getMaxLimit();
  if (max_visits == 0)
  {
    Logger::warning("Max visits is zero, cannot calculate weighted center");
    return null_pair;
  }

  double total_x = 0, total_y = 0, total_weight = 0;
  // XYPoint drone_pos(m_osx, m_osy);
  // XYPoint drone_heading = m_course;

  XYPoint reg_centroid = m_prox_region.get_centroid_pt();
  XYPoint poly_centroid = m_prox_poly.get_centroid_pt();

  double centroid_heading = relAng(reg_centroid, poly_centroid) - 90;

  for (unsigned int i = 0; i < grid.size(); ++i)
  {
    XYSquare cell = grid.getElement(i);
    if (!pol.contains(cell.getCenterX(), cell.getCenterY()))
      continue;

    double cell_visits = grid.getVal(i);
    double weight = max_visits - cell_visits; // Higher weight for fewer visits
    // Skip cells that have been heavily visited
    if (weight <= 0 || cell_visits > 0)
      continue;

    XYPoint cell_center(cell.getCenterX(), cell.getCenterY());
    double cell_angle = relAng(poly_centroid, cell_center);
    double signed_diff = signedAngleDiff(centroid_heading, cell_angle);
    if (signed_diff >= min_signed_diff && signed_diff <= max_signed_diff)
    {
      double weight = 1.0 / (cell_visits + 1);
      total_x += cell.getCenterX() * weight;
      total_y += cell.getCenterY() * weight;
      total_weight += weight;
    }
  }

  XYPoint pt{total_x / total_weight, total_y / total_weight};

  if (total_weight > 0)
   return {pt, total_weight};
  
  return {null_pair}; // Default if no cells in sector
}

XYPoint Proxonoi::calculateCircularSetPt()
{
  static XYPoint prev_setpt;

  XYPoint reg_centroid = m_prox_region.get_centroid_pt();
  XYPoint poly_centroid = m_prox_poly.get_centroid_pt();
  double rel_ang = relAng(reg_centroid, poly_centroid);

  double circular_heading = rel_ang - 90;
  double default_dist = 150;

  // XYPoint final_pt = projectPoint(heading, default_dist, ref_pt);
  XYPoint circular_point = projectPoint(circular_heading, default_dist, poly_centroid);

  if (circular_point.get_vx() != prev_setpt.get_vx() || circular_point.get_vy() != prev_setpt.get_vy())
  {
    circular_point.set_label("cp" + m_ownship);
    circular_point.set_color("vertex", m_vcolor);
    circular_point.set_vertex_size(8);
    circular_point.set_msg("cp");
    Notify("VIEW_POINT", circular_point.get_spec());
  }

  prev_setpt = circular_point;
  return circular_point;
}

bool Proxonoi::postGridSearchSetpointFiltered(XYPoint pt) 
{

  static XYPoint prev_setpt;
  constexpr double sep_radius = 25;

  if (!pt.valid())
    return false;

  double dist = distPointToPoint(pt, prev_setpt);
  if (dist <= sep_radius)
    return false;

  if (isPointInDiscoverdGridCell(pt))
    return false;

  prev_setpt = pt;


  Notify("PROX_SEARCHCENTER", pt.get_spec());


  return true;
}

bool Proxonoi::isPointInDiscoverdGridCell(XYPoint pt) const
{

  if (!m_convex_region_grid.valid())
    return false;

  for (unsigned int i = 0; i < m_convex_region_grid.size(); ++i)
  {
    XYSquare cell = m_convex_region_grid.getElement(i);
    if (!cell.containsPoint(pt.get_vx(), pt.get_vy()))
      continue;

    if (m_convex_region_grid.getVal(i) > 0)
      return true;
  }

  return false;
}

//------------------------------------------------------------
// UTILITY FUNCTIONS
//-------------------------------------------------------------
double signedAngleDiff(double angle1, double angle2)
{
  double diff = fmod(angle2 - angle1 + 360, 360);
  if (diff > 180)
    diff -= 360;
  return diff;
}
