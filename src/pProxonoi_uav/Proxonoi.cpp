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
#include "NodeMessageUtils.h"

#include <cmath>
#include "AngleUtils.h"
#include "GeomUtils.h"
#include "Logger.h"

XYPolygon stringMod2Poly(std::string str);

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
  m_osx = 0;
  m_osy = 0;
  m_osx_tstamp = 0;
  m_osy_tstamp = 0;
  m_poly_erase_pending = false;
  m_last_posted_spec = "";
  m_skip_count = 0;

  m_vcolor = "white";

  m_neighbor_stale_treshold = 10; // 10s
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
    else if (key == "NODE_MESSAGE")
      handled = handleMailNodeMessage(sval);

    // else if (key == "PROX_POLY_AREA")
    // {
    //   m_map_vAreas[comm] = dval;
    //   m_map_neighbour_time_received[comm] = m_curr_time;
    //   handled = true;

    // }
    // else if (key == "PROX_POLY_NEIGHBOR")
    // {
    //   auto poly = string2Poly(sval);
    //   handled = poly.valid();
    //   if (handled)
    //   {
    //     m_map_vPolys[comm] = poly;
    //     m_map_neighbour_time_received[comm] = m_curr_time;
    //   }
    // }

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

  //===========================================================
  // Update the AreaBalanceSetpoint
  //===========================================================

  checkPolyStaleness();

  updatePostAreaBalanceSetpoint();

  postCentroidSetpoint();

  //===========================================================

  // Part 3.  Post it;
  if (m_prox_poly.is_convex())
  {
    std::string spec = m_prox_poly.get_spec(3);
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

  if ((m_iteration % 20) == 0)
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
    else if (param == "neighbor_stale_treshold")
      handled = setNonNegDoubleOnString(m_neighbor_stale_treshold, value);

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
  Register(m_ignore_list_up_var, 0);
  Register(m_region_up_var, 0);

  Register("NODE_MESSAGE", 0);
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
// Procedure: handleMailNodeMessage

bool Proxonoi::handleMailNodeMessage(std::string msg)
{
  NodeMessage node_msg = string2NodeMessage(msg);
  std::string vname = node_msg.getSourceNode();
  if (vname == m_ownship)
    return (true);

  // If the message is from a vehicle we are ignoring, just ignore it.
  if (m_name_reject.count(tolower(vname)) > 0)
    return (true);

  // If the message is from a vehicle we are always ignoring, just ignore it.
  if (m_name_always_reject.count(tolower(vname)) > 0)
    return (true);

  std::string msg_var = node_msg.getVarName();

  if (msg_var == "PROX_POLY_AREA")
  {
    m_map_vAreas[vname] = node_msg.getDoubleVal();
    m_map_neighbour_time_received[vname] = m_curr_time;
  }
  else if (msg_var == "PROX_POLY_NEIGHBOR")
  {

    auto poly = stringMod2Poly(node_msg.getStringVal());
    if (poly.is_convex())
    {
      m_map_vPolys[vname] = poly;
      m_map_neighbour_time_received[vname] = m_curr_time;
      Logger::info("Received valid poly from " + vname);
    }
    else
    {
      Logger::error("Received invalid poly from " + vname + ": " + node_msg.getStringVal());
      reportRunWarning("Bad Poly Received");
    }
  }

  Notify("NODE_MESSAGE", msg);
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
  if (newly_known_vehicle && (m_reject_range > 0))
  {
    double cnx = new_node_record.getX();
    double cny = new_node_record.getY();
    double range = hypot(m_osx - cnx, m_osy - cny);
    if (range > m_reject_range)
      return;
  }

  //  if(!new_node_record.valid()) {
  //  Notify("PROXONOI_WARNING", "Bad Node Report Received");
  //  reportRunWarning("Bad Node Report Received");
  //  return;
  //}

  m_map_node_records[vname] = new_node_record;
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

  m_map_vPolys.clear();
  m_map_vAreas.clear();

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

    double range = hypot((m_osx - cnx), (m_osy - cny));
    m_map_ranges[vname] = range;

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

  NodeMessage msg(m_ownship, "ALL", "PROX_POLY_AREA");
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

  NodeMessage msg(m_ownship, "ALL", "PROX_POLY_NEIGHBOR");

  Logger::info("Sharing Prox Poly: " + m_prox_poly.get_spec());

  msg.setStringVal(m_prox_poly.get_spec());
  msg.setColor("off");

  Notify("NODE_MESSAGE_LOCAL", msg.getSpec());
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
  if ((m_osx_tstamp == 0) || (m_osy_tstamp == 0))
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

  m_msgs << "Treshold Value: " << m_neighbor_stale_treshold << std::endl;

  //=================================================================
  // Part 4: Contact Status Summary
  //=================================================================
  m_msgs << "Contact Status Summary:" << std::endl;
  m_msgs << "-----------------------" << std::endl;

  ACTable actab(5, 2);
  actab.setColumnJustify(1, "right");
  actab.setColumnJustify(2, "right");
  actab.setColumnJustify(3, "right");
  actab.setColumnJustify(4, "right");
  actab << "Contact | Range | Area | TimeSinceRec | ValidPoly";
  actab.addHeaderLines();

  std::map<std::string, NodeRecord>::iterator q;
  for (q = m_map_node_records.begin(); q != m_map_node_records.end(); q++)
  {
    std::string vname = q->first;
    std::string range = doubleToString(m_map_ranges[vname], 1);

    std::string area_str = "-";
    std::string time_str = "-";
    std::string valid_poly_str = "-";

    if (m_map_vPolys.count(vname) > 0)
    {
      bool valid_poly = m_map_vPolys.at(vname).is_convex();
      valid_poly_str = boolToString(valid_poly);
    }
    if (m_map_vAreas.count(vname) > 0)
    {
      double area = m_map_vAreas.at(vname);
      if (area > 10000)
        area /= 1000;
      area_str = doubleToStringX(area, 0);
    }

    if (m_map_neighbour_time_received.count(vname) > 0)
    {
      double time_to_treshold = (MOOSTime() - m_map_neighbour_time_received.at(vname));
      time_str = doubleToStringX(time_to_treshold, 1);
    }

    actab << vname << range << area_str << time_str << valid_poly_str;
  }
  m_msgs << actab.getFormattedString();

  return (true);
}

bool Proxonoi::updatePostAreaBalanceSetpoint()
{

  XYPoint areaSetPt = getSetPtAreaBalance();

  if (!areaSetPt.valid())
  {
    Logger::warning("Area Setpoint not valid");
    return false;
  }

  std::string label = "areaSetPt_" + m_ownship;
  areaSetPt.set_label(label);
  areaSetPt.set_label_color("off");
  areaSetPt.set_color("vertex", "magenta");
  areaSetPt.set_vertex_size(12);
  areaSetPt.set_duration(5);
  Notify("VIEW_POINT", areaSetPt.get_spec());

  Notify("PROX_AREA_SETPT", areaSetPt.get_spec());

  return true;
}

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
}

void Proxonoi::checkPolyStaleness()
{

  double curr_time = MOOSTime();

  for (auto p = m_map_neighbour_time_received.begin(); p != m_map_neighbour_time_received.end(); p++)
  {
    std::string key = p->first;
    double time_received = p->second;

    if (key == m_ownship)
      continue;

    auto timediff = (curr_time - time_received);
    // Logger::info("Checking Poly Staleness: " + key + " " + doubleToStringX(timediff, 1));

    // Check if the time received is older than the threshold
    if ((curr_time - time_received) <= m_neighbor_stale_treshold)
      continue;

    m_map_vPolys.erase(key);
    m_map_vAreas.erase(key);
    m_map_neighbour_time_received.erase(key);
    Logger::info("Checking Poly Staleness: Erased " + key);
  }
}

XYPoint Proxonoi::getSetPtAreaBalance() const
{
  // ==============================================
  // Part 1: Sanity Checks
  // ==============================================
  XYPoint null_pt; // invalid by default

  if (!m_prox_poly.is_convex())
  {
    Logger::warning("current voronoi poly is not convex");
    return (null_pt);
  }

  XYPoint cpt = m_prox_poly.get_centroid_pt();
  if (!cpt.valid())
  {
    Logger::warning("Centroid not valid");
    return (null_pt);
  }

  // Find the neighbors of the current polygon
  std::vector<std::string> neighbors;
  XYPolygon kpoly = m_prox_poly;
  for (auto p = m_map_vPolys.begin(); p != m_map_vPolys.end(); p++)
  {
    std::string pkey = p->first;
    if (pkey == m_ownship)
      continue;

    double dist = kpoly.dist_to_poly(p->second);
    if (dist < 1)
      neighbors.push_back(pkey);
  }

  if (neighbors.size() == 0)
  {
    // Logger::info("No neighbors found");
    return (cpt);
  }

  // ==============================================
  // Part 2: Calculate component vectors, one for each
  //         neighbor and add all vectors, keeping track
  //         of the total magnitud along the way
  // ==============================================
  double pa = m_prox_poly.area();
  double total_abs_mag = 0;

  // The vector point (vpt) is sum of all hdg/mag vectors.
  // It starts at the current center point of the cell.
  XYPoint vpt = cpt;
  for (unsigned int j = 0; j < neighbors.size(); j++)
  {
    std::string jkey = neighbors[j];
    XYPoint jpt = m_map_vPolys.at(jkey).get_center_pt();
    double paj = m_map_vAreas.at(jkey);

    double hdg = relAng(cpt, jpt);
    double mag = 0;
    double dist = distPointToPoint(cpt, jpt);
    if ((pa + paj) > 0)
    {
      double nfactor = paj - pa;
      double dfactor = (pa + paj) / 2;
      mag = (nfactor / dfactor) * (dist / 2);
    }

    // Add the new hdg/mag vector to the running sum
    vpt = projectPoint(hdg, mag, vpt);
    total_abs_mag += std::abs(mag);
  }

  // ==============================================
  // Part 3: Define the final point, having the heading of the sum
  //         of vectors, but magnitude is avg of all magnitudes
  // ==============================================
  double avg_abs_mag = total_abs_mag / (double)(neighbors.size());

  double hdg = relAng(cpt, vpt);
  double dist = avg_abs_mag;

  XYPoint final_pt = projectPoint(hdg, dist, cpt);

  // ==============================================
  // Part 4: Sanity check that point is within op_region
  // ==============================================
  if (!m_prox_region.contains(final_pt.x(), final_pt.y()))
    final_pt = m_prox_region.closest_point_on_poly(final_pt);

  return (final_pt);
}

//------------------------------------------------------------
// UTILITY FUNCTIONS
//  These are not part of the Proxonoi class, but are
//  used by the Proxonoi class.
//-------------------------------------------------------------

//-----------------------------------------------------------
// Procedure: getSetPtAreaBalance()

// XYPoint getSetPtAreaBalance(const XYPolygon op_region, std::string key, const std::map<std::string, XYPolygon> &map_vpolygons, const std::map<std::string, double> &map_vAreas)
// {
//   XYPoint null_pt; // invalid by default
//   if (map_vpolygons.find(key) == map_vpolygons.end()){
//     Logger::warning("Key not found in map_vpolygons: " + key);
//     return (null_pt);
//   }

//   XYPoint cpt = map_vpolygons.at(key).get_centroid_pt();
//   if (!cpt.valid())
//   {
//     Logger::warning("Centroid not valid for key: " + key);
//     return (null_pt);
//   }

//   // If the centroid is not in the op_region, return the centroid
//   // ==============================================
//   // Part 1: Sanity Checks
//   // ==============================================

//   if (!op_region.is_convex())
//     return (cpt);

//   // Find the neighbors of the current polygon
//   std::vector<std::string> neighbors;
//   XYPolygon kpoly = map_vpolygons.at(key);
//   for (auto p = map_vpolygons.begin(); p != map_vpolygons.end(); p++)
//   {
//     std::string pkey = p->first;
//     if (pkey != key)
//     {
//       double dist = kpoly.dist_to_poly(p->second);
//       if (dist < 1)
//         neighbors.push_back(pkey);
//     }
//   }

//   if (neighbors.size() == 0)
//     return (cpt);

//   // ==============================================
//   // Part 2: Calculate component vectors, one for each
//   //         neighbor and add all vectors, keeping track
//   //         of the total magnitud along the way
//   // ==============================================
//   double pa = map_vAreas.at(key);
//   double total_abs_mag = 0;

//   // The vector point (vpt) is sum of all hdg/mag vectors.
//   // It starts at the current center point of the cell.
//   XYPoint vpt = cpt;
//   for (unsigned int j = 0; j < neighbors.size(); j++)
//   {
//     std::string jkey = neighbors[j];
//     XYPoint jpt = map_vpolygons.at(jkey).get_center_pt();
//     double paj = map_vAreas.at(jkey);

//     double hdg = relAng(cpt, jpt);
//     double mag = 0;
//     double dist = distPointToPoint(cpt, jpt);
//     if ((pa + paj) > 0)
//     {
//       double nfactor = paj - pa;
//       double dfactor = (pa + paj) / 2;
//       mag = (nfactor / dfactor) * (dist / 2);
//     }

//     // Add the new hdg/mag vector to the running sum
//     vpt = projectPoint(hdg, mag, vpt);
//     total_abs_mag += std::abs(mag);
//   }

//   // ==============================================
//   // Part 3: Define the final point, having the heading of the sum
//   //         of vectors, but magnitude is avg of all magnitudes
//   // ==============================================
//   double avg_abs_mag = total_abs_mag / (double)(neighbors.size());

//   double hdg = relAng(cpt, vpt);
//   double dist = avg_abs_mag;

//   XYPoint final_pt = projectPoint(hdg, dist, cpt);

//   // ==============================================
//   // Part 4: Sanity check that point is within op_region
//   // ==============================================
//   if (!op_region.contains(final_pt.x(), final_pt.y()))
//     final_pt = op_region.closest_point_on_poly(final_pt);

//   return (final_pt);
// }

XYPolygon stringMod2Poly(std::string str)
{
  XYPolygon null_poly;
  XYPolygon new_poly;

  std::string rest = stripBlankEnds(str);

  Logger::info("stringStandard2Poly: " + str);

  while (rest != "")
  {
    std::string left = stripBlankEnds(biteString(rest, '='));
    rest = stripBlankEnds(rest);

    if (left == "pts")
    {
      std::string pstr = biteStringX(rest, '}');

      // Empty set of points is an error
      if (pstr == "")
        return (null_poly);

      // Points should begin with an open brace (but discard now)
      if (pstr[0] != '{')
        return (null_poly);
      else
        pstr = pstr.substr(1);

      // If more components after pts={}, then it should begin w/ comma
      if (rest != "")
      {
        if (rest[0] != ',')
          return (null_poly);
        else
          rest = rest.substr(1);
      }

      std::vector<std::string> svector = parseString(pstr, ':');
      unsigned int i, vsize = svector.size();
      for (i = 0; i < vsize; i++)
      {
        std::string vertex = stripBlankEnds(svector[i]);
        std::string xstr = biteStringX(vertex, ',');
        std::string ystr = biteStringX(vertex, ',');
        std::string zstr = biteStringX(vertex, ',');
        std::string pstr = biteStringX(vertex, ',');

        std::string property;

        if (!isNumber(xstr) || !isNumber(ystr))
          return (null_poly);
        
        double xval = atof(xstr.c_str());
        double yval = atof(ystr.c_str());
        double zval = 0;
        if (isNumber(zstr))
          zval = atof(zstr.c_str());
        else if (zstr != "")
          property = zstr;

        if (pstr != "")
        {
          if (property != "")
            property += ",";
          property += pstr;
        }
        new_poly.add_vertex(xval, yval, zval, property, false);
      }
      new_poly.determine_convexity();
    }
    else
    {
      std::string right = stripBlankEnds(biteString(rest, ','));
      rest = stripBlankEnds(rest);
      new_poly.set_param(left, right);
    }
  }

  if (new_poly.is_convex())
    return (new_poly);

  // Mod by mikerb Jan 26,2020 Support for deactivation poly strings
  // such as "label=foobar,active=false". This lightens the burden
  // for apps that simply want to erase a polygon without having to
  // provide any points.
  if (new_poly.active() == false)
  {
    null_poly.set_label(new_poly.get_label());
    null_poly.set_active(false);
  }
  return (null_poly);
}