/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: BHV_Voronoi.cpp                                      */
/*    DATE: Jan 6th 2020                                         */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#include <cmath>
#include <cstdlib>
#include "AngleUtils.h"
#include "GeomUtils.h"
#include "XYFormatUtilsPoly.h"
#include "BHV_Voronoi_uav.h"
#include "MBUtils.h"
#include "BuildUtils.h"
#include "ZAIC_PEAK.h"
#include "OF_Coupler.h"

#include "XYFormatUtilsPoint.h"

using namespace std;

//-----------------------------------------------------------
// Constructor

BHV_Voronoi::BHV_Voronoi(IvPDomain gdomain) : IvPBehavior(gdomain)
{
  this->setParam("descriptor", "voronoi havior for uav");

  m_domain = subDomain(m_domain, "course,speed");

  // Initialize state variables
  m_osx = 0;
  m_osy = 0;

  m_set_x = 0;
  m_set_y = 0;

  m_ownship_in_region = false;
  m_total_activated_dist = 0;

  m_state = "idle";

  // Initialize config params
  m_cruise_speed = 0;

  m_capture_radius = 10;
  m_activate_radius = 12;

  m_stale_nav_thresh = 5;
  m_stale_poly_thresh = 10;
  m_stale_searchcenter_thresh = 10;

  m_setpt_method = "center";

  m_allow_slipping = false;

  m_hint_setpt_size = 1;
  m_hint_setpt_color = "red";

  addInfoVars("NAV_X, NAV_Y, NAV_SPEED, NAV_HEADING");
  addInfoVars("PROXONOI_POLY");
  addInfoVars("PROXONOI_REGION");

  addInfoVars("PROX_SEARCHCENTER");
  addInfoVars("PROX_SETPT_METHOD");
}

//-----------------------------------------------------------
// Procedure: setParam

bool BHV_Voronoi::setParam(string param, string param_val)
{
  if (IvPBehavior::setParam(param, param_val))
    return (true);

  bool handled = false;
  if (param == "speed")
    handled = setNonNegDoubleOnString(m_cruise_speed, param_val);

  else if (param == "op_region")
    handled = handleConfigOpRegion(param_val);
  else if (param == "capture_radius")
    handled = setNonNegDoubleOnString(m_capture_radius, param_val);
  else if (param == "activate_radius")
    handled = setNonNegDoubleOnString(m_activate_radius, param_val);
  else if (param == "stale_nav_thresh")
    handled = setNonNegDoubleOnString(m_stale_nav_thresh, param_val);
  else if (param == "stale_poly_thresh")
    handled = setNonNegDoubleOnString(m_stale_poly_thresh, param_val);
  else if (param == "stale_searchcenter_thresh")
    handled = setNonNegDoubleOnString(m_stale_searchcenter_thresh, param_val);
  else if (param == "setpt_method")
    handled = handleConfigSetPointMethod(param_val);

  else if (param == "visual_hints")
  {
    vector<string> svector = parseStringQ(param_val, ',');
    for (unsigned int i = 0; i < svector.size(); i++)
      handleVisualHint(svector[i]);
    handled = true;
  }
  else if (param == "allow_slipping")
  {
    handled = setBooleanOnString(m_allow_slipping, param_val);
  }

  return (handled);
}

//-----------------------------------------------------------
// Procedure: onSetParamComplete
//
//      Note: The active_radius is the range from the set point
//            beyond which the behavior will again produce an
//            objective function to move toward the set point.
//            The active_radius must be greater than the
//            capture_radius by at least 2%. If 2% translates
//            to be less than 2 meters, then a 2 meter difference
//            is enforced.

void BHV_Voronoi::onSetParamComplete()
{
  double min_activate_radius = m_capture_radius * 1.02;

  if ((min_activate_radius - m_capture_radius) < 2)
    min_activate_radius = m_capture_radius + 2;

  if (m_activate_radius < min_activate_radius)
    m_activate_radius = min_activate_radius;
}

//-----------------------------------------------------------
// Procedure: onHelmStart()

void BHV_Voronoi::onHelmStart()
{
}

//-----------------------------------------------------------
// Procedure: updateOwnshipPosition()
//   Returns: true if Nav info is found and not stale
//            false otherwise

bool BHV_Voronoi::updateOwnshipPosition()
{
  //=========================================================
  // Part 1: Update ownship position and check for errors
  //=========================================================
  bool ok_x = true;
  bool ok_y = true;
  double new_osx = getBufferDoubleVal("NAV_X", ok_x);
  double new_osy = getBufferDoubleVal("NAV_Y", ok_y);

  if (!ok_y || !ok_y)
  {
    postEMessage("ownship NAV_X/Y info not found.");
    return (false);
  }

  //=========================================================
  // Part 2: Check for staleness of ownship NAV information
  //=========================================================
  double tstamp_osx = getBufferTimeVal("NAV_X");
  double tstamp_osy = getBufferTimeVal("NAV_Y");
  if ((tstamp_osx > m_stale_nav_thresh) ||
      (tstamp_osy > m_stale_nav_thresh))
  {
    postEMessage("ownship NAV_X/Y info is stale.");
    return (false);
  }

  //=========================================================
  // Part 3: Update ownship position and the odometer
  //=========================================================
  m_osx = new_osx;
  m_osy = new_osy;
  m_ownship_in_region = m_proxonoi_region.contains(m_osx, m_osy);

  m_odometer.setX(m_osx);
  m_odometer.setY(m_osy);

  if ((m_state == "activated") && m_ownship_in_region)
    m_odometer.unpause();
  else
    m_odometer.pause();

  m_odometer.updateDistance();
  m_total_activated_dist = m_odometer.getTotalDist();

  return (true);
}

//-----------------------------------------------------------
// Procedure: updateProxonoiPoly
//   Returns: true if poly is found, ok syntax, and not stale
//            false otherwise

bool BHV_Voronoi::updateProxonoiPolys()
{
  cout << "updateProxonoiPolys()" << endl;
  //=========================================================
  // Part 1: Handle the Proxonoi Region. It may be rarely posted
  // and thus rarely need updating. A staleness of zero means
  // it was updated on this iteration. There is no upper limit
  // on tolerable staleness for this variable.
  //=========================================================
  double region_staleness = getBufferTimeVal("PROXONOI_REGION");
  if (region_staleness == 0)
  {
    cout << "updateProxonoiPolysA()" << endl;
    string polystr = getBufferStringVal("PROXONOI_REGION");
    cout << "updateProxonoiPolysB()" << endl;

    // Check for ok syntax in Proxonoi Region
    XYPolygon new_region = string2Poly(polystr);
    cout << "updateProxonoiPolysC()" << endl;
    if (!new_region.is_convex())
    {
      cout << "updateProxonoiPolysD()" << endl;
      postEMessage("Proxonoi region is non-convex.");
      postMessage("BAD_POLY", polystr);
      return (false);
    }
    cout << "updateProxonoiPolysE()" << endl;
    m_proxonoi_region = new_region;
  }

  cout << "updateProxonoiPolys2()" << endl;
  //=========================================================
  // Part 2: Handle the Proxonoi Polygon. It should be regularly
  // updated but perhaps not on every helm/behavior iteration.
  // A staleness of zero means it was updated on this helm or
  // behavior iteration. An upper limit on tolerable staleness
  // is checked for and enforced.
  //=========================================================
  double poly_staleness = getBufferTimeVal("PROXONOI_POLY");
  if (poly_staleness > m_stale_poly_thresh)
  {
    postWMessage("Proxonoi polygon info_buffer is stale.");
    return (false);
  }
  cout << "updateProxonoiPolys3()" << endl;

  if (poly_staleness == 0)
  {
    string polystr = getBufferStringVal("PROXONOI_POLY");

    // Check for ok syntax in Proxonoi Poly. If convex, all is good.
    // If nonconvex poly with non-zero number of vertices, this is a
    // problem. Truly null polys (zero vertices) are fine, and mean
    // there just is not proxonoi poly to be used.

    XYPolygon new_poly = string2Poly(polystr);
    if (new_poly.is_convex())
      m_proxonoi_poly = new_poly;
    else if (new_poly.size() > 0)
    {
      postEMessage("Proxonoi polygon is non-convex.");
      postMessage("BAD_POLY", polystr);
      return (false);
    }
  }

  cout << "updateProxonoiPolys() END" << endl;
  return (true);
}

//-----------------------------------------------------------
// Procedure: handleConfigOpRegion()
//   Returns: true OpRegion Poly is convex

bool BHV_Voronoi::handleConfigOpRegion(string polystr)
{
  XYPolygon new_poly = string2Poly(polystr);
  if (!new_poly.is_convex())
    return (false);

  m_op_region = new_poly;
  return (true);
}

//-----------------------------------------------------------
// Procedure: handleConfigSetPointMethod()

bool BHV_Voronoi::handleConfigSetPointMethod(string method)
{
  method = tolower(method);

  if ((method != "center") && (method != "centroid") && method != "gridsearch")
    return (false);

  m_setpt_method = method;
  return (true);
}

//-----------------------------------------------------------
// Procedure: updateSetPoint()
//   Returns: true if (a) region is convex

bool BHV_Voronoi::updateSetPoint()
{
  // Part 1: If the region is not convex, all is fubar
  if (!m_proxonoi_region.is_convex())
  {
    postErasableSetPoint();
    return (false);
  }

  double method_staleness = getBufferTimeVal("PROX_SETPT_METHOD");
  if (method_staleness == 0)
  {
    string method_str = getBufferStringVal("PROX_SETPT_METHOD");
    handleConfigSetPointMethod(method_str);
  }

  XYPoint gridsearch_setpt;
  if (m_setpt_method == "gridsearch")
  {

    auto searchcenter_staleness = getBufferTimeVal("PROX_SEARCHCENTER");
    if (searchcenter_staleness > m_stale_searchcenter_thresh)
    {
      postWMessage("Gridsearch setpt info_buffer is stale.");
      return (false);
    }

    std::string searchcenter_str = getBufferStringVal("PROX_SEARCHCENTER");
    auto pt = string2Point(searchcenter_str);
    if (pt.valid())
    {
      gridsearch_setpt = pt;
    }
    else
    {
      postEMessage("Gridsearch setpt is invalid");
      return (false);
    }
  }

  // Part 2: If ownship is in the region and we have a valid
  //         proxonoi poly, then use the proxonoi poly setpt
  if (m_ownship_in_region && m_proxonoi_poly.is_convex())
  {

    if (m_setpt_method == "gridsearch")
    {
      m_set_x = gridsearch_setpt.x();
      m_set_y = gridsearch_setpt.y();
    }
    else if (m_setpt_method == "center")
    {
      m_set_x = m_proxonoi_poly.get_center_x();
      m_set_y = m_proxonoi_poly.get_center_y();
    }
    else if (m_setpt_method == "centroid")
    {
      m_set_x = m_proxonoi_poly.get_centroid_x();
      m_set_y = m_proxonoi_poly.get_centroid_y();
    }
  }
  // Part 3: Otherwise we choose a setpoint that transits ownship
  //         to the center of the region
  else
  {
    if (m_setpt_method == "centroid")
    {
      m_set_x = m_proxonoi_region.get_centroid_x();
      m_set_y = m_proxonoi_region.get_centroid_y();
    }
    else
    {
      m_set_x = m_proxonoi_region.get_center_x();
      m_set_y = m_proxonoi_region.get_center_y();
    }
  }

  // Part 4: If setpt has moved, update the postings
  bool setpt_changed = false;
  if ((m_set_x != m_set_x_prev) || (m_set_y != m_set_y_prev))
    setpt_changed = true;

  if (setpt_changed)
  {
    m_set_x_prev = m_set_x;
    m_set_y_prev = m_set_y;

    string setpt_str = "x=" + doubleToStringX(m_set_x, 1);
    setpt_str += ",y=" + doubleToStringX(m_set_y, 1);
    postMessage("BVOI_SETPT", setpt_str);
    postViewableSetPoint();
  }

  return (true);
}

//-----------------------------------------------------------
// Procedure: postViewableSetPoint

void BHV_Voronoi::postViewableSetPoint()
{
  if (m_hint_setpt_size == 0)
    return;
  if (m_hint_setpt_color == "invisible")
    return;

  XYPoint point(m_set_x, m_set_y);
  point.set_label(m_us_name + "setpt");
  point.set_vertex_size(m_hint_setpt_size);
  point.set_vertex_color(m_hint_setpt_color);

  string spec = point.get_spec();
  postMessage("VIEW_POINT", spec);
}

//-----------------------------------------------------------
// Procedure: postErasableSetPoint

void BHV_Voronoi::postErasableSetPoint()
{
  XYPoint point(m_set_x, m_set_y);
  point.set_label(m_us_name + "setpt");
  point.set_active(false);

  string spec = point.get_spec();
  postMessage("VIEW_POINT", spec);
}

//-----------------------------------------------------------
// Procedure: onRunState

IvPFunction *BHV_Voronoi::onRunState()
{
  cout << "onRunState()" << endl;

  // Part 1: Update ownship and proxonoi information
  bool ok = updateOwnshipPosition();
  if (!ok)
  {
    postMessage("VOI_DEBUG", "Unable to update ownship position");
    return (0);
  }

  ok = updateProxonoiPolys();
  if (!ok)
  {
    postMessage("VOI_DEBUG", "Unable to update proxonoi Polys");
    return (0);
  }

  // Part 2: Update the set point given a proxonoi update
  ok = updateSetPoint();
  if (!ok)
  {
    postMessage("VOI_DEBUG", "Unable to update set point");
    return (0);
  }

  // Part 3: Determine the relevance
  double relevance = getRelevance();
  if (relevance <= 0)
  {
    postMessage("VOI_DEBUG", "Zero relevance");
    return (0);
  }

  // Part 4: Generate the IvP function
  IvPFunction *ipf = buildOF();

  // Part 5: Apply the relevance and priority weight
  if (ipf)
  {
    ipf->setPWT(relevance * m_priority_wt);
    postViewables();
  }
  else
    postMessage("VOI_DEBUG", "Unable to build IvP Function");

  return (ipf);
}

//-----------------------------------------------------------
// Procedure: buildOF

IvPFunction *BHV_Voronoi::buildOF()
{
  IvPFunction *ipf = 0;

  //===================================================
  // Part 1: Build the Speed ZAIC
  //===================================================
  IvPFunction *spd_ipf = 0;
  ZAIC_PEAK spd_zaic(m_domain, "speed");
  double peak_width = m_cruise_speed / 2;
  spd_zaic.setParams(m_cruise_speed, peak_width, 1.6, 20, 0, 100);

  spd_ipf = spd_zaic.extractIvPFunction();
  if (!spd_ipf)
    postWMessage("Failure on the SPD ZAIC via ZAIC_PEAK utility");

  //===================================================
  // Part 1: Build the Speed ZAIC
  //===================================================
  double rel_ang_to_wpt = relAng(m_osx, m_osy, m_set_x, m_set_y);

  ZAIC_PEAK crs_zaic(m_domain, "course");
  crs_zaic.setSummit(rel_ang_to_wpt);
  crs_zaic.setBaseWidth(180);
  crs_zaic.setValueWrap(true);

  IvPFunction *crs_ipf = crs_zaic.extractIvPFunction(false);
  if (!crs_ipf)
    postWMessage("Failure on the CRS ZAIC");

  OF_Coupler coupler;
  ipf = coupler.couple(crs_ipf, spd_ipf, 0.5, 0.5);
  if (!ipf)
    postWMessage("Failure on the CRS_SPD COUPLER");

  return (ipf);
}

//-----------------------------------------------------------
// Procedure: onRunToIdleState

void BHV_Voronoi::onRunToIdleState()
{
  postErasableSetPoint();
  postMessage("PROX_CLEAR", "true");
  setState("idle");
}

//-----------------------------------------------------------
// Procedure: onIdleState

void BHV_Voronoi::onIdleState()
{
}

//-----------------------------------------------------------
// Procedure: getRelevance

double BHV_Voronoi::getRelevance()
{
  // Part 1: First determine if we are in the activated state
  double dist_to_setpt = hypot(m_set_x - m_osx, m_set_y - m_osy);
  postMessage("VOI_DIST_TO_SETPT", dist_to_setpt);
  if (dist_to_setpt <= m_capture_radius)
  {
    setState("captured");
  }
  else if (dist_to_setpt >= m_activate_radius)
  {
    setState("activated");
  }
  // Then we might be slipping if we have captured, and are
  // more than the capture radius (handled in the if statement
  // above), and less than the activate radius (handled in the
  // else if statement above).
  // m_capture_radius =< dist_to_setpt <= m_activate_radius
  else if (m_state == "captured")
    setState("slipping");

  postMessage("VOI_STATE", m_state);

  // Part 2: Return the relevance based on activation state
  double activate_relevance = 100.0;
  double slipping_relevance = 10.0;
  if (m_state == "activated")
    return (activate_relevance);

  postMessage("VOI_STATE_DEBUG", "HERE");
  if (m_state == "slipping")
  {
    // start with low relevance, and then increase to
    // the activate_relevance as we get farther away,
    // and closer to reactivating.
    double dist_from_capture = dist_to_setpt - m_capture_radius;
    double dist_capture_to_active = m_activate_radius - m_capture_radius;
    if (dist_capture_to_active == 0.0)
      dist_capture_to_active = 1.0;

    postMessage("VOI_DEBUG_TMP", (dist_from_capture / dist_capture_to_active));
    double relevance = (activate_relevance - slipping_relevance) * (dist_from_capture / dist_capture_to_active) + slipping_relevance;
    return (relevance);
  }

  // otherwise return 0.
  return (0);
}

//-----------------------------------------------------------
// Procedure: setState()
//   Returns: true if the state changes

bool BHV_Voronoi::setState(string new_state)
{
  if ((new_state != "activated") && (new_state != "slipping") &&
      (new_state != "captured"))
    return (false);

  if (m_state == new_state)
    return (false);

  // If slipping is desired after capturing a point, then don't complete
  // here and keep the behavior active.

  if ((new_state == "captured") && !m_allow_slipping)
    setComplete();

  m_state = new_state;

  string msg = "vname=" + m_us_name + ",state=" + new_state;
  msg += ",activated_dist=" + doubleToStringX(m_total_activated_dist, 1);
  postMessage("BVOI_STATE", msg);

  return (true);
}

//-----------------------------------------------------------
// Procedure: handleVisualHint()

void BHV_Voronoi::handleVisualHint(string hint)
{
  string param = tolower(biteStringX(hint, '='));
  string value = hint;
  double dval = atof(value.c_str());

  if ((param == "setpt_size") && isNumber(value) && (dval >= 0))
    m_hint_setpt_size = dval;
  else if ((param == "setpt_color") && isColor(value))
    m_hint_setpt_color = value;
}

//-----------------------------------------------------------
// Procedure: postViewables()

void BHV_Voronoi::postViewables(unsigned int id)
{
  return;
  XYPoint point(m_set_x, m_set_y);
  string label = m_us_name + "_vpoly_" + intToString(id);

  point.set_label(label);
  point.set_color("label", "invisible");
  point.set_color("vertex", "white");

  string spec = point.get_spec();
  postMessage("VIEW_POINT", spec);
}

//-----------------------------------------------------------
// Procedure: eraseViewables()

void BHV_Voronoi::eraseViewables(unsigned int id)
{
  XYPoint point;
  string label = m_us_name + "_vpoly_" + intToString(id);

  point.set_label(label);
  point.set_active(false);

  string spec = point.get_spec();
  postMessage("VIEW_POINT", spec);
}
