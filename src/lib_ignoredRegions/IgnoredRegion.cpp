/************************************************************/
/*    NAME: Michael Benjamin, modified by                   */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: IgnoredRegion.cpp                               */
/*    DATE: Apr 2nd, 2022                                   */
/************************************************************/

#include <vector>
#include <iterator>
#include "MBUtils.h"
#include "IgnoredRegion.h"
#include "XYFormatUtilsPoly.h"

#include <numeric>
#include <cmath>
#include <algorithm> 


#include "Logger.h"

using namespace std;

IgnoredRegion::IgnoredRegion(string fname)
{
    m_time_enter = 0;
    m_time_discovered = 0;

    m_state = RegionState::UNDISCOVERED;
    m_region_type = RegionType::UNKNOWN;
    m_name = fname;

    m_scout_tries = 0;
}

bool IgnoredRegion::setStateFromString(string s)
{
    return (setState(stringToRegionState(s)));
}

bool IgnoredRegion::setState(RegionState s)
{
    if (s != RegionState::UNDISCOVERED && s != RegionState::DISCOVERED)
        return (false);

    m_state = s;
    return (true);
}

bool IgnoredRegion::hasBeenScouted(string vname) const
{
    if (vname == "")
        return (m_set_scouted.size() != 0);

    if (m_set_scouted.count(vname) == 0)
        return (false);

    return (true);
}

string IgnoredRegion::getSpec() const
{
    string spec = "name=" + m_name;
    std::string state = regionStateToString(m_state);
    if (state != "")
        spec += ", state=" + state;

    
    spec += ", type=" + regionTypeToString(m_region_type);

    if(m_format_spec != "")
        spec += ", " + m_format_spec;
    if (m_time_enter != 0)
        spec += ", time_enter=" + doubleToStringX(m_time_enter, 2);
    if (m_time_discovered != 0)
        spec += ", time_discovered=" + doubleToStringX(m_time_discovered, 2);
    if (m_discoverer != "")
        spec += ", discoverer=" + m_discoverer;
    if (m_id != "")
        spec += ", id=" + m_id;
    if (m_scout_tries != 0)
        spec += ", scout_tries=" + uintToString(m_scout_tries);

    spec += ", format=" + m_format_spec;
    
    return (spec);
}

string IgnoredRegion::getSpecRegion() const
{
    return m_format_spec;
}

string IgnoredRegion::getSpecMarker() const
{
    if (!m_marker.valid())
        return "";

    return m_marker.get_spec();
}

// Format: time_enter=4, time_discovered=52, name=val, id=25, state=discovered, \
           discoverer=ben, scout_tries=2 , \
           format = format|hexagon; msg|wood_chop; x|$[XPOS]; y|$[YPOS]; rad|60; pts|8; snap_val|1.0 \
    
IgnoredRegion stringToIgnoredRegion(std::string str)
{
    IgnoredRegion null_ignoredRegion;
    IgnoredRegion ignoredRegion;

    bool ok = true;
    vector<string> svector = parseString(str, ',');
    string format_spec = "";

    for (unsigned int i = 0; i < svector.size(); i++)
    {
        string param = tolower(biteStringX(svector[i], '='));
        string value = svector[i];
        double dval = atof(value.c_str());

        if (param == "time_enter")
            ignoredRegion.setTimeEnter(dval);
        else if (param == "time_discovered")
            ignoredRegion.setTimeDiscovered(dval);
        else if (param == "name")
            ignoredRegion.setName(value);
        else if (param == "id")
            ignoredRegion.setID(value);
        else if (param == "state")
            ok = ignoredRegion.setStateFromString(value);
        else if (param == "discoverer")
            ignoredRegion.setDiscoverer(value);
        else if (param == "scout_tries")
            ignoredRegion.setScoutTries((unsigned int)(dval));
        else if (param == "type" && stringToRegionType(value) != IgnoredRegion::RegionType::UNKNOWN)
            ignoredRegion. setRegionType(stringToRegionType(value));

        else if (param == "format")
            format_spec = value;
    }

    Logger::info("format_spec: " + format_spec);

    if (!ok)
        return (null_ignoredRegion);

    if(format_spec.empty())
        return null_ignoredRegion;


    std::string regionType_str = tokStringParse(format_spec, "format", ';', '|');
    std::string msg = tokStringParse(format_spec, "msg", ';', '|');
    

    Logger::info("regionType_str: " + regionType_str);
    Logger::info("msg : " + msg);
    Logger::info("name: " + ignoredRegion.getName());


    std::function<XYPolygon(std::string)> convertionFnc;
    switch (stringToRegionType(regionType_str))
    {
    case IgnoredRegion::RegionType::HEXAGON: 
        convertionFnc = stringHexagon2Poly;
        break;

    case IgnoredRegion::RegionType::RECTANGLE:
        convertionFnc = stringRectangle2Poly;
        break;

    default:
        convertionFnc = string2Poly;
        break;
    }
    
    std::string modified_spec = findReplace(format_spec, ";",',');
    modified_spec = findReplace(modified_spec, "|",'=');
    Logger::info("format_spec (modified): " + modified_spec);

    XYPolygon region = convertionFnc(modified_spec);
    ignoredRegion.setRegion(region);  
    ignoredRegion.setLabel(msg);

    if (ignoredRegion.getState() == IgnoredRegion::RegionState::UNKNOWN)
    {
        ignoredRegion.setState(IgnoredRegion::RegionState::UNDISCOVERED);
    }

    // Create and set the marker
    XYMarker marker(region.get_center_x(), region.get_center_y());
    marker.set_msg(msg);
    marker.set_type("efield");
    marker.set_color("primary_color", "black");
    marker.set_color("secondary_color", "orange");
    marker.set_width(REGION_MARKER_WIDTH);

    ignoredRegion.setMarker(marker);
    ignoredRegion.setFormatSpec(format_spec);
    ignoredRegion.setRegionType(stringToRegionType(regionType_str)); // Extract format type

    return (ignoredRegion);
}


XYPolygon stringHexagon2Poly(std::string str)
{
  std::string msg = tokStringParse(str, "msg");
  double x = tokDoubleParse(str, "x");
  double y = tokDoubleParse(str, "y");
  double rad = tokDoubleParse(str, "rad");
  unsigned int pts = std::floor(tokDoubleParse(str, "pts"));
  double snap = tokDoubleParse(str, "snap_val");

  // Logger::info("Hexagon: x: " + doubleToStringX(x) + ", y: " + doubleToStringX(y) + ", rad: " + doubleToStringX(rad) + ", pts: " + doubleToStringX(pts) + ", snap: " + doubleToStringX(snap));

  XYPolygon region = XYPolygon(x, y, rad, pts);
  region.set_msg(msg);
  region.apply_snap(snap);
  return region;
}

XYPolygon stringRectangle2Poly(std::string str) 
{
  std::string msg = tokStringParse(str, "msg");
  double cx = tokDoubleParse(str, "cx");
  double cy = tokDoubleParse(str, "cy");
  double width = tokDoubleParse(str, "width");
  double height = tokDoubleParse(str, "height");
  double degs = tokDoubleParse(str, "degs");

  // compute the points of the rectangle from the center, width and heigh
  XYSegList corners;
  double half_width = width / 2.0;
  double half_height = height / 2.0;
  corners.add_vertex(cx + half_width, cy + half_height); // Top-right
  corners.add_vertex(cx - half_width, cy + half_height); // Top-left
  corners.add_vertex(cx - half_width, cy - half_height); // Bottom-left
  corners.add_vertex(cx + half_width, cy - half_height); // Bottom-right

  XYPolygon region = XYPolygon(corners);
  region.rotate(degs);
  region.set_msg(msg);

  return region;
}

std::string regionStateToString(IgnoredRegion::RegionState state)
{
    switch (state)
    {
    case IgnoredRegion::UNDISCOVERED:
        return "undiscovered";
    case IgnoredRegion::DISCOVERED:
        return "discovered";
    }
    return "unknown";
}

IgnoredRegion::RegionState stringToRegionState(std::string state)
{
    if (state == "undiscovered")
        return IgnoredRegion::RegionState::UNDISCOVERED;
    else if (state == "discovered")
        return IgnoredRegion::RegionState::DISCOVERED;

    return IgnoredRegion::RegionState::UNKNOWN;
}

string regionTypeToString(IgnoredRegion::RegionType type)
{
    switch (type)
    {
    case IgnoredRegion::RegionType::ELLIPSE:
        return "ellipse";
    case IgnoredRegion::RegionType::RADIAL:
        return "radial";
    case IgnoredRegion::RegionType::OVAL:
        return "oval";
    case IgnoredRegion::RegionType::HEXAGON:
        return "hexagon";
    case IgnoredRegion::RegionType::RECTANGLE:
        return "rectangle";
    default:
        return "unknown";
    }
}

IgnoredRegion::RegionType stringToRegionType(string type_str)
{
    if (type_str == "ellipse")
        return IgnoredRegion::RegionType::ELLIPSE;
    else if (type_str == "radial")
        return IgnoredRegion::RegionType::RADIAL;
    else if (type_str == "oval")
        return IgnoredRegion::RegionType::OVAL;
    else if (type_str == "hexagon")
        return IgnoredRegion::RegionType::HEXAGON;
    else if (type_str == "rectangle")
        return IgnoredRegion::RegionType::RECTANGLE;

    return IgnoredRegion::RegionType::UNKNOWN;
}
