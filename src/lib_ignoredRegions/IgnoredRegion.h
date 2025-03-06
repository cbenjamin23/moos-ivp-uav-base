/************************************************************/
/*    NAME: Michael Benjamin, modified by                   */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: IgnoredRegion.h                                 */
/*    DATE: Apr 2nd, 2022                                   */
/************************************************************/

#ifndef IGNORED_REGION_HEADER
#define IGNORED_REGION_HEADER

#include <string>
#include <set>
#include "XYPolygon.h"
#include "XYMarker.h"


const double REGION_MARKER_WIDTH = 20;


class IgnoredRegion
{
public:
    IgnoredRegion(std::string fname = "");
    ~IgnoredRegion() {};

    enum RegionState
    {
        UNDISCOVERED,
        DISCOVERED,
        UNKNOWN
    };

    enum class RegionType
    {
        ELLIPSE,
        RADIAL,
        OVAL,
        HEXAGON,
        RECTANGLE,
        UNKNOWN
    };

public: // Setters
    void setTimeEnter(double v) { m_time_enter = v; }
    void setTimeDiscovered(double v) { m_time_discovered = v; }

    bool setStateFromString(std::string s);
    bool setState(RegionState s);
    void setDiscoverer(std::string s) { m_discoverer = s; }
    void setName(std::string s) { m_name = s; }
    void setID(std::string s) { m_id = s; }
    void setLabel(std::string s) { m_id = s; }
    void setScoutTries(unsigned int v) { m_scout_tries = v; }
    void addScouted(std::string s) { m_set_scouted.insert(s); }
    void incScoutTries() { m_scout_tries++; }
    void setRegion(XYPolygon region) { m_region = region; }
    void setRegionType(RegionType type) { m_region_type = type; } // Might not be needed
    void setMarker(XYMarker marker) { m_marker = marker; }
    void setFormatSpec(std::string spec) { m_format_spec = spec; }

public: // Getters
    double getTimeEnter() const { return (m_time_enter); }
    double getTimeDiscovered() const { return (m_time_discovered); }

    RegionState getState() const { return (m_state); }
    std::string getDiscoverer() const { return (m_discoverer); }
    std::string getName() const { return (m_name); } 
    std::string getID() const { return (m_id); } 
    std::string getLabel() const { return (m_label); } 
    unsigned int getScoutTries() const { return (m_scout_tries); }
    XYPolygon getRegion() const { return (m_region); }
    RegionType getRegionType() const { return m_region_type; }
    XYMarker getMarker() const { return m_marker; }
    std::string getFormatSpec() const { return m_format_spec; }

    std::set<std::string> getScoutSet() { return (m_set_scouted); }
    bool hasBeenScouted(std::string vname = "") const;
    bool isDiscovered() const { return (m_state == DISCOVERED); }
    bool contains(double x, double y) const { return m_region.contains(x, y); }
    std::string getSpec() const;
    std::string getSpecRegion() const;
    std::string getSpecMarker() const;

    bool isValid(){return(m_region_type!=RegionType::UNKNOWN);}

private:
    XYPolygon m_region; // The ignored region
    XYMarker m_marker;  // The marker for the region

    double m_time_enter;      // time region started
    double m_time_discovered; // time discovered
    RegionState m_state;      // undiscovered or discovered
    RegionType m_region_type; // type of region
    std::string m_discoverer; // who discovered
    std::string m_name;       // unique key name
    std::string m_id;
    std::string m_label;       // Non-unique name of the region (eg. office, field, etc)
    std::string m_format_spec; // original format specification

    std::set<std::string> m_set_scouted;
    unsigned int m_scout_tries;
};

IgnoredRegion stringToIgnoredRegion(std::string str);

std::string regionStateToString(IgnoredRegion::RegionState state);
IgnoredRegion::RegionState stringToRegionState(std::string state);

std::string regionTypeToString(IgnoredRegion::RegionType type);
IgnoredRegion::RegionType stringToRegionType(std::string type_str);

// Helpers
XYPolygon stringHexagon2Poly(std::string str);
XYPolygon stringRectangle2Poly(std::string str);

#endif
