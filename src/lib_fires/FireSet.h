#ifndef FIRE_SET_HEADER
#define FIRE_SET_HEADER

#include <map>
#include <string>
#include "XYMarker.h"
#include "XYPolygon.h"
#include "Fire.h"

class FireSet
{
public:
    FireSet();
    virtual ~FireSet() {}

    bool handleFireConfig(std::string str, double curr_time, std::string &warning);
    bool handleFireFile(std::string, double, std::string &warning);
    bool handleSearchRegionStr(std::string str, std::string &warning);
    bool fireAlert(std::string str, double, std::string &warning);
    bool modFire(Fire);

    std::string getNameClosestFire(double x, double y, double rng = 10);

    bool hasFire(std::string fname) const;
    Fire getFire(std::string fname) const;

    bool hasFireByID(std::string id) const;
    Fire getFireByID(std::string id) const;

    bool addFire(std::string name, std::string fstate,
                 double xpos, double ypos,
                 double curr_time,
                 std::string &warning);

    std::set<std::string> getFireNames() const;

    XYPolygon getSearchRegion() const { return (m_search_region); }
    void setSearchReagion(XYPolygon poly) { m_search_region = poly; }
    bool isSearchRegionValid() const { return (m_search_region.size() > 0 && m_search_region.is_convex()); }

    std::string getFireFile() const { return (m_fire_file); }

    void tagFireID(Fire &fire);

    unsigned int size() const { return (m_map_fires.size()); }
    unsigned int getTotalFiresDiscovered() const;
    unsigned int getTotalFiresDiscoveredBy(std::string vname) const;
    // unsigned int getMinSep() const { return (0); }

    bool allFiresDiscovered() const;

    std::vector<Fire> getFires() const;

    std::vector<std::string> getFireFileSpec() const;

protected:
    void shuffleIDs();

protected: // State variables
    std::map<std::string, Fire> m_map_fires;
    std::map<std::string, std::string> m_map_fire_ids;

    std::vector<int> m_shuffled_ids;

protected: // Configuration variables
    std::string m_fire_file;
    XYPolygon m_search_region;
    unsigned int m_max_size; // Maximum number of initial fires (const defined in .cpp)
};

#endif
