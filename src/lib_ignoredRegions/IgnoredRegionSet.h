#ifndef IGNORED_REGION_SET_HEADER
#define IGNORED_REGION_SET_HEADER

#include <map>
#include <string>
#include "XYMarker.h"
#include "XYPolygon.h"
#include "IgnoredRegion.h"
#include "IgnoredRegionGenerator.h"

class IgnoredRegionSet
{
public:
    IgnoredRegionSet();
    virtual ~IgnoredRegionSet() {}

    bool handleRegionConfig(std::string str, double curr_time, std::string &warning);
    bool handleRegionFile(std::string, double, std::string &warning);
    bool handleSearchRegionStr(std::string str, std::string &warning);
    bool modIgnoredRegion(IgnoredRegion);

    std::string getNameOfIgnoredRegionContaining(double x, double y);

    bool hasIgnoredRegion(std::string name) const;
    IgnoredRegion getIgnoredRegion(std::string name) const;

    bool hasIgnoredRegionByID(std::string id) const;
    IgnoredRegion getRegionByID(std::string id) const;

    bool allIgnoredRegionsDiscovered() const;
    std::vector<IgnoredRegion> tryAddSpawnableRegion(double mission_start_utc, double curr_time_utc);
    bool addRegion(std::string str, double curr_time, std::string &warning);

    void setMissionStartTimeOnRegions(double v);

    XYPolygon getSearchRegion() const { return (m_search_region); }
    void setSearchRegion(XYPolygon poly) { m_search_region = poly; }
    bool isSearchRegionValid() const { return (m_search_region.size() > 0 && m_search_region.is_convex()); }

    void tagIgnoredRegionID(IgnoredRegion &region);

    std::vector<IgnoredRegion> getRegions() const;
    std::set<std::string> getIgnoredRegionNames() const;
    std::vector<std::string> getIgnoredRegionFileSpec() const;
    std::string getRegionFile() const { return (m_region_file); }

    unsigned int getTotalIgnoredRegionsDiscovered() const;
    unsigned int getTotalIgnoredRegionsDiscoveredBy(std::string vname) const;
    unsigned int size() const { return (m_map_ignoredRegions.size()); }
    unsigned int spawnsize() const { return (m_vec_spawnable_regions.size()); }

    std::string getSavePath() const { return (m_region_config_save_path); }

    bool removeIgnoreRegion(std::string rname);

    std::string spawnIgnoreRegion(double x, double y, double scale_factor=1);

protected:
    void shuffleIDs();
    bool configureIgnoreRegionVisuals(std::string rname);

protected: // State variables
    std::map<std::string, IgnoredRegion> m_map_ignoredRegions;
    std::vector<std::pair<double, std::string>> m_vec_spawnable_regions;
    std::map<std::string, std::string> m_map_ignoredRegion_ids;

    std::vector<int> m_shuffled_ids;

protected: // Configuration variables
    std::string m_region_config_save_path;
    std::string m_region_file;
    XYPolygon m_search_region;
    unsigned int m_max_size; // Maximum number of initial regions (const defined in .cpp)

    IgnoredRegionGenerator m_generator;
};

#endif
