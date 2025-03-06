/*****************************************************************/
/*    NAME: Michael Benjamin, modified by Steve Nomeny           */
/*    ORGN: Mechanical Eng / CSAIL, MIT Cambridge MA             */
/*    FILE: IgnoredRegionSet.cpp                                 */
/*    DATE: February 2022                                        */
/*****************************************************************/

#include <iterator>

#include <numeric>
#include <algorithm>
#include <cmath>
#include <fstream>

#include "IgnoredRegionSet.h"
#include "ColorParse.h"
#include "MBUtils.h"
#include "FileBuffer.h"
#include "XYFormatUtilsPoly.h"

#include "IgnoredRegionGenerator.h"

#include "Logger.h"
#include "common.h"

IgnoredRegionSet::IgnoredRegionSet()
{
    m_max_size = 99;
    shuffleIDs();
}

// Format:  generate = true, file = region.txt, count = 10, sep_min = 10, \
            region = {x0,y0:x1,y1:...:x2,y2}, save_path = "missions/UAV_FLY/gen_regions/", \
            spawn_count=10, spawn_interval = 200:400

bool IgnoredRegionSet::handleRegionConfig(std::string str, double curr_time, std::string &warning)
{
    std::string generate_str = tokStringParse(str, "generate");
    bool generate = false;
    setBooleanOnString(generate, generate_str);

    std::string file = tokStringParse(str, "file");

    if (!generate && file.empty())
    {
        warning = "Bad RegionConfig Line (need a file if not generating): " + str;
        return false;
    }

    if (!generate)
        return handleRegionFile(file, curr_time, warning);

    // Will generate regions

    std::string count_str = tokStringParse(str, "count");
    std::string sep_min_str = tokStringParse(str, "sep_min");
    std::string region_str = tokStringParse(str, "region");
    region_str = "pts=" + region_str;
    std::string save_path = tokStringParse(str, "save_path");
    std::string spawn_count_str = tokStringParse(str, "spawn_count");
    unsigned int spawn_count = 0;
    setUIntOnString(spawn_count, spawn_count_str);
    std::string spawn_interval_str = tokStringParse(str, "spawn_interval");

    if (count_str.empty())
        warning = "Bad RegionConfig Line (need count w/ generating): " + str;
    else if (sep_min_str.empty())
        warning = "Bad RegionConfig Line (need sep_min w/ generating): " + str;
    else if (region_str.empty())
        warning = "Bad RegionConfig Line (need region w/ generating): " + str;
    else if (save_path.empty())
        warning = "Bad RegionConfig Line (need save_path w/ generating): " + str;
    else if ((spawn_count > 0) && spawn_interval_str.empty())
        warning = "Bad RegionConfig Line (need spawn_interval w/ spawn_count): " + str;

    if (!warning.empty())
        return false;


    if (!m_generator.setSpawnInterval(spawn_interval_str))
        warning = "Bad RegionConfig Line (bad spawn_interval): " + str;
    else if (!m_generator.setRegionAmt(count_str))
        warning = "Bad RegionConfig Line (bad count): " + str;
    else if (!m_generator.setSpawnableRegionAmt(spawn_count_str))
        warning = "Bad RegionConfig Line (bad spawn_count): " + str;
    else if (!m_generator.setBufferDist(sep_min_str))
        warning = "Bad RegionConfig Line (bad sep_min): " + str;
    else if (!m_generator.addPolygon(region_str))
        warning = "Bad RegionConfig Line (bad region): " + str;

    if (!warning.empty())
        return false;

    std::stringstream ss;
    if (!m_generator.generate(ss))
    {
        warning = "Failed to generate regions with region_config line: " + str;
        return false;
    }

    std::string result = ss.str();

    double sep_min_meters;
    setDoubleOnString(sep_min_meters, sep_min_str);
    sep_min_meters *= MOOSDIST2METERS;

    std::string file_name = "regions_c" + count_str + "_sep" + doubleToStringX(sep_min_meters, 0) + ".txt";
    // find absolute path to moos-ivp-uav folder
    m_region_config_save_path = getenv("HOME");
    m_region_config_save_path += "/moos-ivp-uav/" + save_path;

    std::string file_path = m_region_config_save_path + file_name;
    // save result to file name
    std::ofstream file_out(file_path, std::ios::out);
    file_out << result;
    file_out.close();

    // m_min_sep = generator.getMinSep();

    Logger::info("Generated regions saved to: " + file_path);
    return handleRegionFile(file_path, curr_time, warning);
}

bool IgnoredRegionSet::handleRegionFile(std::string str, double curr_time, std::string &warning)
{
    std::vector<std::string> lines = fileBuffer(str);
    if (lines.size() == 0)
    {
        warning = "File not found, or empty: " + str;
        return (false);
    }

    for (unsigned int i = 0; i < lines.size(); i++)
    {
        std::string orig = lines[i];
        std::string line = stripBlankEnds(stripComment(lines[i], "//"));
        if (line == "")
            continue;

        std::string param = biteStringX(line, '=');
        std::string value = line;

        if (param == "ignoredRegion")
        {
            IgnoredRegion ignoredRegion = stringToIgnoredRegion(value);
            if (!ignoredRegion.isValid())
            {
                warning = "Bad RegionFile Line (Invalid ignoredRegion): " + orig;
                return false;
            }

            std::string name = ignoredRegion.getName();
            if (m_map_ignoredRegions.count(name) != 0)
            {
                warning = "Bad RegionFile Line (name already exist): " + orig;
                return (false);
            }

            Logger::info("IgnoredRegionSet::handleRegionFile ignoredRegion line: " + value);
            // std::string mod_val = findReplace(value, ";",",");
            double spawntime = tokDoubleParse(value, "spawntime");

            Logger::info("IgnoredRegionSet::handleRegionFile spawntime: " + doubleToStringX(spawntime, 2));
            if (spawntime > 0)
            {
                m_vec_spawnable_regions.push_back(std::make_pair(spawntime, value));
                continue;
            }

            ignoredRegion.setTimeEnter(curr_time);
            tagIgnoredRegionID(ignoredRegion);
            m_map_ignoredRegions[name] = ignoredRegion;

            configureIgnoreRegionVisuals(name);
        }
        else if ((param == "search_area") || (param == "poly"))
        {
            std::string _;
            bool ok = handleSearchRegionStr(value, _);
            if (!ok)
            {
                Logger::info("IgnoredRegionSet::handleRegionFile: " + value);
                warning = "Bad RegionFile Line: " + orig;
                return (false);
            }
        }
    }

    m_region_file = str;
    return (true);
}

bool IgnoredRegionSet::handleSearchRegionStr(std::string str, std::string &warning)
{
    m_search_region = string2Poly(str);
    m_search_region.set_color("edge", "gray90");
    m_search_region.set_color("vertex", "dodger_blue");
    m_search_region.set_vertex_size(5);

    if (!m_search_region.is_convex())
    {
        warning = "Bad Search Area String: " + str;
        return (false);
    }

    return true;
}

bool IgnoredRegionSet::configureIgnoreRegionVisuals(std::string rname)
{
    if (m_map_ignoredRegions.count(rname) == 0)
        return false;

    IgnoredRegion ignoredRegion = m_map_ignoredRegions[rname];
    std::string display_name = ignoredRegion.getLabel();

    XYPolygon region = ignoredRegion.getRegion();
    region.set_active(true);
    region.set_label(rname);
    region.set_label_color("off");
    region.set_vertex_color("off");
    region.set_edge_color("off");
    region.set_color("fill", "brown");
    region.set_transparency(0.1);

    XYMarker marker = ignoredRegion.getMarker();
    marker.set_type("efield");
    marker.set_active(true);
    marker.set_label("marker_"+rname);
    marker.set_msg(display_name);
    marker.set_label_color("white");
    marker.set_width(REGION_MARKER_WIDTH);
    marker.set_color("primary_color", "green");
    marker.set_color("secondary_color", "yellow");
    marker.set_transparency(0.1);

    ignoredRegion.setRegion(region);
    ignoredRegion.setMarker(marker);

    return modIgnoredRegion(ignoredRegion);
    ;
}

std::vector<IgnoredRegion> IgnoredRegionSet::tryAddSpawnableRegion(double mission_start_utc, double curr_time_utc)
{
    std::vector<IgnoredRegion> spawned_regions;
    if (m_vec_spawnable_regions.empty())
        return spawned_regions;

    std::vector<std::pair<double, std::string>>::iterator it;
    for (it = m_vec_spawnable_regions.begin(); it != m_vec_spawnable_regions.end();)
    {
        double spawntime = it->first;
        std::string spec_str = it->second;

        double missionDuration = curr_time_utc - mission_start_utc;
        if (missionDuration >= spawntime)
        {
            std::string dummy_str;
            addRegion(spec_str, curr_time_utc, dummy_str);

            if(!dummy_str.empty()){
                Logger::warning("IgnoredRegionSet::tryAddSpawnableRegion: " + dummy_str);
                return spawned_regions;
            }

            
            std::string name = tokStringParse(spec_str, "name");
            
            if(name.empty()){
                Logger::warning("IgnoredRegionSet::tryAddSpawnableRegion: No name in spec_str: " + spec_str);
                return spawned_regions;
            }

            spawned_regions.push_back(m_map_ignoredRegions.at(name));
            it = m_vec_spawnable_regions.erase(it);
        }
        else
            it++;
    }

    return spawned_regions;
}

void IgnoredRegionSet::setMissionStartTimeOnRegions(double v)
{
    for (auto &[_, region] : m_map_ignoredRegions)
        region.setTimeEnter(v);
}

std::vector<IgnoredRegion> IgnoredRegionSet::getRegions() const
{
    std::vector<IgnoredRegion> region_vector;

    std::map<std::string, IgnoredRegion>::const_iterator p;
    for (p = m_map_ignoredRegions.begin(); p != m_map_ignoredRegions.end(); p++)
    {
        IgnoredRegion region = p->second;
        region_vector.push_back(region);
    }

    return (region_vector);
}

std::vector<std::string> IgnoredRegionSet::getIgnoredRegionFileSpec() const
{
    std::vector<std::string> svector;

    if (isSearchRegionValid())
        svector.push_back("poly = " + m_search_region.get_spec_pts());

    std::map<std::string, IgnoredRegion>::const_iterator p;
    for (p = m_map_ignoredRegions.begin(); p != m_map_ignoredRegions.end(); p++)
    {
        std::string name = p->first;
        IgnoredRegion ignoredRegion = p->second;

        std::string line = "ignoredRegion = name=" + name;
        line += ", " + ignoredRegion.getSpecRegion();
        svector.push_back(line);
    }

    return (svector);
}

//---------------------------------------------------------
// Procedure: addRegion()
//   Example: state=discovered, format|hexagon; msg|wood_chop; x|$[XPOS]; y|$[YPOS]; rad|60; pts|8; snap_val|1.0, name=region1
//            state=undiscovered, format|hexagon; msg|wood_chop; x|$[XPOS]; y|$[YPOS]; rad|60; pts|8; snap_val|1.0, name=val

bool IgnoredRegionSet::addRegion(std::string str,
                                 double curr_time,
                                 std::string &warning)
{

    auto total_regions = m_map_ignoredRegions.size() + 1;
    if (total_regions > m_max_size)
    {
        warning = "Region Alert with too many regions";
        return (false);
    }

    std::string rname = tokStringParse(str, "name");
    std::string rstate = tokStringParse(str, "state");
    std::string format = tokStringParse(str, "format");

    rstate = rstate.empty() ? "undiscovered" : rstate; // default state is undiscovered

    if (format.empty())
    {
        warning = "Empty format in Fire Alert: " + str;
        return false;
    }

    IgnoredRegion ignoredRegion = stringToIgnoredRegion(str);
    if (!ignoredRegion.isValid())
    {
        warning = "Bad RegionFile Line (Invalid ignoredRegion): " + str;
        return false;
    }

    if (m_map_ignoredRegions.count(rname) != 0)
    {
        warning = "Bad RegionFile Line (name already exist): " + str;
        return (false);
    }

    ignoredRegion.setTimeEnter(curr_time);
    tagIgnoredRegionID(ignoredRegion);
    m_map_ignoredRegions[rname] = ignoredRegion;

    configureIgnoreRegionVisuals(rname);

    return (true);
}

std::string IgnoredRegionSet::getNameOfIgnoredRegionContaining(double x, double y)
{
    if (m_map_ignoredRegions.size() == 0)
        return ("");

    std::map<std::string, IgnoredRegion>::iterator p;
    for (p = m_map_ignoredRegions.begin(); p != m_map_ignoredRegions.end(); p++)
    {
        std::string name = p->first;
        IgnoredRegion region = p->second;
        if (region.getState() == IgnoredRegion::RegionState::DISCOVERED)
            continue;

        if (region.contains(x, y))
            return name;
    }

    return ("");
}

std::set<std::string> IgnoredRegionSet::getIgnoredRegionNames() const
{
    std::set<std::string> region_names;
    for (const auto &[key, _] : m_map_ignoredRegions)
        region_names.insert(key);

    return (region_names);
}

bool IgnoredRegionSet::modIgnoredRegion(IgnoredRegion region)
{
    std::string name = region.getName();
    if (m_map_ignoredRegions.count(name) == 0)
        return (false);

    m_map_ignoredRegions[name] = region;
    return (true);
}

bool IgnoredRegionSet::hasIgnoredRegion(std::string name) const
{
    if (m_map_ignoredRegions.count(name) == 0)
        return (false);
    return (true);
}

IgnoredRegion IgnoredRegionSet::getIgnoredRegion(std::string name) const
{
    IgnoredRegion null_region;
    if (m_map_ignoredRegions.count(name) == 0)
        return (null_region);

    return (m_map_ignoredRegions.at(name));
}

bool IgnoredRegionSet::hasIgnoredRegionByID(std::string id) const
{
    if (m_map_ignoredRegion_ids.count(id) == 0)
        return (false);
    return (true);
}

IgnoredRegion IgnoredRegionSet::getRegionByID(std::string id) const
{
    IgnoredRegion null_region;
    if (m_map_ignoredRegion_ids.count(id) == 0)
        return (null_region);

    std::string name = m_map_ignoredRegion_ids.at(id);
    if (m_map_ignoredRegions.count(name) == 0)
        return (null_region);

    return (m_map_ignoredRegions.at(name));
}

void IgnoredRegionSet::shuffleIDs()
{
    m_shuffled_ids.clear();

    std::iota(m_shuffled_ids.begin(), m_shuffled_ids.end(), 0);

    random_shuffle(m_shuffled_ids.begin(), m_shuffled_ids.end());
}

void IgnoredRegionSet::tagIgnoredRegionID(IgnoredRegion &region)
{
    std::string new_id;
    if (m_map_ignoredRegions.size() >= m_shuffled_ids.size())
        new_id = "id" + uintToString(m_map_ignoredRegions.size(), 2);
    else
    {
        unsigned int next_ix = m_map_ignoredRegions.size();
        new_id = "id" + uintToString(m_shuffled_ids[next_ix], 2);
    }

    region.setID(new_id);
    m_map_ignoredRegion_ids[new_id] = region.getName();
}

unsigned int IgnoredRegionSet::getTotalIgnoredRegionsDiscovered() const
{
    unsigned int total = 0;
    for (auto const &[_, region] : m_map_ignoredRegions)
    {
        if (region.getState() == IgnoredRegion::RegionState::DISCOVERED)
            total++;
    }

    return (total);
}

unsigned int IgnoredRegionSet::getTotalIgnoredRegionsDiscoveredBy(std::string vname) const
{
    if (vname == "")
        return (0);

    vname = tolower(vname);

    unsigned int total = 0;
    for (auto const &[name, region] : m_map_ignoredRegions)
    {
        if (tolower(region.getDiscoverer()) == vname)
            total++;
    }

    return (total);
}

//------------------------------------------------------------
// Procedure: allRegionsDiscovered()
//   Purpose: Determine if all regions have
//            been discovered.

bool IgnoredRegionSet::allIgnoredRegionsDiscovered() const
{
    for (auto const &[_, region] : m_map_ignoredRegions)
    {
        if (region.getDiscoverer() == "")
            return (false);
    }

    return (true);
}

bool IgnoredRegionSet::removeIgnoreRegion(std::string rname)
{
    // Check if the region exists
    if (m_map_ignoredRegions.count(rname) == 0)
        return false;

    // Find and remove the ID mapping
    std::string id = m_map_ignoredRegions[rname].getID();
    if (!id.empty())
        m_map_ignoredRegion_ids.erase(id);

    // Remove the region from the main map
    m_map_ignoredRegions.erase(rname);

    return true;
}


std::string IgnoredRegionSet::spawnIgnoreRegion(double x, double y, double scale_factor) { 
    
    static unsigned int region_count = 1;
    std::string rname = "ignregion_" + uintToString(region_count++);
    std::string format = m_generator.generateRegionSpec(x, y, scale_factor);
    std::string spec = "name=" + rname + ", format=" + format;
    
    m_vec_spawnable_regions.push_back(std::make_pair(0, spec));
    return rname;
}
