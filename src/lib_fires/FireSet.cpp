/*****************************************************************/
/*    NAME: Michael Benjamin, modified by Steve Nomeny           */
/*    ORGN: Mechanical Eng / CSAIL, MIT Cambridge MA             */
/*    FILE: FireSet.cpp                                          */
/*    DATE: February 2022                                        */
/*****************************************************************/

#include <iterator>

#include <numeric>
#include <algorithm>
#include <cmath>
#include <fstream>

#include "FireSet.h"
#include "ColorParse.h"
#include "MBUtils.h"
#include "FileBuffer.h"
#include "XYFormatUtilsPoly.h"

#include "FireFldGenerator.h"

#include "Logger.h"
#include "common.h"

FireSet::FireSet()
{
    m_max_size = 99;
    m_min_sep = 0;
    shuffleIDs();
}

bool FireSet::reset(double curr_time){ 
    if(m_fire_config_str.empty()) 
        return false;
    
    std::string _ = "";
    FireSet temp;
    temp.handleFireConfig(m_fire_config_str, curr_time, _); 

    *this = temp;
    return true;
}


// Format:  generate = true, file = fire.txt, count = 10, sep_min = 10, \
            region = {x0,y0:x1,y1:...:x2,y2}, save_path = "missions/UAV_FLY/gen_fires/", \
            spawn_count=10, spawn_interval = 200:400

bool FireSet::handleFireConfig(std::string str, double curr_time, std::string &warning)
{

    std::string generate_str = tokStringParse(str, "generate");
    bool generate = false;
    setBooleanOnString(generate, generate_str);

    std::string file = tokStringParse(str, "file");

    if (!generate && file.empty())
    {
        warning = "Bad FireConfig Line (need a file if not generating): " + str;
        return false;
    }

    m_fire_config_str = str;

    if (!generate)
        return handleFireFile(file, curr_time, warning);

    // Will generate fires

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
        warning = "Bad FireConfig Line (need count w/ generating): " + str;
    else if (sep_min_str.empty())
        warning = "Bad FireConfig Line (need sep_min w/ generating): " + str;
    else if (region_str.empty())
        warning = "Bad FireConfig Line (need region w/ generating): " + str;
    else if (save_path.empty())
        warning = "Bad FireConfig Line (need save_path w/ generating): " + str;
    else if ((spawn_count > 0) && spawn_interval_str.empty())
        warning = "Bad FireConfig Line (need spawn_interval w/ spawn_count): " + str;

    if (!warning.empty())
        return false;

    FireFldGenerator generator;

    if (!generator.setSpawnInterval(spawn_interval_str))
        warning = "Bad FireConfig Line (bad spawn_interval): " + str;
    else if (!generator.setFireAmt(count_str))
        warning = "Bad FireConfig Line (bad count): " + str;
    else if (!generator.setSpawnableFireAmt(spawn_count_str))
        warning = "Bad FireConfig Line (bad spawn_count): " + str;
    else if (!generator.setBufferDist(sep_min_str))
        warning = "Bad FireConfig Line (bad sep_min): " + str;
    else if (!generator.addPolygon(region_str))
        warning = "Bad FireConfig Line (bad region): " + str;

    if (!warning.empty())
        return false;

    std::stringstream ss;
    if (!generator.generate(ss))
    {
        warning = "Failed to generate fires with fire_config line: " + str;
        return false;
    }

    std::string result = ss.str();

    double sep_min_meters;
    setDoubleOnString(sep_min_meters, sep_min_str);
    sep_min_meters *= MOOSDIST2METERS;

    std::string file_name = "fires_c" + count_str + "_sep" + doubleToStringX(sep_min_meters, 0) + ".txt";
    // find absolute path to moos-ivp-uav folder
    m_fire_config_save_path = getenv("HOME");
    m_fire_config_save_path += "/moos-ivp-uav/" + save_path;

    std::string file_path = m_fire_config_save_path + file_name;
    // save result to file name
    std::ofstream file_out(file_path, std::ios::out);
    file_out << result;
    file_out.close();

    m_min_sep = generator.getMinSep();

    Logger::info("Generated fires saved to: " + file_path);
    return handleFireFile(file_path, curr_time, warning);
}

bool FireSet::handleFireFile(std::string str, double curr_time, std::string &warning)
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

        if (param == "fire")
        {

            Fire fire = stringToFire(value);
            std::string fname = fire.getName();
            if (m_map_fires.count(fname) != 0)
            {
                warning = "Bad FireFile Line (fname already exist): " + orig;
                return (false);
            }

            double spawntime = tokDoubleParse(value, "spawntime");
            if (spawntime > 0)
            {
                m_vec_spawnable_fires.push_back(std::make_pair(spawntime, value));
                continue;
            }

            fire.setTimeEnter(curr_time);

            tagFireID(fire);
            m_map_fires[fname] = fire;
        }
        else if ((param == "search_area") || (param == "poly"))
        {
            std::string _;
            bool ok = handleSearchRegionStr(value, _);
            if (!ok)
            {
                Logger::info("FireSet::handleFireFile: " + value);
                warning = "Bad FireFile Line: " + orig;
                return (false);
            }
        }
    }

    m_fire_file = str;
    return (true);
}

bool FireSet::handleSearchRegionStr(std::string str, std::string &warning)
{
    m_search_region = string2Poly(str);
    m_search_region.set_color("edge", "gray90");
    m_search_region.set_color("vertex", "dodger_blue");
    m_search_region.set_vertex_size(5);

    // Logger::info("FireSet::handleSearchRegionStr: " + str);
    // Logger::info("FireSet Search reagion convex: " + boolToString(m_search_region.is_convex()));
    if (!m_search_region.is_convex())
    {
        warning = "Bad Search Reagion std::String: " + str;
        return (false);
    }

    return true;
}

std::vector<Fire> FireSet::tryAddSpawnableFire(double mission_start_utc, double curr_time_utc)
{

    std::vector<Fire> spawned_fires;
    if (m_vec_spawnable_fires.empty())
        return spawned_fires;

    std::vector<std::pair<double, std::string>>::iterator it;
    for (it = m_vec_spawnable_fires.begin(); it != m_vec_spawnable_fires.end();)
    {
        double spawntime = it->first;
        std::string fire_str = it->second;

        double missionDuration = curr_time_utc - mission_start_utc;
        if (missionDuration >= spawntime)
        {
            std::string dummy_str;
            fireAlert(fire_str, curr_time_utc, dummy_str);

            std::string fname = tokStringParse(fire_str, "name");
            spawned_fires.push_back(m_map_fires.at(fname));

            it = m_vec_spawnable_fires.erase(it);
        }
        else
            it++;
    }

    return spawned_fires;
}

void FireSet::setMissionStartTimeOnFires(double v)
{
    for (auto &[_, fire] : m_map_fires)
        fire.setTimeEnter(v);
}

std::vector<Fire> FireSet::getFires() const
{
    std::vector<Fire> fvector;

    std::map<std::string, Fire>::const_iterator p;
    for (p = m_map_fires.begin(); p != m_map_fires.end(); p++)
    {
        Fire fire = p->second;
        fvector.push_back(fire);
    }

    return (fvector);
}

std::vector<XYPoint> FireSet::getFirePoints() const
{
    std::vector<XYPoint> fire_pos;

    std::map<std::string, Fire>::const_iterator p;
    for (p = m_map_fires.begin(); p != m_map_fires.end(); p++)
    {
        Fire fire = p->second;
        double fire_x = fire.getCurrX();
        double fire_y = fire.getCurrY();
        fire_pos.emplace_back(fire_x, fire_y);
    }

    return (fire_pos);
}

std::vector<std::string> FireSet::getFireFileSpec() const
{
    std::vector<std::string> svector;

    if (isSearchRegionValid())
        svector.push_back("poly = " + m_search_region.get_spec_pts());

    std::map<std::string, Fire>::const_iterator p;
    for (p = m_map_fires.begin(); p != m_map_fires.end(); p++)
    {
        std::string name = p->first;
        Fire fire = p->second;
        double start_x = fire.getStartX();
        double start_y = fire.getStartX();
        std::string x_str = doubleToStringX(start_x);
        std::string y_str = doubleToStringX(start_y);

        std::string line = "fire = name=" + name;
        line += ", x=" + x_str + ", y=" + y_str;
        svector.push_back(line);
    }

    return (svector);
}

//---------------------------------------------------------
// Procedure: fireAlert()
//   Example: state=discovered, x=2, y=3, name=joe
//            state=undiscovered, x=2, y=3
//      Note: XFIRE_ALERT

bool FireSet::fireAlert(std::string str, double curr_time, std::string &warning)
{
    std::string fname = tokStringParse(str, "name");
    std::string fstate = tokStringParse(str, "state");
    double x = tokDoubleParse(str, "x");
    double y = tokDoubleParse(str, "y");

    fstate = fstate.empty() ? "undiscovered" : fstate; // default state is undiscovered

    return (addFire(fname, fstate, x, y, curr_time, warning));
}

bool FireSet::addFire(std::string fname, std::string fstate,
                      double x, double y,
                      double curr_time,
                      std::string &warning)
{
    if ((stringToFireState(fstate) == Fire::FireState::UNKNOWN))
    {
        warning = "Fire Alert with unknown state:[" + fstate + "]";
        return (false);
    }

    if ((fname != "") && (m_map_fires.count(fname)))
    {
        warning = "Fire Alert with duplicate name:" + fname;
        return (false);
    }

    auto total_fires = m_map_fires.size() + 1;
    if (total_fires > m_max_size)
    {
        warning = "Fire Alert with too many fires";
        return (false);
    }

    if (fname == "")
        fname = "f" + uintToString(total_fires, 2);

    Fire new_fire(fname);
    new_fire.initXY(x, y);
    new_fire.setStateFromString(fstate);
    new_fire.setTimeEnter(curr_time);
    tagFireID(new_fire);

    m_map_fires[fname] = new_fire;

    return (true);
}

std::string FireSet::getNameClosestFire(double x, double y, double min_range)
{
    if (m_map_fires.size() == 0)
        return ("");

    std::string closest_fname;
    double closest_range = -1;

    std::map<std::string, Fire>::iterator p;
    for (p = m_map_fires.begin(); p != m_map_fires.end(); p++)
    {
        std::string fname = p->first;
        Fire fire = p->second;
        if (fire.getState() == Fire::FireState::DISCOVERED)
            continue;

        double curr_x = fire.getCurrX();
        double curr_y = fire.getCurrY();
        double range = hypot(x - curr_x, y - curr_y);
        if (range > min_range)
            continue;

        if ((closest_range < 0) || (range < closest_range))
        {
            closest_fname = fname;
            closest_range = range;
        }
    }

    return (closest_fname);
}

std::set<std::string> FireSet::getFireNames() const
{
    std::set<std::string> fire_names;
    for (const auto &[key, _] : m_map_fires)
        fire_names.insert(key);

    return (fire_names);
}

bool FireSet::modFire(Fire fire)
{
    std::string fname = fire.getName();
    if (m_map_fires.count(fname) == 0)
        return (false);

    m_map_fires[fname] = fire;
    return (true);
}

bool FireSet::hasFire(std::string fname) const
{
    if (m_map_fires.count(fname) == 0)
        return (false);
    return (true);
}

Fire FireSet::getFire(std::string fname) const
{
    Fire null_fire;
    if (m_map_fires.count(fname) == 0)
        return (null_fire);

    return (m_map_fires.at(fname));
}

bool FireSet::hasFireByID(std::string id) const
{
    if (m_map_fire_ids.count(id) == 0)
        return (false);
    return (true);
}

Fire FireSet::getFireByID(std::string id) const
{
    Fire null_fire;
    if (m_map_fire_ids.count(id) == 0)
        return (null_fire);

    std::string fname = m_map_fire_ids.at(id);
    if (m_map_fires.count(fname) == 0)
        return (null_fire);

    return (m_map_fires.at(fname));
}

void FireSet::shuffleIDs()
{
    m_shuffled_ids.clear();

    std::iota(m_shuffled_ids.begin(), m_shuffled_ids.end(), 0);

    random_shuffle(m_shuffled_ids.begin(), m_shuffled_ids.end());
}

void FireSet::tagFireID(Fire &fire)
{
    std::string new_id;
    if (m_map_fires.size() >= m_shuffled_ids.size())
        new_id = "id" + uintToString(m_map_fires.size(), 2);
    else
    {
        unsigned int next_ix = m_map_fires.size();
        new_id = "id" + uintToString(m_shuffled_ids[next_ix], 2);
    }

    fire.setID(new_id);
    m_map_fire_ids[new_id] = fire.getName();
}

unsigned int FireSet::getTotalFiresDiscovered() const
{
    unsigned int total = 0;
    for (auto const &[_, fire] : m_map_fires)
    {
        if (fire.getState() == Fire::FireState::DISCOVERED)
            total++;
    }

    return (total);
}
unsigned int FireSet::getTotalFiresDiscoveredBy(std::string vname) const
{
    if (vname == "")
        return (0);

    vname = tolower(vname);

    unsigned int total = 0;
    for (auto const &[fname, fire] : m_map_fires)
    {
        if (tolower(fire.getDiscoverer()) == vname)
            total++;
    }

    return (total);
}

//------------------------------------------------------------
// Procedure: allFiresDiscovered()
//   Purpose: Determine if all fires have
//            been discovered.

bool FireSet::allFiresDiscovered() const
{
    for (auto const &[_, fire] : m_map_fires)
    {
        if (fire.getDiscoverer() == "")
            return (false);
    }

    return (true);
}
