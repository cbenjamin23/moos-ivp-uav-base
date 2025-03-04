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

#include "FireSet.h"
#include "ColorParse.h"
#include "MBUtils.h"
#include "FileBuffer.h"
#include "XYFormatUtilsPoly.h"

using namespace std;

FireSet::FireSet()
{
    m_max_size = 99;
    shuffleIDs();
}

bool FireSet::handleFireFile(string str, double curr_time, string &warning)
{
    vector<string> lines = fileBuffer(str);
    if (lines.size() == 0)
    {
        warning = "File not found, or empty: " + str;
        return (false);
    }

    for (unsigned int i = 0; i < lines.size(); i++)
    {
        string orig = lines[i];
        string line = stripBlankEnds(stripComment(lines[i], "//"));
        if (line == "")
            continue;

        string param = biteStringX(line, '=');
        string value = line;

        if (param == "fire")
        {
            Fire fire = stringToFire(value);
            string fname = fire.getName();
            if (m_map_fires.count(fname) != 0)
            {
                warning = "Bad FireFile Line: " + orig;
                return (false);
            }

            fire.setTimeEnter(curr_time);
            tagFireID(fire);
            m_map_fires[fname] = fire;
        }
        else if ((param == "region") || (param == "poly"))
        {
            m_search_region = string2Poly(value);
            m_search_region.set_color("edge", "gray90");
            m_search_region.set_color("vertex", "dodger_blue");
            m_search_region.set_vertex_size(5);
            if (!m_search_region.is_convex())
            {
                warning = "Bad FireFile Line: " + orig;
                return (false);
            }
        }
    }

    m_fire_file = str;
    return (true);
}

vector<Fire> FireSet::getFires() const
{
    vector<Fire> fvector;

    map<string, Fire>::const_iterator p;
    for (p = m_map_fires.begin(); p != m_map_fires.end(); p++)
    {
        Fire fire = p->second;
        fvector.push_back(fire);
    }

    return (fvector);
}

vector<string> FireSet::getFireFileSpec() const
{
    vector<string> svector;

    if (isSearchRegionValid())
        svector.push_back("poly = " + m_search_region.get_spec_pts());

    map<string, Fire>::const_iterator p;
    for (p = m_map_fires.begin(); p != m_map_fires.end(); p++)
    {
        string name = p->first;
        Fire fire = p->second;
        double start_x = fire.getStartX();
        double start_y = fire.getStartX();
        string x_str = doubleToStringX(start_x);
        string y_str = doubleToStringX(start_y);

        string line = "fire = name=" + name;
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

bool FireSet::fireAlert(string str, double curr_time, string &warning)
{
    string fname = tokStringParse(str, "name");
    string fstate = tokStringParse(str, "state");
    double x = tokDoubleParse(str, "x");
    double y = tokDoubleParse(str, "y");

    fstate = fstate.empty() ? "undiscovered" : fstate; // default state is undiscovered

    return (addFire(fname, fstate, x, y, curr_time, warning));
}

bool FireSet::addFire(string fname, string fstate,
                      double x, double y,
                      double curr_time,
                      string &warning)
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

string FireSet::getNameClosestFire(double x, double y, double min_range)
{
    if (m_map_fires.size() == 0)
        return ("");

    string closest_fname;
    double closest_range = -1;

    map<string, Fire>::iterator p;
    for (p = m_map_fires.begin(); p != m_map_fires.end(); p++)
    {
        string fname = p->first;
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

set<string> FireSet::getFireNames() const
{
    set<string> fire_names;
    for (const auto &[key, _] : m_map_fires)
        fire_names.insert(key);

    return (fire_names);
}

bool FireSet::modFire(Fire fire)
{
    string fname = fire.getName();
    if (m_map_fires.count(fname) == 0)
        return (false);

    m_map_fires[fname] = fire;
    return (true);
}

bool FireSet::hasFire(string fname) const
{
    if (m_map_fires.count(fname) == 0)
        return (false);
    return (true);
}

Fire FireSet::getFire(string fname) const
{
    Fire null_fire;
    if (m_map_fires.count(fname) == 0)
        return (null_fire);

    return (m_map_fires.at(fname));
}

bool FireSet::hasFireByID(string id) const
{
    if (m_map_fire_ids.count(id) == 0)
        return (false);
    return (true);
}

Fire FireSet::getFireByID(string id) const
{
    Fire null_fire;
    if (m_map_fire_ids.count(id) == 0)
        return (null_fire);

    string fname = m_map_fire_ids.at(id);
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
    string new_id;
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
unsigned int FireSet::getTotalFiresDiscoveredBy(string vname) const
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
