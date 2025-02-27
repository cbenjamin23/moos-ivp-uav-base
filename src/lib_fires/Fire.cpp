/************************************************************/
/*    NAME: Michael Benjamin                                */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: Fire.cpp                                        */
/*    DATE: Apr 2nd, 2022                                   */
/************************************************************/

#include <vector>
#include <iterator>
#include "MBUtils.h"
#include "Fire.h"

using namespace std;

Fire::Fire(string fname)
{
    m_start_x = 0;
    m_start_y = 0;
    m_curr_x = 0;
    m_curr_y = 0;

    m_time_enter = 0;
    m_time_discovered = 0;

    m_state = "burning";
    m_type = "reg";
    m_name = fname;

    m_discover_tries = 0;
    m_scout_tries = 0;
}

void Fire::initXY(double x, double y)
{
    m_start_x = x;
    m_start_y = y;
    m_curr_x = x;
    m_curr_y = y;
}

bool Fire::setState(string str)
{
    str = tolower(str);
    if ((str != "burning") && (str != "discovered"))
        return (false);

    m_state = str;
    return (true);
}

bool Fire::setType(string str)
{
    str = tolower(str);
    if ((str == "fire") || (str == "reg"))
        m_type = "reg";
    else if ((str == "heat") || (str == "unreg"))
        m_type = "unreg";
    else
        return (false);

    return (true);
}

//---------------------------------------------------------
// Procedure: hasBeenScouted()
//      Note: If no vehicle name provided, then the question
//            is whether *anyone* has scouted this swimmer.
//            Otherwise the question is whether this swimmer
//            has been scouted by the given vname.

bool Fire::hasBeenScouted(string vname) const
{
    if (vname == "")
        return (m_set_scouted.size() != 0);

    if (m_set_scouted.count(vname) == 0)
        return (false);

    return (true);
}

string Fire::getSpec() const
{
    string spec = "type=" + m_type + ", name=" + m_name;
    if (m_start_x != 0)
        spec += ", start_x=" + doubleToStringX(m_start_x, 2);
    if (m_start_y != 0)
        spec += ", start_y=" + doubleToStringX(m_start_y, 2);
    if (m_curr_x != 0)
        spec += ", curr_x=" + doubleToStringX(m_curr_x, 2);
    if (m_curr_y != 0)
        spec += ", curr_y=" + doubleToStringX(m_curr_y, 2);
    if (m_time_enter != 0)
        spec += ", time_enter=" + doubleToStringX(m_time_enter, 2);
    if (m_time_discovered != 0)
        spec += ", time_discovered=" + doubleToStringX(m_time_discovered, 2);
    if (m_state != "")
        spec += ", state=" + m_state;
    if (m_discoverer != "")
        spec += ", discoverer=" + m_discoverer;
    if (m_id != "")
        spec += ", id=" + m_id;
    if (m_discover_tries != 0)
        spec += ", discover_tries=" + uintToString(m_discover_tries);
    if (m_scout_tries != 0)
        spec += ", scout_tries=" + uintToString(m_scout_tries);

    return (spec);
}

Fire stringToFire(std::string str)
{
    Fire null_fire;
    Fire fire;

    vector<string> svector = parseString(str, ',');
    for (unsigned int i = 0; i < svector.size(); i++)
    {
        string param = tolower(biteStringX(svector[i], '='));
        string value = svector[i];
        double dval = atof(value.c_str());

        if (param == "start_x")
            fire.setStartX(dval);
        else if (param == "start_y")
            fire.setStartY(dval);
        else if (param == "x")
        {
            fire.setStartX(dval);
            fire.setCurrX(dval);
        }
        else if (param == "y")
        {
            fire.setStartY(dval);
            fire.setCurrY(dval);
        }
        else if (param == "curr_x")
            fire.setCurrX(dval);
        else if (param == "curr_y")
            fire.setCurrY(dval);
        else if (param == "time_enter")
            fire.setTimeEnter(dval);
        else if (param == "time_discovered")
            fire.setTimeDiscovered(dval);
        else if (param == "type")
            fire.setType(value);
        else if (param == "name")
            fire.setName(value);
        else if (param == "id")
            fire.setID(value);
        else if (param == "state")
            fire.setState(value);
        else if (param == "discoverer")
            fire.setDiscoverer(value);
        else if (param == "discover_tries")
            fire.setDiscoverTries((unsigned int)(dval));
        else if (param == "scout_tries")
            fire.setScoutTries((unsigned int)(dval));
    }

    return (fire);
}
