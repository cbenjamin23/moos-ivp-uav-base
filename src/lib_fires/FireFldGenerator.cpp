/*****************************************************************/
/*    NAME: Michael Benjamin, modified by Steve Nomeny           */
/*    ORGN: Mechanical Eng / CSAIL, MIT Cambridge MA, NTNU       */
/*    FILE: FireFldGenerator.cpp                                 */
/*    DATE: February 2025                                        */
/*****************************************************************/

#include <unistd.h>
#include <iostream>
#include <cstdlib>
#include <cmath>
#include "FireFldGenerator.h"
#include "MBUtils.h"

using namespace std;

FireFldGenerator::FireFldGenerator()
{
    m_fire_amt = 1;
    m_unreg_amt = 0;
    m_buffer_dist = 10;
}

bool FireFldGenerator::setFireAmt(string amt)
{
    return (setUIntOnString(m_fire_amt, amt));
}

bool FireFldGenerator::setUnregAmt(string amt)
{
    return (setUIntOnString(m_unreg_amt, amt));
}

bool FireFldGenerator::setBufferDist(string str)
{
    return (setNonNegDoubleOnString(m_buffer_dist, str));
}

bool FireFldGenerator::generate()
{
    unsigned int total = m_fire_amt + m_unreg_amt;
    if (total == 0)
    {
        cout << "No fires requested. No fires generated." << endl;
        return (false);
    }
    if (m_generator.size() == 0)
    {
        cout << "No region specified. No fires generated" << endl;
        return (false);
    }

    // Seed the random number generator
    unsigned long tseed = time(NULL) + 1;
    unsigned long pid = (long)getpid() + 1;
    unsigned long seed = (tseed % 999999);
    seed = ((rand())*seed) % 999999;
    seed = (seed * pid) % 999999;
    srand(seed);

    m_generator.setSnap(1);
    if (total > 50)
        m_generator.setSnap(0.1);

    m_generator.setBufferDist(m_buffer_dist);
    m_generator.setFlexBuffer(true);
    m_generator.generatePoints(total);
    vector<XYPoint> points = m_generator.getPoints();
    if (points.size() != total)
        return (false);

    // Output results
    double nearest = m_generator.getGlobalNearest();
    cout << "// Lowest dist between fires: ";
    cout << doubleToString(nearest, 2) << endl;
    for (unsigned int i = 0; i < m_generator.size(); i++)
    {
        string poly_spec = m_generator.getPolygon(i).get_spec(4);
        cout << "poly = " << poly_spec << endl;
    }

    // Output registered fires
    for (unsigned int i = 0; i < m_fire_amt; i++)
    {
        string fire_name = "f";
        if (i + 1 < 10)
            fire_name += "0";
        fire_name += uintToString(i + 1);
        double xval = points[i].get_vx();
        double yval = points[i].get_vy();
        if (m_unreg_amt > 0)
            cout << "fire = type=reg, name=" << fire_name;
        else
            cout << "fire = name=" << fire_name;
        cout << ", x=" << doubleToStringX(xval, 2);
        cout << ", y=" << doubleToStringX(yval, 2) << endl;
    }

    // Output unreg fires
    for (unsigned int i = m_fire_amt; i < points.size(); i++)
    {
        string unreg_name = "h"; // h for heat signature
        unsigned int index = (i - m_fire_amt) + 1;
        if (index < 10)
            unreg_name += "0";
        unreg_name += uintToString(index);
        double xval = points[i].get_vx();
        double yval = points[i].get_vy();
        cout << "fire = type=unreg, name=" << unreg_name;
        cout << ", x=" << doubleToStringX(xval, 2);
        cout << ", y=" << doubleToStringX(yval, 2) << endl;
    }

    return (true);
}
