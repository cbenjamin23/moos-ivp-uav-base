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

#include "common.h"

using namespace std;

FireFldGenerator::FireFldGenerator()
{
    m_fire_amt = 1;
    m_buffer_dist = 200;
}

bool FireFldGenerator::setFireAmt(string amt)
{
    return (setUIntOnString(m_fire_amt, amt));
}


bool FireFldGenerator::setBufferDist(string str)
{
    return (setNonNegDoubleOnString(m_buffer_dist, str));
}

bool FireFldGenerator::generate(std::stringstream& out)
{
    unsigned int total = m_fire_amt;
    if (total == 0)
    {
        out << "No fires requested. No fires generated." << endl;
        return (false);
    }
    if (m_generator.size() == 0)
    {
        out << "No region specified. No fires generated" << endl;
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
    out << "// Lowest dist between fires: ";
    out << doubleToString(nearest * MOOSDIST2METERS, 2) << endl;
    for (unsigned int i = 0; i < m_generator.size(); i++)
    {
        string poly_spec = m_generator.getPolygon(i).get_spec(4);
        out << "poly = " << poly_spec << endl;
    }

    // Output fires
    for (unsigned int i = 0; i < m_fire_amt; i++)
    {
        string fire_name = "f";
        if (i + 1 < 10)
            fire_name += "0";
        fire_name += uintToString(i + 1);
        double xval = points[i].get_vx();
        double yval = points[i].get_vy();
        
        out << "fire = name=" << fire_name;
        out << ", x=" << doubleToStringX(xval, 2);
        out << ", y=" << doubleToStringX(yval, 2) << endl;
    }


    return (true);
}
