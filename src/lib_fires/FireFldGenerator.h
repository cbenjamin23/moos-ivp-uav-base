#pragma once

#include <string>
#include "XYFieldGenerator.h"
#include "XYPolygon.h"

#include <sstream>

class FireFldGenerator
{
public:
    FireFldGenerator();
    virtual ~FireFldGenerator() {}

    bool setFireAmt(std::string);
    bool setSpawnableFireAmt(std::string);
    bool setSpawnInterval(std::string);
    bool setBufferDist(std::string);

    bool addPolygon(std::string s) { return (m_generator.addPolygon(s)); }
    bool addPolygon(XYPolygon poly) { return (m_generator.addPolygon(poly)); }

    bool generate(std::stringstream& ss);
    
    double getMinSep() { return (m_buffer_dist); };

protected: // Config variables
    unsigned int m_fire_amt;
    unsigned int m_spawnable_fire_amt;
    unsigned int m_spawn_tmin, m_spawn_tmax;
    
    double m_buffer_dist;

protected: // State variables
    XYFieldGenerator m_generator;
};
