/*****************************************************************/
/*    NAME: Michael Benjamin, modified by Steve Nomeny           */
/*    ORGN: Mechanical Eng / CSAIL, MIT Cambridge MA, NTNU       */
/*    FILE: IgnoredRegionGenerator.h                             */
/*    DATE: February 2025                                        */
/*****************************************************************/

#ifndef IGNORED_REGION_GENERATOR_HEADER
#define IGNORED_REGION_GENERATOR_HEADER

#include "XYPolygon.h"
#include "XYFieldGenerator.h"
#include "XYMarker.h"
#include "IgnoredRegion.h"
#include <string>
#include <vector>
#include <sstream>

class IgnoredRegionGenerator
{
public:
    IgnoredRegionGenerator();
    virtual ~IgnoredRegionGenerator() {}

    bool setRegionAmt(std::string);
    bool setSpawnableRegionAmt(std::string);
    bool setSpawnInterval(std::string);
    bool setBufferDist(std::string);
    bool addPolygon(std::string);

    bool generate(std::stringstream &);
    double getMinSep() const { return (m_buffer_dist); }

    // Generate a region specification at given location
    std::string generateRegionSpec(double x, double y, double scale_factor);

protected:
    // Helper methods to generate specific shape specifications
    std::string generateEllipseSpec(double x, double y, double scale_factor);
    std::string generateRadialSpec(double x, double y, double scale_factor);
    std::string generateOvalSpec(double x, double y, double scale_factor);
    std::string generateHexagonSpec(double x, double y, double scale_factor);
    std::string generateRectangleSpec(double x, double y, double scale_factor);

    // Helper to generate a random shape name appropriate for the type
    std::string randomShapeName(const IgnoredRegion::RegionType type) const;

    // Get a random region type from the RegionType enum
    IgnoredRegion::RegionType getRandomRegionType() const;

protected:
    // Generator from base class
    XYFieldGenerator m_generator;
    
    // Parameters
    unsigned int m_region_amt;
    unsigned int m_spawnable_region_amt;
    double m_buffer_dist;
    unsigned int m_spawn_tmin;
    unsigned int m_spawn_tmax;

    // Size parameters for generated regions
    double m_min_region_size;
    double m_max_region_size;

    // Parameter ranges for different shapes
    // Ellipse
    double m_ellipse_major_min;
    double m_ellipse_major_max;
    double m_ellipse_minor_min;
    double m_ellipse_minor_max;

    // Radial (circle)
    double m_radial_radius_min;
    double m_radial_radius_max;

    // Oval
    double m_oval_rad_min;
    double m_oval_rad_max;
    double m_oval_len_min;
    double m_oval_len_max;

    // Hexagon
    double m_hexagon_rad_min;
    double m_hexagon_rad_max;

    // Rectangle
    double m_rectangle_width_min;
    double m_rectangle_width_max;
    double m_rectangle_height_min;
    double m_rectangle_height_max;

    // Common parameters
    unsigned int m_min_points;
    unsigned int m_max_points;
    double m_max_rotation_deg;
};

#endif
