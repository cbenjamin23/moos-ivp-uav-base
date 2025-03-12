/*****************************************************************/
/*    NAME: Michael Benjamin, modified by Steve Nomeny           */
/*    ORGN: Mechanical Eng / CSAIL, MIT Cambridge MA, NTNU       */
/*    FILE: IgnoredRegionGenerator.cpp                           */
/*    DATE: February 2025                                        */
/*****************************************************************/

#include <unistd.h>
#include <iostream>
#include <cstdlib>
#include <cmath>
#include <algorithm>

#include "IgnoredRegionGenerator.h"
#include "MBUtils.h"
#include "XYFormatUtilsPoly.h"

#include "common.h"



using namespace std;

IgnoredRegionGenerator::IgnoredRegionGenerator()
{
    m_region_amt = 1;
    m_spawnable_region_amt = 0;
    m_spawn_tmin = 0;
    m_spawn_tmax = 0;
    m_buffer_dist = 200;
    m_min_region_size = 10;
    m_max_region_size = 40;

    // Initialize shape-specific parameters
    
    // Ellipse parameters
    m_ellipse_major_min = 100;
    m_ellipse_major_max = 300;
    m_ellipse_minor_min = 50;
    m_ellipse_minor_max = 150;

    // Radial (circle) parameters
    m_radial_radius_min = 30;
    m_radial_radius_max = 80;

    // Oval parameters
    m_oval_rad_min = 50;
    m_oval_rad_max = 120;
    m_oval_len_min = 150;
    m_oval_len_max = 300;

    // Hexagon parameters
    m_hexagon_rad_min = 40;
    m_hexagon_rad_max = 100;

    // Rectangle parameters
    m_rectangle_width_min = 100;
    m_rectangle_width_max = 300;
    m_rectangle_height_min = 150;
    m_rectangle_height_max = 400;

    // Common parameters
    m_min_points = 8;
    m_max_points = 16;
    m_max_rotation_deg = 270;
}

bool IgnoredRegionGenerator::setRegionAmt(string amt)
{
    return (setUIntOnString(m_region_amt, amt));
}

bool IgnoredRegionGenerator::setSpawnableRegionAmt(string amt)
{
    return (setUIntOnString(m_spawnable_region_amt, amt));
}

// Format: 200:400
bool IgnoredRegionGenerator::setSpawnInterval(string interval)
{
    std::string tmin = biteStringX(interval, ':');
    std::string tmax = interval;
    if (tmin.empty() || tmax.empty())
        return (false);

    if (!(setUIntOnString(m_spawn_tmin, tmin) && setUIntOnString(m_spawn_tmax, tmax)))
        return (false);

    if (m_spawn_tmin > m_spawn_tmax)
        return (false);

    return true;
}

bool IgnoredRegionGenerator::setBufferDist(string str)
{
    return (setNonNegDoubleOnString(m_buffer_dist, str));
}

bool IgnoredRegionGenerator::addPolygon(string str)
{
    return (m_generator.addPolygon(str));
}

string IgnoredRegionGenerator::randomShapeName(const IgnoredRegion::RegionType type) const
{
    // Return a random name appropriate for the shape type
    switch (type)
    {
    case IgnoredRegion::RegionType::ELLIPSE:
    {
        const vector<string> names = {"lake", "pond", "water", "lagoon", "bay"};
        return names[rand() % names.size()];
    }
    case IgnoredRegion::RegionType::RADIAL:
    {
        const vector<string> names = {"building", "tower", "silo", "well", "bunker"};
        return names[rand() % names.size()];
    }
    case IgnoredRegion::RegionType::OVAL:
    {
        const vector<string> names = {"track", "field", "stadium", "oval_area", "court"};
        return names[rand() % names.size()];
    }
    case IgnoredRegion::RegionType::HEXAGON:
    {
        const vector<string> names = {"garden", "patch", "hex_zone", "hive", "plaza"};
        return names[rand() % names.size()];
    }
    case IgnoredRegion::RegionType::RECTANGLE:
    {
        const vector<string> names = {"building", "parking", "hangar", "warehouse", "block"};
        return names[rand() % names.size()];
    }
    default:
        return "region";
    }
}

IgnoredRegion::RegionType IgnoredRegionGenerator::getRandomRegionType() const
{
    // Choose a random region type from the enum
    // We'll use a number between 0 and 4 (since there are 5 types)
    int randType = rand() % 5;
    switch (randType)
    {
    case 0:
        return IgnoredRegion::RegionType::ELLIPSE;
    case 1:
        return IgnoredRegion::RegionType::RADIAL;
    case 2:
        return IgnoredRegion::RegionType::OVAL;
    case 3:
        return IgnoredRegion::RegionType::HEXAGON;
    case 4:
        return IgnoredRegion::RegionType::RECTANGLE;
    default:
        return IgnoredRegion::RegionType::HEXAGON; // Default case
    }
}

string IgnoredRegionGenerator::generateEllipseSpec(double x, double y, double scale_factor)
{

    double major = m_ellipse_major_min + ((double)rand() / RAND_MAX) * (m_ellipse_major_max - m_ellipse_major_min);
    double minor = m_ellipse_minor_min + ((double)rand() / RAND_MAX) * (m_ellipse_minor_max - m_ellipse_minor_min);

    major *= scale_factor;
    minor *= scale_factor;

    unsigned int pts = m_min_points + rand() % (m_max_points - m_min_points + 1);
    double degs = ((double)rand() / RAND_MAX) * m_max_rotation_deg;
    double snap = 0.5 + ((double)rand() / RAND_MAX);

    string msg = randomShapeName(IgnoredRegion::RegionType::ELLIPSE);

    // Format: "format|ellipse; msg|val; x|val; y|val; major|val; minor|val; pts|val; degs|val; snap_value|val"
    string spec = "format|ellipse; msg|" + msg + "; x|" + doubleToStringX(x, 2) +
                  "; y|" + doubleToStringX(y, 2) + "; major|" + doubleToStringX(major, 2) +
                  "; minor|" + doubleToStringX(minor, 2) + "; pts|" + intToString(pts) +
                  "; degs|" + doubleToStringX(degs, 2) + "; snap_value|" + doubleToStringX(snap, 1);

    return spec;
}

string IgnoredRegionGenerator::generateRadialSpec(double x, double y, double scale_factor)
{

    double radius = m_radial_radius_min + ((double)rand() / RAND_MAX) * (m_radial_radius_max - m_radial_radius_min);
    radius *= scale_factor;

    unsigned int pts = m_min_points + rand() % (m_max_points - m_min_points + 1);
    double snap = 0.5 + ((double)rand() / RAND_MAX);

    string msg = randomShapeName(IgnoredRegion::RegionType::RADIAL);

    // Format: "format|radial; msg|val; x|val; y|val; radius|val; pts|val; snap|val"
    string spec = "format|radial; msg|" + msg + "; x|" + doubleToStringX(x, 2) +
                  "; y|" + doubleToStringX(y, 2) + "; radius|" + doubleToStringX(radius, 2) +
                  "; pts|" + intToString(pts) + "; snap|" + doubleToStringX(snap, 1);

    return spec;
}

string IgnoredRegionGenerator::generateOvalSpec(double x, double y, double scale_factor)
{

    double rad = m_oval_rad_min + ((double)rand() / RAND_MAX) * (m_oval_rad_max - m_oval_rad_min);
    double len = m_oval_len_min + ((double)rand() / RAND_MAX) * (m_oval_len_max - m_oval_len_min);

    rad *= scale_factor;
    len *= scale_factor;

    // Ensure len > 2*rad as required
    if (len <= 2 * rad)
    {
        len = 2.1 * rad;
    }

    unsigned int draw_degs = 5 + rand() % 15; // Between 5 and 20

    string msg = randomShapeName(IgnoredRegion::RegionType::OVAL);

    // Format: "format|oval; msg=val; x=val; y=val; rad=val; len=val; draw_degs=val"
    string spec = "format|oval; msg|" + msg + "; x|" + doubleToStringX(x, 2) +
                  "; y|" + doubleToStringX(y, 2) + "; rad|" + doubleToStringX(rad, 2) +
                  "; len|" + doubleToStringX(len, 2) + "; draw_degs|" + intToString(draw_degs);

    return spec;
}

string IgnoredRegionGenerator::generateHexagonSpec(double x, double y, double scale_factor)
{

    double rad = m_hexagon_rad_min + ((double)rand() / RAND_MAX) * (m_hexagon_rad_max - m_hexagon_rad_min);
    rad *= scale_factor;

    unsigned int pts = 6 + rand() % 5; // Between 6 and 10 points (hexagon to decagon)
    double snap_val = 0.5 + ((double)rand() / RAND_MAX);

    string msg = randomShapeName(IgnoredRegion::RegionType::HEXAGON);

    // Format: "format|hexagon; msg|val; x|val; y|val; rad|val; pts|val; snap_val|val"
    string spec = "format|hexagon; msg|" + msg + "; x|" + doubleToStringX(x, 2) +
                  "; y|" + doubleToStringX(y, 2) + "; rad|" + doubleToStringX(rad, 2) +
                  "; pts|" + intToString(pts) + "; snap_val|" + doubleToStringX(snap_val, 1);

    return spec;
}

string IgnoredRegionGenerator::generateRectangleSpec(double x, double y, double scale_factor)
{

    double width = m_rectangle_width_min + ((double)rand() / RAND_MAX) * (m_rectangle_width_max - m_rectangle_width_min);
    double height = m_rectangle_height_min + ((double)rand() / RAND_MAX) * (m_rectangle_height_max - m_rectangle_height_min);

    width *= scale_factor;
    height *= scale_factor;

    double degs = ((double)rand() / RAND_MAX) * m_max_rotation_deg;

    string msg = randomShapeName(IgnoredRegion::RegionType::RECTANGLE);

    // Format: "format|rectangle; msg|val; cx|val; cy|val; width|val; height|val; degs|val"
    string spec = "format|rectangle; msg|" + msg + "; cx|" + doubleToStringX(x, 2) +
                  "; cy|" + doubleToStringX(y, 2) + "; width|" + doubleToStringX(width, 2) +
                  "; height|" + doubleToStringX(height, 2) + "; degs|" + doubleToStringX(degs, 2);

    return spec;
}

string IgnoredRegionGenerator::generateRegionSpec(double x, double y, double scale_factor)
{
    // Randomly select a shape type using the enum
    IgnoredRegion::RegionType regionType = getRandomRegionType();

    // Generate the appropriate shape specification based on the random selection
    switch (regionType)
    {
    case IgnoredRegion::RegionType::ELLIPSE:
        return generateEllipseSpec(x, y, scale_factor);
    case IgnoredRegion::RegionType::RADIAL:
        return generateRadialSpec(x, y, scale_factor);
    case IgnoredRegion::RegionType::OVAL:
        return generateOvalSpec(x, y, scale_factor);
    case IgnoredRegion::RegionType::HEXAGON:
        return generateHexagonSpec(x, y, scale_factor);
    case IgnoredRegion::RegionType::RECTANGLE:
        return generateRectangleSpec(x, y, scale_factor);
    default:
        return generateHexagonSpec(x, y, scale_factor); // Default to hexagon
    }
}

bool IgnoredRegionGenerator::generate(std::stringstream &out)
{
    unsigned int total_regions = m_region_amt + m_spawnable_region_amt;
    if (total_regions == 0)
    {
        out << "No regions requested. No regions generated." << endl;
        return (false);
    }
    if (m_generator.size() == 0)
    {
        out << "No area polygon specified. No regions generated" << endl;
        return (false);
    }
    if (m_spawnable_region_amt > 0 && (m_spawn_tmin == 0 || m_spawn_tmax == 0))
    {
        out << "Spawnable regions requested but no spawn interval specified. No regions generated." << endl;
        return (false);
    }

    // Seed the random number generator
    unsigned long tseed = time(NULL) + 2;
    unsigned long pid = (long)getpid() + 2;
    unsigned long seed = (tseed % 999999);
    seed = ((rand())*seed) % 999999;
    seed = (seed * pid) % 999999;
    srand(seed);

    // Generate spawn times between min and max
    vector<double> spawntimes;
    for (unsigned int i = 0; i < m_spawnable_region_amt; i++)
    {
        double spawntime = m_spawn_tmin + (rand() % (m_spawn_tmax - m_spawn_tmin));
        spawntimes.push_back(spawntime);
    }

    m_generator.setSnap(1);
    if (total_regions > 50)
        m_generator.setSnap(0.1);

    // Calculate max possible size to ensure proper buffer distance
    const std::vector<double> sizes {m_ellipse_major_max,
                                     m_radial_radius_max * 2,
                                     m_oval_len_max,
                                     m_hexagon_rad_max * 2,
                                     std::max(m_rectangle_width_max, m_rectangle_height_max)};

                                     
    double max_size = *std::max_element(sizes.begin(), sizes.end());


    // Add buffer for polygon size
    m_generator.setBufferDist(m_buffer_dist + max_size / 2);
    m_generator.setFlexBuffer(false); // Do not allow min_separation to shrink
    m_generator.generatePoints(total_regions);

    vector<XYPoint> points = m_generator.getPoints();
    if (points.size() != total_regions)
        return (false);

    // Generate region specifications and corresponding polygons
    vector<string> region_specs;

    for (unsigned int i = 0; i < points.size(); i++)
    {
        double x = points[i].get_vx();
        double y = points[i].get_vy();

        // Generate size between min and max
        double size = m_min_region_size + ((double)rand() / RAND_MAX) * (m_max_region_size - m_min_region_size);
        double scale_factor = size/20.0;

        // Generate region specification
        string spec = generateRegionSpec(x, y, scale_factor);
        region_specs.push_back(spec);

    }

    // Output results
    double nearest = m_generator.getGlobalNearest();
    out << "// Lowest dist between center of regions: ";
    out << doubleToString(nearest * MOOSDIST2METERS, 2) << "m" << endl;
    for (unsigned int i = 0; i < m_generator.size(); i++)
    {
        string poly_spec = m_generator.getPolygon(i).get_spec(4);
        out << "poly = " << poly_spec << endl;
    }

    // Output regular regions
    for (unsigned int i = 0; i < m_region_amt; i++)
    {
        string region_name = "region";
        if (i + 1 < 10)
            region_name += "0";
        region_name += uintToString(i + 1);

        out << "ignoredRegion = " << "format=" << region_specs[i];
        out << ", name=" << region_name << endl;
    }

    // Output spawnable regions
    for (unsigned int i = 0; i < m_spawnable_region_amt; i++)
    {
        string region_name = "spawn_region";
        if (i + 1 < 10)
            region_name += "0";
        region_name += uintToString(i + 1);
        double spawntime = spawntimes[i];

        out << "ignoredRegion = " << "format=" << region_specs[i + m_region_amt];
        out << ", name=" << region_name;
        out << ", spawntime=" << uintToString(spawntime) << endl;
    }

    return (true);
}
