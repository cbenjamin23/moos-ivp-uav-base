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
#include "IgnoredRegion.h"

#include "MBUtils.h"
#include "XYFormatUtilsPoly.h"

#include "common.h"
#include "Logger.h"



using namespace std;

IgnoredRegionGenerator::IgnoredRegionGenerator()
{
    m_region_amt = 1;
    m_spawnable_region_amt = 0;
    m_spawn_tmin = 0;
    m_spawn_tmax = 0;
    m_buffer_dist = 200;
    m_min_region_size = 20;
    m_max_region_size = 40;

    // Initialize shape-specific parameters
    
    // Ellipse parameters
    m_ellipse_major_min = 100;
    m_ellipse_major_max = 300;
    m_ellipse_minor_min = 70;
    m_ellipse_minor_max = 150;

    // Radial (circle) parameters
    m_radial_radius_min = 70;
    m_radial_radius_max = 100;

    // Oval parameters
    m_oval_rad_min = 70;
    m_oval_rad_max = 120;
    m_oval_len_min = 150;
    m_oval_len_max = 300;

    // Hexagon parameters
    m_hexagon_rad_min = 70;
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

bool IgnoredRegionGenerator::generate(std::stringstream &out, const std::vector<XYPoint>& fire_points)
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
        int max_attempts = 20;
        
        string spec = generateRegionSpec(x, y, scale_factor);
        do{
            spec = moveRegionAwayFromFires(spec, x, y, fire_points, scale_factor);
            spec = generateRegionSpec(x, y, scale_factor);
            max_attempts--;
        } while(  max_attempts > 0 && spec.empty() );

        // Logger::info("Generated region spec: " + spec);

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



std::string convertToPipeFormat(std::string format_spec)
{
    // Convert from equals format to pipe format if needed
    bool using_equals_format = format_spec.find('=') != std::string::npos;
    std::string working_format = format_spec;

    if (using_equals_format)
    {
        working_format = findReplace(format_spec, ",", ";");
        working_format = findReplace(working_format, "=", "|");
    }

    return working_format;
}

std::string convertToEqualFormat(std::string format_spec)
{
    // Convert from pipe format to equals format if needed
    bool using_pipe_format = format_spec.find('|') != std::string::npos;
    std::string working_format = format_spec;

    if (using_pipe_format)
    {
        working_format = findReplace(format_spec, ";", ",");
        working_format = findReplace(working_format, "|", "=");
    }

    return working_format;
}

std::string IgnoredRegionGenerator::moveRegionAwayFromFires(std::string format_spec,
                                                      double new_x, double new_y,
                                                      const std::vector<XYPoint> &fire_points,
                                                      double scale_factor)
{
    // Convert from pipe format to equals format if needed
    std::string working_format = convertToEqualFormat(format_spec);

    // Create a temporary region using the original format but with new position
    std::string region_type = tokStringParse(working_format, "format");
    std::string msg_val = tokStringParse(working_format, "msg");

    // Replace only the x and y values in the spec
    working_format = findReplace(working_format, "x=" + tokStringParse(working_format, "x"), "x=" + doubleToStringX(new_x, 2));
    working_format = findReplace(working_format, "y=" + tokStringParse(working_format, "y"), "y=" + doubleToStringX(new_y, 2));

    // Apply scale factor to size parameters if needed
    if (scale_factor != 1.0)
    {
        if (region_type == "ellipse")
        {
            double major = tokDoubleParse(working_format, "major") * scale_factor;
            double minor = tokDoubleParse(working_format, "minor") * scale_factor;
            working_format = findReplace(working_format, "major=" + tokStringParse(working_format, "major"), "major=" + doubleToStringX(major, 2));
            working_format = findReplace(working_format, "minor=" + tokStringParse(working_format, "minor"), "minor=" + doubleToStringX(minor, 2));
        }
        else if (region_type == "radial" || region_type == "hexagon")
        {
            double rad = tokDoubleParse(working_format, "rad") * scale_factor;
            working_format = findReplace(working_format, "rad=" + tokStringParse(working_format, "rad"), "rad=" + doubleToStringX(rad, 2));
        }
        else if (region_type == "rectangle")
        {
            double width = tokDoubleParse(working_format, "width") * scale_factor;
            double height = tokDoubleParse(working_format, "height") * scale_factor;
            working_format = findReplace(working_format, "width=" + tokStringParse(working_format, "width"), "width=" + doubleToStringX(width, 2));
            working_format = findReplace(working_format, "height=" + tokStringParse(working_format, "height"), "height=" + doubleToStringX(height, 2));
        }
        else if (region_type == "oval")
        {
            double rad = tokDoubleParse(working_format, "rad") * scale_factor;
            double len = tokDoubleParse(working_format, "len") * scale_factor;
            working_format = findReplace(working_format, "rad=" + tokStringParse(working_format, "rad"), "rad=" + doubleToStringX(rad, 2));
            working_format = findReplace(working_format, "len=" + tokStringParse(working_format, "len"), "len=" + doubleToStringX(len, 2));
        }
    }

    // Check if the updated region overlaps with any fire points
    IgnoredRegion ireg = stringToIgnoredRegion("format=" + convertToPipeFormat(working_format));
    auto polygon = ireg.getPoly();

    // Check if the polygon contains any fire points
    bool contains_fire = false;
    XYPoint closest_fire;
    double min_distance = 1e10;

    for (const auto &point : fire_points)
    {
        if (polygon.contains(point.get_vx(), point.get_vy()))
        {
            contains_fire = true;

            // Keep track of closest fire to help with movement direction
            double dist = hypot(new_x - point.get_vx(), new_y - point.get_vy());
            if (dist < min_distance)
            {
                min_distance = dist;
                closest_fire = point;
            }
        }
    }

    // If polygon contains fire points, try to fix by moving
    if (!contains_fire)
        return format_spec;

    Logger::info("Region contains fire points, attempting to move");
    // First attempt: try moving away from the closest fire
    const int max_move_attempts = 5;
    const double move_distance = 20.0;

    double orig_x = new_x;
    double orig_y = new_y;
    double current_scale = scale_factor;

    // Try moving the region away from fires
    for (int attempt = 0; attempt < max_move_attempts && contains_fire; attempt++)
    {
        Logger::info("Attempt " + std::to_string(attempt + 1) + " to move region away from fires");
        // Calculate direction away from closest fire
        double dx = orig_x - closest_fire.get_vx();
        double dy = orig_y - closest_fire.get_vy();

        // Normalize direction vector
        double magnitude = hypot(dx, dy);
        if (magnitude > 0)
        {
            dx /= magnitude;
            dy /= magnitude;
        }
        else
        {
            // If we're exactly on top of a fire, choose a random direction
            dx = (double)rand() / RAND_MAX - 0.5;
            dy = (double)rand() / RAND_MAX - 0.5;
            magnitude = hypot(dx, dy);
            dx /= magnitude;
            dy /= magnitude;
        }

        // Move in the calculated direction
        double move_x = orig_x + dx * move_distance * (attempt + 1);
        double move_y = orig_y + dy * move_distance * (attempt + 1);

        // Create a new format spec with the adjusted position
        std::string adjusted_spec = working_format;
        adjusted_spec = findReplace(adjusted_spec, "x=" + tokStringParse(adjusted_spec, "x"), "x=" + doubleToStringX(move_x, 2));
        adjusted_spec = findReplace(adjusted_spec, "y=" + tokStringParse(adjusted_spec, "y"), "y=" + doubleToStringX(move_y, 2));

        // Check if the new position avoids fire points
        IgnoredRegion temp_region = stringToIgnoredRegion("format=" + convertToPipeFormat(adjusted_spec));
        auto temp_poly = temp_region.getPoly();

        contains_fire = false;
        for (const auto &point : fire_points)
        {
            if (temp_poly.contains(point.get_vx(), point.get_vy()))
            {
                contains_fire = true;
                break;
            }
        }

        // If this position works, use it
        if (!contains_fire)
        {
            working_format = adjusted_spec;
        }
    }

    // If moving didn't work, try shrinking
    if (contains_fire)
    {
        Logger::info("Region still contains fire points, attempting to shrink");
        const int max_shrink_attempts = 5;
        const double shrink_factor = 0.8;

        current_scale = scale_factor;
        for (int attempt = 0; attempt < max_shrink_attempts && contains_fire; attempt++)
        {
            Logger::info("Attempt " + std::to_string(attempt + 1) + " to shrink region away from fires");
            current_scale *= shrink_factor;

            // Apply scaling to the region parameters
            std::string shrunk_spec = working_format;
            if (region_type == "ellipse")
            {
                double major = tokDoubleParse(shrunk_spec, "major") * shrink_factor;
                double minor = tokDoubleParse(shrunk_spec, "minor") * shrink_factor;
                shrunk_spec = findReplace(shrunk_spec, "major=" + tokStringParse(shrunk_spec, "major"), "major=" + doubleToStringX(major, 2));
                shrunk_spec = findReplace(shrunk_spec, "minor=" + tokStringParse(shrunk_spec, "minor"), "minor=" + doubleToStringX(minor, 2));
            }
            else if (region_type == "radial" || region_type == "hexagon")
            {
                double rad = tokDoubleParse(shrunk_spec, "rad") * shrink_factor;
                shrunk_spec = findReplace(shrunk_spec, "rad=" + tokStringParse(shrunk_spec, "rad"), "rad=" + doubleToStringX(rad, 2));
            }
            else if (region_type == "rectangle")
            {
                double width = tokDoubleParse(shrunk_spec, "width") * shrink_factor;
                double height = tokDoubleParse(shrunk_spec, "height") * shrink_factor;
                shrunk_spec = findReplace(shrunk_spec, "width=" + tokStringParse(shrunk_spec, "width"), "width=" + doubleToStringX(width, 2));
                shrunk_spec = findReplace(shrunk_spec, "height=" + tokStringParse(shrunk_spec, "height"), "height=" + doubleToStringX(height, 2));
            }
            else if (region_type == "oval")
            {
                double rad = tokDoubleParse(shrunk_spec, "rad") * shrink_factor;
                double len = tokDoubleParse(shrunk_spec, "len") * shrink_factor;
                shrunk_spec = findReplace(shrunk_spec, "rad=" + tokStringParse(shrunk_spec, "rad"), "rad=" + doubleToStringX(rad, 2));
                shrunk_spec = findReplace(shrunk_spec, "len=" + tokStringParse(shrunk_spec, "len"), "len=" + doubleToStringX(len, 2));
            }

            // Check if the shrunk region avoids fire points
            IgnoredRegion temp_region = stringToIgnoredRegion("format=" + convertToPipeFormat(shrunk_spec));
            auto temp_poly = temp_region.getPoly();

            contains_fire = false;
            for (const auto &point : fire_points)
            {
                if (temp_poly.contains(point.get_vx(), point.get_vy()))
                {
                    contains_fire = true;
                    break;
                }
            }

            // If this size works, use it
            if (!contains_fire)
            {
                working_format = shrunk_spec;
            }
        }
    }

    // If we still have conflicts, try one last approach - random location
    if (contains_fire)
    {
        Logger::info("Region still contains fire points, attempting random location");
        double move_x = orig_x + ((double)rand() / RAND_MAX - 0.5) * 100;
        double move_y = orig_y + ((double)rand() / RAND_MAX - 0.5) * 100;
        current_scale = scale_factor * 0.5; // Half the original size as last resort

        // Apply changes to the format spec
        std::string final_spec = working_format;
        final_spec = findReplace(final_spec, "x=" + tokStringParse(final_spec, "x"), "x=" + doubleToStringX(move_x, 2));
        final_spec = findReplace(final_spec, "y=" + tokStringParse(final_spec, "y"), "y=" + doubleToStringX(move_y, 2));

        // Apply scaling for last attempt
        if (region_type == "ellipse")
        {
            double major = tokDoubleParse(final_spec, "major") * 0.5;
            double minor = tokDoubleParse(final_spec, "minor") * 0.5;
            final_spec = findReplace(final_spec, "major=" + tokStringParse(final_spec, "major"), "major=" + doubleToStringX(major, 2));
            final_spec = findReplace(final_spec, "minor=" + tokStringParse(final_spec, "minor"), "minor=" + doubleToStringX(minor, 2));
        }
        else if (region_type == "radial" || region_type == "hexagon")
        {
            double rad = tokDoubleParse(final_spec, "rad") * 0.5;
            final_spec = findReplace(final_spec, "rad=" + tokStringParse(final_spec, "rad"), "rad=" + doubleToStringX(rad, 2));
        }
        else if (region_type == "rectangle")
        {
            double width = tokDoubleParse(final_spec, "width") * 0.5;
            double height = tokDoubleParse(final_spec, "height") * 0.5;
            final_spec = findReplace(final_spec, "width=" + tokStringParse(final_spec, "width"), "width=" + doubleToStringX(width, 2));
            final_spec = findReplace(final_spec, "height=" + tokStringParse(final_spec, "height"), "height=" + doubleToStringX(height, 2));
        }
        else if (region_type == "oval")
        {
            double rad = tokDoubleParse(final_spec, "rad") * 0.5;
            double len = tokDoubleParse(final_spec, "len") * 0.5;
            final_spec = findReplace(final_spec, "rad=" + tokStringParse(final_spec, "rad"), "rad=" + doubleToStringX(rad, 2));
            final_spec = findReplace(final_spec, "len=" + tokStringParse(final_spec, "len"), "len=" + doubleToStringX(len, 2));
        }

        working_format = final_spec;

        // Final check to see if we avoided fires
        IgnoredRegion temp_region = stringToIgnoredRegion("format=" + convertToPipeFormat(working_format));
        auto temp_poly = temp_region.getPoly();

        contains_fire = false;
        for (const auto &point : fire_points)
        {
            if (temp_poly.contains(point.get_vx(), point.get_vy()))
            {
                contains_fire = true;
                break;
            }
        }
    }


    working_format = convertToPipeFormat(working_format);

    // If we still have conflicts, return empty string to indicate failure
    if (contains_fire)
    {
        Logger::warning("Failed to move region away from fires");
        return "";
    }

    return working_format;
}
