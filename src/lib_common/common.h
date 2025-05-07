#pragma once

constexpr double MOOSDIST2METERS = 0.5;

namespace Planner
{

    enum PlannerMode {
        TMSTC_STAR,
        VORONOI_SEARCH,
        UNKNOWN_MODE
    };

    inline std::string modeToString(PlannerMode mode)
    {
        switch (mode)
        {
        case TMSTC_STAR:
            return "TMSTC_STAR";
        case VORONOI_SEARCH:
            return "VORONOI_SEARCH";
        default:
            return "UNKNOWN_MODE";
        }
    }

    inline PlannerMode stringToMode(const std::string &mode_str)
    {
        if (mode_str == "TMSTC_STAR")
            return TMSTC_STAR;
        else if (mode_str == "VORONOI_SEARCH")
            return VORONOI_SEARCH;
        else
            throw std::invalid_argument("Invalid PlannerMode string: " + mode_str);
    }

}