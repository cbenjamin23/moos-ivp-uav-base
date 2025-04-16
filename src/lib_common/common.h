#pragma once

constexpr double MOOSDIST2METERS = 0.5;


namespace Planner {

    const enum PlannerMode
    {
        TMSTC_STAR,
        VORONOI_BASED
    };
    
    inline std::string modeToString(PlannerMode mode) {
        switch (mode)
        {
            case TMSTC_STAR:
            return "TMSTC_STAR";
            case VORONOI_BASED:
            return "VORONOI_BASED";
            default:
            return "UNKNOWN_MODE";
        }
    }

    inline PlannerMode stringToMode(const std::string& mode_str) {
        if (mode_str == "TMSTC_STAR") {
            return TMSTC_STAR;
        } else if (mode_str == "VORONOI_BASED") {
            return VORONOI_BASED;
        } else {
            throw std::invalid_argument("Invalid PlannerMode string: " + mode_str);
        }
    }



}