#ifndef _VISUALIZATION_H
#define _VISUALIZATION_H

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <numeric>
#include <cmath>
#include "TMSTCStar.h"

// Console colors for visualization
#define COLOR_RESET "\033[0m"
#define COLOR_RED "\033[31m"
#define COLOR_GREEN "\033[32m"
#define COLOR_YELLOW "\033[33m"
#define COLOR_BLUE "\033[34m"
#define COLOR_MAGENTA "\033[35m"
#define COLOR_CYAN "\033[36m"
#define COLOR_WHITE "\033[37m"

namespace TMSTCViz
{
    // Standard grid-based path visualization
    void visualizePaths(const Mat &regionMap, const Mat &paths,
                        const std::vector<std::pair<int, int>> &robot_positions,
                        std::ostream &out = std::cout,
                        bool useColor = true);

    // Visualization with initial robot positions
    void visualizeInitialMap(const Mat &map,
                             const std::vector<std::pair<int, int>> &robot_positions,
                             std::ostream &out = std::cout,
                             bool useColor = true);

    // Path statistics visualization with colored bars
    void visualizeStats(const TMSTCStar::PathStats &stats,
                        const std::vector<double> &path_costs,
                        std::ostream &out = std::cout,
                        bool useColor = true);

    // Path visualization with directional arrows
    void visualizeDirectionalPaths(const Mat &regionMap, const Mat &paths,
                                   const std::vector<std::pair<int, int>> &robot_positions,
                                   std::ostream &out = std::cout,
                                   bool useColor = true);

    // Calculate individual robot path costs (helper function)
    std::vector<double> calculateRobotPathCosts(TMSTCStar &tmstc,
                                                const Mat &paths, int robot_count);
}

#endif
