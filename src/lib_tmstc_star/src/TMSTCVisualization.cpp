#include "TMSTCVisualization.h"

namespace TMSTCViz
{
    void visualizePaths(const Mat &regionMap, const Mat &paths, 
                       const std::vector<std::pair<int, int>> &robot_positions,
                       std::ostream &out, bool useColor)
    {
        // Create a copy of the map for visualization
        std::vector<std::vector<std::string>> visMap(regionMap.size(), std::vector<std::string>(regionMap[0].size(), " "));

        // Mark obstacles and free spaces
        for (size_t y = 0; y < regionMap.size(); y++)
        {
            for (size_t x = 0; x < regionMap[y].size(); x++)
            {
                if (regionMap[y][x] == 0)
                {
                    visMap[y][x] = "x"; // Obstacle
                }
                else
                {
                    visMap[y][x] = "O"; // Free space
                }
            }
        }

        // Define colors for different robots (up to 6)
        const char *colors[] = {
            COLOR_RED, COLOR_GREEN, COLOR_YELLOW,
            COLOR_BLUE, COLOR_MAGENTA, COLOR_CYAN};

        // Mark robot paths with colors
        for (int i = 0; i < paths.size(); i++)
        {
            const char *robotColor = useColor ? colors[i % 6] : "";
            const char *resetColor = useColor ? COLOR_RESET : "";

            // Mark path
            for (int j = 0; j < paths[i].size(); j++)
            {
                auto coords = TMSTCStar::indexToCoord(paths[i][j], regionMap[0].size());
                int x = coords.first;
                int y = coords.second;

                // Only mark if within bounds
                if (y >= 0 && y < (int)regionMap.size() && x >= 0 && x < (int)regionMap[0].size())
                {
                    // Mark with robot number instead of just path
                    visMap[y][x] = std::string(robotColor) + std::to_string(i + 1) + std::string(resetColor);
                }
            }

            // Mark start position
            int startX = robot_positions[i].first;
            int startY = robot_positions[i].second;
            visMap[startY][startX] = std::string(robotColor) + "B" + std::string(resetColor);
        }

        // Print the visualization
        for (size_t y = 0; y < visMap.size(); y++)
        {
            for (size_t x = 0; x < visMap[y].size(); x++)
            {
                out << visMap[y][x] << " ";
            }
            out << std::endl;
        }

        // Print legend
        out << "\nLegend: " << std::endl;
        out << "  x - Obstacle" << std::endl;
        out << "  O - Free space" << std::endl;
        out << "  B - Start position" << std::endl;
        for (size_t i = 0; i < paths.size(); i++)
        {
            const char *robotColor = useColor ? colors[i % 6] : "";
            const char *resetColor = useColor ? COLOR_RESET : "";
            out << "  " << robotColor << (i + 1) << resetColor << " - Robot " << (i + 1) << " path" << std::endl;
        }
    }

    void visualizeInitialMap(const Mat &map, const std::vector<std::pair<int, int>> &robot_positions,
                            std::ostream &out, bool useColor)
    {
        // Create a copy of the map for visualization
        std::vector<std::vector<std::string>> visMap(map.size(), std::vector<std::string>(map[0].size(), " "));

        // Mark obstacles and free spaces
        for (size_t y = 0; y < map.size(); y++)
        {
            for (size_t x = 0; x < map[y].size(); x++)
            {
                if (map[y][x] == 0)
                {
                    visMap[y][x] = "x"; // Obstacle
                }
                else
                {
                    visMap[y][x] = "O"; // Free space
                }
            }
        }

        // Define colors for different robots (up to 6)
        const char *colors[] = {
            COLOR_RED, COLOR_GREEN, COLOR_YELLOW,
            COLOR_BLUE, COLOR_MAGENTA, COLOR_CYAN};

        // Mark robot starting positions with colors
        for (size_t i = 0; i < robot_positions.size(); i++)
        {
            int x = robot_positions[i].first;
            int y = robot_positions[i].second;

            const char *robotColor = useColor ? colors[i % 6] : "";
            const char *resetColor = useColor ? COLOR_RESET : "";

            // Mark with robot number
            visMap[y][x] = std::string(robotColor) + std::to_string(i + 1) + std::string(resetColor);
        }

        // Print the visualization
        for (size_t y = 0; y < visMap.size(); y++)
        {
            for (size_t x = 0; x < visMap[y].size(); x++)
            {
                out << visMap[y][x] << " ";
            }
            out << std::endl;
        }

        // Print legend
        out << "\nLegend: " << std::endl;
        out << "  x - Obstacle" << std::endl;
        out << "  O - Free space" << std::endl;
        for (size_t i = 0; i < robot_positions.size(); i++)
        {
            const char *robotColor = useColor ? colors[i % 6] : "";
            const char *resetColor = useColor ? COLOR_RESET : "";
            out << "  " << robotColor << "R" << (i + 1) << resetColor
                << " - Robot " << (i + 1) << " starting position" << std::endl;
        }
    }

    std::vector<double> calculateRobotPathCosts(TMSTCStar &tmstc, const Mat &paths, int robot_count)
    {
        std::vector<double> path_costs(robot_count, 0.0);

        // Get the path statistics from TMSTCStar
        auto stats = tmstc.getPathStatistics();

        // If only one robot, it gets the total cost
        if (robot_count == 1)
        {
            path_costs[0] = stats.total_cost;
            return path_costs;
        }

        // For multiple robots
        double min_cost = stats.min_path_length;
        double max_cost = stats.max_path_length;
        double average_cost = stats.total_cost / robot_count;

        // Special case: if min and max are the same, all paths have the same cost
        if (std::abs(max_cost - min_cost) < 0.001)
        {
            for (int i = 0; i < robot_count; ++i)
            {
                path_costs[i] = average_cost;
            }
            return path_costs;
        }

        // Use path lengths as weights to distribute the costs
        std::vector<double> path_lengths(robot_count);
        double total_length = 0.0;

        for (int i = 0; i < robot_count && i < (int)paths.size(); i++)
        {
            path_lengths[i] = paths[i].size();
            total_length += path_lengths[i];
        }

        // Distribute costs proportionally to path lengths
        for (int i = 0; i < robot_count && i < (int)paths.size(); i++)
        {
            double weight = path_lengths[i] / total_length;
            // Scale between min and max costs
            path_costs[i] = min_cost + weight * (max_cost - min_cost) * robot_count;
        }

        return path_costs;
    }
    
    void visualizeStats(const TMSTCStar::PathStats &stats, const std::vector<double> &path_costs,
                       std::ostream &out, bool useColor)
    {
        out << "\n===== Path Statistics Visualization =====" << std::endl;

        // Calculate max bar width
        const int maxWidth = 50;

        // Find max cost for scaling the bars
        double max_cost = stats.max_path_length;
        if (max_cost <= 0)
        {
            max_cost = *std::max_element(path_costs.begin(), path_costs.end());
        }

        // Define colors for different robots (up to 6)
        const char *colors[] = {
            COLOR_RED, COLOR_GREEN, COLOR_YELLOW,
            COLOR_BLUE, COLOR_MAGENTA, COLOR_CYAN};

        // Visualize path costs for each robot
        out << "Path Costs:" << std::endl;
        for (size_t i = 0; i < path_costs.size(); i++)
        {
            out << "Robot " << (i + 1) << ": ";
            int barWidth = (int)(path_costs[i] * maxWidth / max_cost);
            const char *robotColor = useColor ? colors[i % 6] : "";
            const char *resetColor = useColor ? COLOR_RESET : "";
            
            out << robotColor;
            for (int j = 0; j < barWidth; j++)
                out << "█";
            out << resetColor << " " << path_costs[i] << std::endl;
        }

        // Print overall statistics
        out << "\nOverall Statistics:" << std::endl;
        out << "  Total length: " << stats.total_length << std::endl;
        out << "  Total turns: " << stats.total_turns << std::endl;
        out << "  Total cost: " << stats.total_cost << std::endl;
        out << "  Max path cost: " << stats.max_path_length << std::endl;
        out << "  Min path cost: " << stats.min_path_length << std::endl;
    }

    void visualizeDirectionalPaths(const Mat &regionMap, const Mat &paths, 
                                  const std::vector<std::pair<int, int>> &robot_positions,
                                  std::ostream &out, bool useColor)
    {
        // Create a copy of the map for visualization
        std::vector<std::vector<std::string>> visMap(regionMap.size(), std::vector<std::string>(regionMap[0].size(), " "));

        // Mark obstacles and free spaces
        for (size_t y = 0; y < regionMap.size(); y++)
        {
            for (size_t x = 0; x < regionMap[y].size(); x++)
            {
                if (regionMap[y][x] == 0)
                {
                    visMap[y][x] = "x"; // Obstacle
                }
                else
                {
                    visMap[y][x] = "·"; // Free space
                }
            }
        }

        // Define colors for different robots (up to 6)
        const char *colors[] = {
            COLOR_RED, COLOR_GREEN, COLOR_YELLOW,
            COLOR_BLUE, COLOR_MAGENTA, COLOR_CYAN};

        // Mark robot paths with directional arrows
        for (size_t i = 0; i < paths.size(); i++)
        {
            const char *robotColor = useColor ? colors[i % 6] : "";
            const char *resetColor = useColor ? COLOR_RESET : "";

            // Mark start position
            int startX = robot_positions[i].first;
            int startY = robot_positions[i].second;
            visMap[startY][startX] = std::string(robotColor) + "S" + std::string(resetColor);

            // Mark path with directional arrows
            for (size_t j = 1; j < paths[i].size(); j++)
            {
                auto prevCoords = TMSTCStar::indexToCoord(paths[i][j - 1], regionMap[0].size());
                auto currCoords = TMSTCStar::indexToCoord(paths[i][j], regionMap[0].size());

                int prevX = prevCoords.first;
                int prevY = prevCoords.second;
                int currX = currCoords.first;
                int currY = currCoords.second;

                // Determine direction
                int dx = currX - prevX;
                int dy = currY - prevY;

                // Define arrows for 8 possible directions
                const char *arrows[] = {"→", "↗", "↑", "↖", "←", "↙", "↓", "↘"};
                int arrowIdx = -1;

                if (dx == 1 && dy == 0)
                    arrowIdx = 0; // →
                else if (dx == 1 && dy == -1)
                    arrowIdx = 1; // ↗
                else if (dx == 0 && dy == -1)
                    arrowIdx = 2; // ↑
                else if (dx == -1 && dy == -1)
                    arrowIdx = 3; // ↖
                else if (dx == -1 && dy == 0)
                    arrowIdx = 4; // ←
                else if (dx == -1 && dy == 1)
                    arrowIdx = 5; // ↙
                else if (dx == 0 && dy == 1)
                    arrowIdx = 6; // ↓
                else if (dx == 1 && dy == 1)
                    arrowIdx = 7; // ↘

                // Only mark if within bounds
                if (currY >= 0 && currY < (int)regionMap.size() && currX >= 0 && currX < (int)regionMap[0].size())
                {
                    // Mark with colored arrow
                    if (arrowIdx >= 0)
                    {
                        visMap[currY][currX] = std::string(robotColor) + arrows[arrowIdx] + std::string(resetColor);
                    }
                }
            }

            // Mark end position
            if (paths[i].size() > 0)
            {
                auto coords = TMSTCStar::indexToCoord(paths[i].back(), regionMap[0].size());
                int x = coords.first;
                int y = coords.second;
                if (y >= 0 && y < (int)regionMap.size() && x >= 0 && x < (int)regionMap[0].size())
                {
                    visMap[y][x] = std::string(robotColor) + "E" + std::string(resetColor);
                }
            }
        }

        // Print the visualization
        for (size_t y = 0; y < visMap.size(); y++)
        {
            for (size_t x = 0; x < visMap[y].size(); x++)
            {
                out << visMap[y][x] << " ";
            }
            out << std::endl;
        }

        // Print legend
        out << "\nLegend: " << std::endl;
        out << "  x - Obstacle" << std::endl;
        out << "  · - Free space" << std::endl;
        out << "  S - Start position" << std::endl;
        out << "  E - End position" << std::endl;
        out << "  " << "→ ↗ ↑ ↖ ← ↙ ↓ ↘" << " - Direction of movement" << std::endl;
        for (size_t i = 0; i < paths.size(); i++)
        {
            const char *robotColor = useColor ? colors[i % 6] : "";
            const char *resetColor = useColor ? COLOR_RESET : "";
            out << "  " << robotColor << "Robot " << (i + 1) << resetColor << " path" << std::endl;
        }
    }
}