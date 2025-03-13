#include <iostream>
#include <fstream>
#include <string>
#include <ctime>

#include "TMSTCStar.h"

#define DEBUG

// Console colors for visualization
#define COLOR_RESET "\033[0m"
#define COLOR_RED "\033[31m"
#define COLOR_GREEN "\033[32m"
#define COLOR_YELLOW "\033[33m"
#define COLOR_BLUE "\033[34m"
#define COLOR_MAGENTA "\033[35m"
#define COLOR_CYAN "\033[36m"
#define COLOR_WHITE "\033[37m"

// Utility function to load map from a text file
Mat loadMapFromFile(const std::string &filename)
{
    std::ifstream infile(filename);
    if (!infile.is_open())
    {
        std::cerr << "Error opening file: " << filename << std::endl;
        return {};
    }

    int h, w;
    infile >> h >> w;
    std::cout << "Map dimensions: " << h << " x " << w << std::endl;

    Mat map(h, std::vector<int>(w, 0));

    std::string line;
    // Skip the line with dimensions
    std::getline(infile, line);

    for (int i = 0; i < h && std::getline(infile, line); ++i)
    {
        int len = 0;
        for (size_t j = 0; j < line.length(); ++j)
        {
            if (line[j] == '0' || line[j] == '1')
            {
                map[i][len++] = line[j] - '0';
            }
        }
    }

    return map;
}

// Utility function to print a map
void printMap(const Mat &map)
{
    for (size_t i = 0; i < map.size(); ++i)
    {
        for (size_t j = 0; j < map[i].size(); ++j)
        {
            std::cout << map[i][j] << " ";
        }
        std::cout << std::endl;
    }
}

// Utility function to visualize paths on grid map
void visualizePaths(const Mat &regionMap, const Mat &paths, const std::vector<std::pair<int, int>> &robot_positions)
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
        const char *robotColor = colors[i % 6];

        
        // Mark path
        for (int j = 0; j < paths[i].size(); j++)
        {
            auto coords = TMSTCStar::indexToCoord(paths[i][j], regionMap[0].size() );
            int x = coords.first;
            int y = coords.second;

            // Only mark if within bounds
            if (y >= 0 && y < (int)regionMap.size() && x >= 0 && x < (int)regionMap[0].size())
            {
                // Mark with robot number instead of just path
                visMap[y][x] = robotColor + std::to_string(i + 1) + COLOR_RESET;
            }
        }

        // Mark start position
        int startX = robot_positions[i].first;
        int startY = robot_positions[i].second;
        visMap[startY][startX] = robotColor + std::string("B") + COLOR_RESET;

    }

    // Print the visualization
    std::cout << "\n===== Path Visualization =====" << std::endl;
    for (size_t y = 0; y < visMap.size(); y++)
    {
        for (size_t x = 0; x < visMap[y].size(); x++)
        {
            std::cout << visMap[y][x] << " ";
        }
        std::cout << std::endl;
    }
    std::cout << "============================" << std::endl;

    // Print legend
    std::cout << "\nLegend: " << std::endl;
    std::cout << "  x - Obstacle" << std::endl;
    std::cout << "  O - Free space" << std::endl;
    std::cout << "  B - Start position" << std::endl;
    for (size_t i = 0; i < paths.size(); i++)
    {
        std::cout << "  " << colors[i % 6] << (i + 1) << COLOR_RESET << " - Robot " << (i + 1) << " path" << std::endl;
    }
}

// Utility function to visualize initial map with robot positions
void visualizeInitialMap(const Mat &map, const std::vector<std::pair<int, int>> &robot_positions)
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

        // Mark with robot number and different symbol
        visMap[y][x] = std::string(colors[i % 6]) + std::to_string(i + 1) + COLOR_RESET;
        // visMap[y][x] = std::string(colors[i % 6]) + "R" + std::to_string(i + 1) + COLOR_RESET;
    }

    // Print the visualization
    std::cout << "\n===== Initial Map with Robot Positions =====" << std::endl;
    for (size_t y = 0; y < visMap.size(); y++)
    {
        for (size_t x = 0; x < visMap[y].size(); x++)
        {
            std::cout << visMap[y][x] << " ";
        }
        std::cout << std::endl;
    }
    std::cout << "==========================================" << std::endl;

    // Print legend
    std::cout << "\nLegend: " << std::endl;
    std::cout << "  x - Obstacle" << std::endl;
    std::cout << "  O - Free space" << std::endl;
    for (size_t i = 0; i < robot_positions.size(); i++)
    {
        std::cout << "  " << colors[i % 6] << "R" << (i + 1) << COLOR_RESET
                  << " - Robot " << (i + 1) << " starting position" << std::endl;
    }
}

// Utility function to calculate individual path costs for robots
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

    // For multiple robots: extract the proper path costs for each robot
    // Ask TMSTCStar for individual costs if possible

    // Since we don't have direct access to individual costs through the API,
    // we need to estimate costs based on the available stats and paths

    // Calculate path costs for each robot using available information
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

// Utility function to visualize path statistics
void visualizeStats(const TMSTCStar::PathStats &stats, const std::vector<double> &path_costs)
{
    std::cout << "\n===== Path Statistics Visualization =====" << std::endl;

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
    std::cout << "Path Costs:" << std::endl;
    for (size_t i = 0; i < path_costs.size(); i++)
    {
        std::cout << "Robot " << (i + 1) << ": ";
        int barWidth = (int)(path_costs[i] * maxWidth / max_cost);
        std::cout << colors[i % 6];
        for (int j = 0; j < barWidth; j++)
            std::cout << "█";
        std::cout << COLOR_RESET << " " << path_costs[i] << std::endl;
    }

    // Calculate and display statistics about path distribution
    if (path_costs.size() > 1)
    {
        // Find min cost
        double min_cost = *std::min_element(path_costs.begin(), path_costs.end());

        // Balance ratio (max/min path cost)
        double balance = max_cost / (min_cost > 0 ? min_cost : 1.0);
        std::cout << "\nPath Balance Metrics:" << std::endl;
        std::cout << "  Min path cost: " << min_cost << std::endl;
        std::cout << "  Max path cost: " << max_cost << std::endl;
        std::cout << "  Balance ratio: " << balance << " (lower is better)" << std::endl;

        // Calculate standard deviation to measure balance
        double sum = std::accumulate(path_costs.begin(), path_costs.end(), 0.0);
        double mean = sum / path_costs.size();
        double sq_sum = std::inner_product(path_costs.begin(), path_costs.end(),
                                           path_costs.begin(), 0.0,
                                           std::plus<>(), [mean](double x, double y)
                                           { return (x - mean) * (y - mean); });
        double std_dev = std::sqrt(sq_sum / path_costs.size());
        std::cout << "  Standard deviation: " << std_dev << std::endl;
        std::cout << "  Coefficient of variation: " << (std_dev / mean) << std::endl;
    }

    std::cout << "=============================" << std::endl;
}

// Utility function to visualize paths with directional arrows
void visualizeDirectionalPaths(const Mat &regionMap, const Mat &paths, const std::vector<std::pair<int, int>> &robot_positions)
{
    // Create a copy of the map for visualization
    std::vector<std::vector<std::string>> visMap(regionMap.size(), std::vector<std::string>(regionMap[0].size(), " "));

    // Define directional arrows for 8 directions
    const std::string arrows[8] = {
        "→", // East
        "↗", // Northeast
        "↑", // North
        "↖", // Northwest
        "←", // West
        "↙", // Southwest
        "↓", // South
        "↘"  // Southeast
    };

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
                visMap[y][x] = "·"; // Free space with small dot
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
        const char *robotColor = colors[i % 6];

        // Mark start position
        int startX = robot_positions[i].first;
        int startY = robot_positions[i].second;
        visMap[startY][startX] = robotColor + std::string("S") + COLOR_RESET;

        // Mark paths with directional arrows
        for (size_t j = 0; j < paths[i].size() - 1; j++)
        {
            auto curr = TMSTCStar::indexToCoord(paths[i][j], regionMap[0].size() );
            auto next = TMSTCStar::indexToCoord(paths[i][j + 1], regionMap[0].size());

            // Convert to map coordinates
            int currX = curr.first ;
            int currY = curr.second;
            int nextX = next.first ;
            int nextY = next.second;

            // Only mark if within bounds and cells are different
            if (currY >= 0 && currY < (int)regionMap.size() && currX >= 0 && currX < (int)regionMap[0].size() &&
                (currX != nextX || currY != nextY))
            {

                // Calculate direction
                int dx = nextX - currX;
                int dy = nextY - currY;

                // Normalize to -1, 0, or 1
                if (dx != 0)
                    dx = dx / abs(dx);
                if (dy != 0)
                    dy = dy / abs(dy);

                // Map direction to arrow index
                int arrowIdx = -1;
                if (dx == 1 && dy == 0)
                    arrowIdx = 0; // East
                else if (dx == 1 && dy == -1)
                    arrowIdx = 1; // Northeast
                else if (dx == 0 && dy == -1)
                    arrowIdx = 2; // North
                else if (dx == -1 && dy == -1)
                    arrowIdx = 3; // Northwest
                else if (dx == -1 && dy == 0)
                    arrowIdx = 4; // West
                else if (dx == -1 && dy == 1)
                    arrowIdx = 5; // Southwest
                else if (dx == 0 && dy == 1)
                    arrowIdx = 6; // South
                else if (dx == 1 && dy == 1)
                    arrowIdx = 7; // Southeast

                if (arrowIdx >= 0)
                {
                    // Mark with colored arrow
                    visMap[currY][currX] = robotColor + arrows[arrowIdx] + COLOR_RESET;
                }
            }
        }

        // Mark end position
        if (paths[i].size() > 0)
        {
            auto coords = TMSTCStar::indexToCoord(paths[i].back(), regionMap[0].size());
            int x = coords.first ;
            int y = coords.second;
            if (y >= 0 && y < (int)regionMap.size() && x >= 0 && x < (int)regionMap[0].size())
            {
                visMap[y][x] = std::string(robotColor) + "E" + COLOR_RESET;
            }
        }
    }

    // Print the visualization
    std::cout << "\n===== Directional Path Visualization =====" << std::endl;
    for (size_t y = 0; y < visMap.size(); y++)
    {
        for (size_t x = 0; x < visMap[y].size(); x++)
        {
            std::cout << visMap[y][x] << " ";
        }
        std::cout << std::endl;
    }
    std::cout << "=======================================" << std::endl;

    // Print legend
    std::cout << "\nLegend: " << std::endl;
    std::cout << "  x - Obstacle" << std::endl;
    std::cout << "  · - Free space" << std::endl;
    std::cout << "  S - Start position" << std::endl;
    std::cout << "  E - End position" << std::endl;
    std::cout << "  " << "→ ↗ ↑ ↖ ← ↙ ↓ ↘" << " - Direction of movement" << std::endl;
    for (size_t i = 0; i < paths.size(); i++)
    {
        std::cout << "  " << colors[i % 6] << "Robot " << (i + 1) << COLOR_RESET << " path" << std::endl;
    }
}

int main(int argc, char **argv)
{
    // Parse command line arguments
    std::string mapfile = "example_map.txt";
    std::string method = "MSTC";
    std::string shape = "RECT_DIV";
    int robot_count = 1;
    bool return_to_start = false;

    // Simple command-line parser
    for (int i = 1; i < argc; i++)
    {
        std::string arg = argv[i];
        if (arg == "--map" && i + 1 < argc)
        {
            mapfile = argv[++i];
        }
        else if (arg == "--method" && i + 1 < argc)
        {
            method = argv[++i];
        }
        else if (arg == "--shape" && i + 1 < argc)
        {
            shape = argv[++i];
        }
        else if (arg == "--robots" && i + 1 < argc)
        {
            robot_count = std::stoi(argv[++i]);
        }
        else if (arg == "--return")
        {
            return_to_start = true;
        }
        else if (arg == "--help")
        {
            std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
            std::cout << "Options:" << std::endl;
            std::cout << "  --map FILE       Map file to load (default: example_map.txt)" << std::endl;
            std::cout << "  --method METHOD  Allocation method: MSTC or MTSP (default: MSTC)" << std::endl;
            std::cout << "  --shape SHAPE    MST shape: RECT_DIV, DFS_VERTICAL, DFS_HORIZONTAL, KRUSKAL, ACO_OPT, DINIC, HEURISTIC" << std::endl;
            std::cout << "  --robots N       Number of robots (default: 1)" << std::endl;
            std::cout << "  --return         Return to start position after coverage" << std::endl;
            std::cout << "  --help           Display this help message" << std::endl;
            return 0;
        }
    }

    cout << "Map file: " << mapfile << endl;
    cout << "Method: " << method << endl;
    cout << "Shape: " << shape << endl;
    cout << "Robot count: " << robot_count << endl;
    cout << "Return to start: " << (return_to_start ? "true" : "false") << endl;

    // Load the map
    Mat map = loadMapFromFile(mapfile);
    if (map.empty())
    {
        std::cerr << "Failed to load map." << std::endl;
        return 1;
    }

    // Create robot positions (for demonstration, we'll place them at corners and center)
    std::vector<std::pair<int, int>> robot_positions;
    std::vector<int> robot_positions_indx;
    const int region_width = map[0].size() * 2;
    if (robot_count == 1)
    {
        robot_positions.push_back({1, 1}); // Top-left
        robot_positions_indx.push_back(TMSTCStar::coordToIndex(0, 1, region_width));
    }
    if (robot_count == 2)
    {
        // robot_positions.push_back({(int)map[0].size() - 1, (int)map.size() - 1}); // Bottom-right
        robot_positions.push_back({1,1}); 
        robot_positions.push_back({2,2}); 
        robot_positions_indx.push_back(TMSTCStar::coordToIndex(0, 1, region_width));
        robot_positions_indx.push_back(TMSTCStar::coordToIndex(2, 3, region_width));
        robot_positions_indx.push_back(TMSTCStar::coordToIndex(2, 4, region_width));
        
        
    }
    if (robot_count == 3)
    {
        robot_positions.push_back({(int)map[0].size() - 1, 0}); // Top-right
        robot_positions.push_back({1,1}); 
        robot_positions.push_back({2,2}); 
        
        robot_positions_indx.push_back(TMSTCStar::coordToIndex(0, 1, region_width));
        robot_positions_indx.push_back(TMSTCStar::coordToIndex(2, 3, region_width));
        robot_positions_indx.push_back(TMSTCStar::coordToIndex(2, 4, region_width));
        
    }
    if (robot_count == 4)
    {
        robot_positions.push_back({0, (int)map.size() - 1}); // Bottom-left
        robot_positions_indx.push_back(TMSTCStar::coordToIndex(3, 3, region_width));

    }
    // Add more robots at random positions if needed

    std::cout << "Robot count: " << robot_positions.size() << std::endl;

    std::srand(std::time(nullptr));
    while (robot_positions.size() < robot_count)
    {
        int x = std::rand() % map[0].size();
        int y = std::rand() % map.size();
        if (map[y][x])
        { // If it's a free cell
            robot_positions.push_back({x, y});
        }
    }

    // Visualize the initial map with robot positions
    visualizeInitialMap(map, robot_positions);
    
    // Configure the TMSTC* algorithm
    TMSTCStarConfig config;
    config.allocate_method = method;
    config.mst_shape = shape;
    config.robot_num = robot_count;
    config.cover_and_return = return_to_start;
    config.one_turn_value = 2.0; // Default turning cost

    // Create the TMSTC* instance and calculate paths
    TMSTCStar tmstc(map, robot_positions_indx, config);
    tmstc.eliminateIslands(); // Remove unreachable areas

    std::cout << "Calculating paths..." << std::endl;
    Mat paths = tmstc.calculatePaths();

    // Get and print path statistics
    auto stats = tmstc.getPathStatistics();
    std::cout << "--------------------------------" << std::endl;
    std::cout << "Path Statistics:" << std::endl;
    std::cout << "  Total length: " << stats.total_length << std::endl;
    std::cout << "  Total turns: " << stats.total_turns << std::endl;
    std::cout << "  Total cost: " << stats.total_cost << std::endl;
    std::cout << "  Max path cost: " << stats.max_path_length << std::endl;
    std::cout << "  Min path cost: " << stats.min_path_length << std::endl;

    // Apply path shortening
    // std::cout << "\nShortening paths to remove unnecessary points..." << std::endl;
    // Mat checkpoints;
    // paths = tmstc.shortenPaths(checkpoints);

    // paths = tmstc.removeDuplicateEdgesOnPath();

    

    Mat regionMap = tmstc.getRegion();
    vector<pair<int, int>> robot_path_start;


    

  
    for (int i = 0; i < paths.size(); i++)
    {
        if(paths[i].size() == 0)
            continue;
        
        if (map[0].size() * 2 != regionMap[0].size()){
            std::cout << "Error: region map size mismatch" << std::endl;
            return 1;
        }
      

        auto coord = TMSTCStar::indexToCoord(paths[i][0], regionMap[0].size());
        robot_path_start.push_back({coord.first , coord.second});
    }
    visualizeInitialMap(regionMap, robot_path_start);

    // Visualize the paths on the grid map (standard and directional)
    visualizePaths(regionMap, paths, robot_path_start);
    visualizeDirectionalPaths(regionMap, paths, robot_path_start);

    // Calculate individual robot path costs
    std::vector<double> robot_costs = calculateRobotPathCosts(tmstc, paths, robot_count);

    // Visualize the statistics with the calculated path costs
    visualizeStats(stats, robot_costs);

    // Print paths in a readable format
    std::cout << "Paths:" << std::endl;
    for (size_t i = 0; i < paths.size(); i++)
    {
        std::cout << "Robot " << i + 1 << " path (" << paths[i].size() << " points):" << std::endl;
        for (size_t j = 0; j < paths[i].size() && j < 10; j++)
        {
            auto coords = TMSTCStar::indexToCoord(paths[i][j], map[0].size() * 2);
            std::cout << "  (" << coords.first / 2 << "," << coords.second / 2 << ")";
            if (j < paths[i].size() - 1)
                std::cout << " -> ";
        }
        if (paths[i].size() > 10)
        {
            std::cout << " ... (truncated)";
        }
        std::cout << std::endl;
    }

    
    for (size_t i = 0; i < paths.size(); i++)
    {
        std::cout << "Robot " << i + 1 << " path (" << paths[i].size() << " points):" << std::endl;
        for (size_t j = 0; j < paths[i].size(); j++)
        {
            auto coords = TMSTCStar::indexToCoord(paths[i][j], map[0].size() * 2);
            std::cout << "  (" << coords.first  << "," << coords.second  << ")" << "|ix " << paths[i][j] <<"|";
            if (j < paths[i].size() - 1)
                std::cout << " -> ";
        }
        std::cout << std::endl;
    }

    return 0;
}
