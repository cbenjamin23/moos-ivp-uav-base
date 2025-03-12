#include <iostream>
#include <fstream>
#include <string>
#include <ctime>

#include "TMSTCStar.h"

#define DEBUG

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

    // Load the map
    Mat map = loadMapFromFile(mapfile);
    if (map.empty())
    {
        std::cerr << "Failed to load map." << std::endl;
        return 1;
    }

    // Create robot positions (for demonstration, we'll place them at corners and center)
    std::vector<std::pair<int, int>> robot_positions;
    if (robot_count >= 1)
    {
        robot_positions.push_back({0, 0}); // Top-left
    }
    if (robot_count >= 2)
    {
        robot_positions.push_back({(int)map[0].size() - 1, (int)map.size() - 1}); // Bottom-right
    }
    if (robot_count >= 3)
    {
        robot_positions.push_back({(int)map[0].size() - 1, 0}); // Top-right
    }
    if (robot_count >= 4)
    {
        robot_positions.push_back({0, (int)map.size() - 1}); // Bottom-left
    }
    // Add more robots at random positions if needed
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

    // Configure the TMSTC* algorithm
    TMSTCStarConfig config;
    config.allocate_method = method;
    config.mst_shape = shape;
    config.robot_num = robot_count;
    config.cover_and_return = return_to_start;
    config.one_turn_value = 2.0; // Default turning cost

    // Create the TMSTC* instance and calculate paths
    TMSTCStar tmstc(map, robot_positions, config);
    tmstc.eliminateIslands(); // Remove unreachable areas

    std::cout << "Calculating paths..." << std::endl;
    Mat paths = tmstc.calculatePaths();

    // Get and print path statistics
    auto stats = tmstc.getPathStatistics();
    std::cout << "Path Statistics:" << std::endl;
    std::cout << "  Total length: " << stats.total_length << std::endl;
    std::cout << "  Total turns: " << stats.total_turns << std::endl;
    std::cout << "  Total cost: " << stats.total_cost << std::endl;
    std::cout << "  Max path cost: " << stats.max_path_length << std::endl;
    std::cout << "  Min path cost: " << stats.min_path_length << std::endl;

    // Print paths in a readable format
    for (size_t i = 0; i < paths.size(); i++)
    {
        std::cout << "Robot " << i << " path (" << paths[i].size() << " points):" << std::endl;
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

    return 0;
}
