#ifndef _TMSTC_STAR_H
#define _TMSTC_STAR_H

#include <vector>
#include <string>
#include <memory>
#include <iostream>
#include <algorithm>
#include <cmath>
#include "PathCut.h"
#include "MaximumSubRectDivision.h"
#include "HeuristicPartition.h"
#include "ACO_STC.h"
#include "Dinic.h"

typedef vector<vector<int>> Mat;

struct TMSTCStarConfig
{
    std::string allocate_method = "MSTC"; // "MSTC", "MTSP"
    std::string mst_shape = "RECT_DIV";   // "RECT_DIV", "DFS_VERTICAL", "DFS_HORIZONTAL", "KRUSKAL", "ACO_OPT", "DINIC", "HEURISTIC"
    int robot_num = 1;
    bool cover_and_return = false;
    int max_iterations = 1000; // For iterative algorithms
    VehicleParameters vehicle_params = {0.8, 0.6, 0.5, 0.5}; // rad/s, m/s^2, m/s, meters
    std::function<bool(int)> is_point_filtered_func = nullptr; // Function to determine if a point should be ignored
};

class TMSTCStar
{
private:
    Mat map_;                         // Original occupancy grid (0 = obstacle, 1 = free)
    Mat region_;                      // Expanded region map
    std::vector<int> robot_init_pos_; // Robot positions as indices in the grid
    TMSTCStarConfig config_;

    // Core algorithm components
    int smallrows_, smallcols_; // Map dimensions
    int bigrows_, bigcols_;     // Region dimensions
    Mat mst_;
    Mat paths_; // Final paths

    // Helper methods
    void preprocessMap();
    void showMapInfo();
    bool isSameLine(int a, int b, int c) { return a + c == 2 * b; }
    void getPathInfo();

public:

    TMSTCStar(const TMSTCStarConfig &config = TMSTCStarConfig()): config_(config) {};

    // Constructor accepting a grid map and robot positions
    TMSTCStar(const Mat &map, const std::vector<int> &robot_positions,
              const TMSTCStarConfig &config = TMSTCStarConfig());

    // Alternative constructor with 2D coordinates instead of indices
    TMSTCStar(const Mat &map, const std::vector<std::pair<int, int>> &robot_positions,
              const TMSTCStarConfig &config = TMSTCStarConfig());

    TMSTCStarConfig getConfig() const { return config_; }
    TMSTCStarConfig& getConfig() { return config_; }

    void reconfigureMapRobot(const Mat &map, const std::vector<int> &robot_positions);

    // Calculate paths using the specified method
    Mat calculateRegionIndxPaths();

    // Get path statistics
    struct PathStats
    {
        int total_length = 0;
        int total_turns = 0;
        double total_cost = 0.0;
        double max_path_length = 0.0;
        double min_path_length = 0.0;
    };
    PathStats getPathStatistics();

    std::vector<std::vector<std::pair<int, int>>> pathsIndxToRegionCoords(Mat paths_indx) const;

    std::pair<int, int> indexToRegionCoord(int index) const
    {
        return indexToCoord(index, bigcols_);
    }
    std::pair<int, int> indexToSpanningCoord(int index) const
    {
        return indexToCoord(index, smallcols_);
    }

    int regionCoordToIndex(int x, int y) const
    {
        return y * bigcols_ + x;
    }

    // Utility for converting between 2D coordinates and linear indices
    static int coordToIndex(int x, int y, int width)
    {
        return y * width + x;
    }

    static std::pair<int, int> indexToCoord(int index, int width)
    {
        return {index % width, index / width};
    }

    // Remove isolated regions from the map
    void eliminateIslands();

    // Get the computed paths
    const Mat &getPaths() const
    {
        return paths_;
    }

    Mat getRegion() const { return region_; }
    // Path shortening and checkpoint generation
    Mat shortenPaths(Mat &checkpoints, int interval = 4);

    Mat removeDuplicateEdgesOnPath();
};

#endif
