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
    double one_turn_value = 2.0;
    int max_iterations = 1000; // For iterative algorithms
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
    // Constructor accepting a grid map and robot positions
    TMSTCStar(const Mat &map, const std::vector<int> &robot_positions,
              const TMSTCStarConfig &config = TMSTCStarConfig());

    // Alternative constructor with 2D coordinates instead of indices
    TMSTCStar(const Mat &map, const std::vector<std::pair<int, int>> &robot_positions,
              const TMSTCStarConfig &config = TMSTCStarConfig());

    // Calculate paths using the specified method
    Mat calculatePaths();

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
