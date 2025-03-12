# TMSTC\* Multi-Robot Coverage Path Planning Library

TMSTC\* (Tree-based Multi-Robot Spanning Tree Coverage) is a C++ library for calculating efficient coverage paths for multiple robots. The library provides ROS-independent implementations of various path planning algorithms based on different spanning tree construction methods.

## Overview

The TMSTC\* library offers:

- Multiple spanning tree construction algorithms (RECT_DIV, DFS, Kruskal, etc.)
- Path allocation methods for multi-robot scenarios
- Turn-minimal path planning
- Coverage path planning with or without return-to-start capabilities

## Installation

### Prerequisites

- C++14 or higher
- CMake 3.0 or higher

### Building the Library

1. Navigate to the TMSTC\* directory:

   ```bash
   cd /path/to/moos-ivp-uav/src/lib_tmstc_star
   ```

2. Create a build directory:

   ```bash
   mkdir -p build && cd build
   ```

3. Run CMake and build:
   ```bash
   cmake ..
   make
   ```

## Basic Usage

### Step 1: Include the necessary headers

```cpp
#include "TMSTCStar.h"
```

### Step 2: Create a map representation

Maps are represented as 2D grids where `1` represents free space and `0` represents obstacles:

```cpp
// Create a simple 5x5 map with obstacles
Mat map = {
    {1, 1, 1, 1, 1},
    {1, 0, 0, 0, 1},
    {1, 1, 1, 1, 1},
    {1, 0, 1, 0, 1},
    {1, 1, 1, 1, 1}
};
```

You can also load maps from files (see example_main.cpp for an implementation).

### Step 3: Define robot positions

Specify the initial positions of your robots:

```cpp
// Define robot positions as 2D coordinates
std::vector<std::pair<int, int>> robot_positions = {
    {0, 0},     // Robot 1 at top-left
    {4, 4}      // Robot 2 at bottom-right
};

// Alternatively, define positions as indices in the expanded map
// std::vector<int> robot_indices = {0, 24};
```

### Step 4: Configure the algorithm

```cpp
TMSTCStarConfig config;
config.allocate_method = "MSTC";           // "MSTC" or "MTSP"
config.mst_shape = "RECT_DIV";             // Spanning tree construction method
config.robot_num = 2;                      // Number of robots
config.cover_and_return = true;            // Return to starting positions
config.one_turn_value = 2.0;               // Cost penalty for turns
config.max_iterations = 1000;              // For iterative algorithms
```

### Step 5: Calculate paths

```cpp
// Create a TMSTC* instance
TMSTCStar tmstc(map, robot_positions, config);

// Remove unreachable areas (optional)
tmstc.eliminateIslands();

// Calculate the paths
Mat paths = tmstc.calculatePaths();

// Get statistics
auto stats = tmstc.getPathStatistics();
std::cout << "Total path length: " << stats.total_length << std::endl;
std::cout << "Total turns: " << stats.total_turns << std::endl;
```

## Available Configuration Options

### Allocation Methods

- `MSTC`: Multi-Robot Spanning Tree Coverage
- `MTSP`: Multiple Traveling Salesman Problem approach (limited support)

### Spanning Tree Construction Methods

- `RECT_DIV`: Rectangle division-based spanning tree
- `DFS_VERTICAL`: Depth-first search with vertical priority
- `DFS_HORIZONTAL`: Depth-first search with horizontal priority
- `KRUSKAL`: Minimum spanning tree using Kruskal's algorithm
- `ACO_OPT`: Ant Colony Optimization
- `DINIC`: Network flow-based spanning tree
- `HEURISTIC`: Heuristic partitioning approach

## Example Program

The library includes an example program that demonstrates how to use the TMSTC\* library:

```bash
# Build the example
g++ -std=c++14 example_main.cpp TMSTCStar.cpp PathCut.cpp -o tmstc_example

# Run with default settings
./tmstc_example

# Run with custom settings
./tmstc_example --map example_map.txt --method MSTC --shape RECT_DIV --robots 3 --return
```

### Command-line Options

- `--map FILE`: Path to the map file (default: example_map.txt)
- `--method METHOD`: Allocation method - MSTC or MTSP (default: MSTC)
- `--shape SHAPE`: MST shape - RECT_DIV, DFS_VERTICAL, etc. (default: RECT_DIV)
- `--robots N`: Number of robots (default: 1)
- `--return`: Enable return to starting position after coverage
- `--help`: Show help information

## Map File Format

The first line contains two integers representing the height and width of the map.
Subsequent lines represent the map with '1' for free space and '0' for obstacles:
