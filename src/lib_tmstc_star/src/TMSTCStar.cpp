#include "TMSTCStar.h"

TMSTCStar::TMSTCStar(const Mat &map, const std::vector<int> &robot_positions,
                     const TMSTCStarConfig &config)
    : map_(map), robot_init_pos_(robot_positions), config_(config)
{

    // Set global turning cost value
    ONE_TURN_VAL = config.one_turn_value;

    // Initialize dimensions
    smallrows_ = map_.size();
    smallcols_ = map_[0].size();
    bigrows_ = smallrows_ * 2;
    bigcols_ = smallcols_ * 2;

    // Preprocess the map to create the expanded region
    preprocessMap();
    showMapInfo();
}

TMSTCStar::TMSTCStar(const Mat &map, const std::vector<std::pair<int, int>> &robot_positions,
                     const TMSTCStarConfig &config)
    : map_(map), config_(config)
{

    // Set global turning cost value
    ONE_TURN_VAL = config.one_turn_value;

    // Initialize dimensions
    smallrows_ = map_.size();
    smallcols_ = map_[0].size();
    bigrows_ = smallrows_ * 2;
    bigcols_ = smallcols_ * 2;

    // Convert coordinate positions to indices
    robot_init_pos_.resize(robot_positions.size());
    for (size_t i = 0; i < robot_positions.size(); i++)
    {

        auto indx = coordToIndex(robot_positions[i].first * 2, robot_positions[i].second * 2, bigcols_);

        robot_init_pos_[i] = indx;
        std::cout << "Robot " << i + 1 << " position: (" << robot_positions[i].first << ","
                  << robot_positions[i].second << ") -> index: " << robot_init_pos_[i] << std::endl;

        auto coord = indexToCoord(robot_init_pos_[i], bigcols_);
        std::cout << "Index " << robot_init_pos_[i] << " -> reg coordinates: (" << coord.first << ","
                  << coord.second << ")" << std::endl;

        if (std::find(robot_init_pos_.begin(), robot_init_pos_.end(), indx) == robot_init_pos_.end())
        {
            throw std::runtime_error("Robot positions must be unique and within the map boundaries.");
        }
    }

    // Preprocess the map to create the expanded region
    preprocessMap();
    showMapInfo();
}

void TMSTCStar::reconfigureMapRobot(const Mat &map, const std::vector<int> &robot_positions)
{
    map_ = map;
    robot_init_pos_ = robot_positions;

    // Set global turning cost value
    ONE_TURN_VAL = config_.one_turn_value;

    // Reconfigure dimensions
    smallrows_ = map_.size();
    smallcols_ = map_[0].size();
    bigrows_ = smallrows_ * 2;
    bigcols_ = smallcols_ * 2;

    // Preprocess the map to create the expanded region
    preprocessMap();
    showMapInfo();
}


void TMSTCStar::preprocessMap()
{
    // Create expanded region map (2x scale of original map)
    region_.resize(bigrows_, std::vector<int>(bigcols_, 0));

    for (int i = 0; i < smallrows_; ++i)
    {
        for (int j = 0; j < smallcols_; ++j)
        {
            // For each cell in original map, create 4 cells in region map
            region_[2 * i][2 * j] = map_[i][j];
            region_[2 * i][2 * j + 1] = map_[i][j];
            region_[2 * i + 1][2 * j] = map_[i][j];
            region_[2 * i + 1][2 * j + 1] = map_[i][j];
        }
    }
}

void TMSTCStar::showMapInfo()
{
    int free_cells = 0;
    for (int i = 0; i < smallrows_; ++i)
    {
        for (int j = 0; j < smallcols_; ++j)
        {
            if (map_[i][j])
                free_cells++;
        }
    }

    std::cout << "Map dimensions: " << smallrows_ << " x " << smallcols_ << std::endl;
    std::cout << "Region dimensions: " << bigrows_ << " x " << bigcols_ << std::endl;
    std::cout << "Free cells in map: " << free_cells << std::endl;
    std::cout << "Robot count: " << robot_init_pos_.size() << std::endl;
}

void TMSTCStar::getPathInfo()
{
    std::cout << "--------------------------------------" << std::endl;
    std::cout << "Path information:" << std::endl;
    // std::cout << "path size: " << paths_.size() << std::endl;
    for (int i = 0; i < paths_.size(); ++i)
    {
        // std::cout << "Number of paths for robot " << i << ": " << std::flush;
        // std::cout << paths_[i].size() << std::endl;

        int turns = 0;
        for (int j = 1; j < paths_[i].size() - 1; ++j)
        {
            turns += isSameLine(paths_[i][j - 1], paths_[i][j], paths_[i][j + 1]) ? 0 : 1;
        }

        std::cout << "Path " << i << ": length=" << paths_[i].size()
                  << ", turns=" << turns
                //   << ", total_cost=" << (1.0 * paths_[i].size() + ONE_TURN_VAL * turns)
                  << ", total_cost=" << computePathCost(paths_[i], config_.vehicle_params, smallcols_)
                  << std::endl;
    }
}

Mat TMSTCStar::calculateRegionIndxPaths()
{
    std::cout << "Calculating paths with " << config_.allocate_method << " using "
              << config_.mst_shape << " shape..." << std::endl;

    if (config_.allocate_method == "MSTC")
    {
        Division div(map_);

        // Create the MST based on the shape configuration
        if (config_.mst_shape == "RECT_DIV")
        {
            mst_ = div.rectDivisionSolver();
        }
        else if (config_.mst_shape == "DFS_VERTICAL")
        {
            mst_ = div.dfsWithStackSolver(VERTICAL);
        }
        else if (config_.mst_shape == "DFS_HORIZONTAL")
        {
            mst_ = div.dfsWithStackSolver(HORIZONTAL);
        }
        else if (config_.mst_shape == "KRUSKAL")
        {
            mst_ = div.kruskalSolver();
        }
        else if (config_.mst_shape == "ACO_OPT")
        {
            ACO_STC aco(1, 1, 1, 0.15, 60, config_.max_iterations, map_, mst_);
            mst_ = aco.aco_stc_solver();
        }
        else if (config_.mst_shape == "DINIC")
        {
            mst_ = dinic.dinic_solver(map_, true);
        }
        else if (config_.mst_shape == "HEURISTIC")
        {
            HeuristicSolver::HeuristicPartition hp(map_, config_.max_iterations);
            mst_ = hp.hpSolver(true);
        }
        else
        {
            std::cout << "Unknown MST shape: " << config_.mst_shape << ", defaulting to RECT_DIV" << std::endl;
            mst_ = div.rectDivisionSolver();
        }

        // Create PathCut solver
        PathCut cut(map_, region_, mst_, robot_init_pos_, config_.vehicle_params ,config_.cover_and_return);
        cut.setOneTurnVal(config_.one_turn_value);
        paths_ = cut.cutSolver();
    }
    else if (config_.allocate_method == "MTSP")
    {
        // MTSP implementation would go here
        std::cout << "MTSP implementation not included in this version" << std::endl;
        paths_ = Mat(robot_init_pos_.size());
    }
    else
    {
        std::cout << "Unknown allocation method: " << config_.allocate_method << ", defaulting to MSTC" << std::endl;

        Division div(map_);
        mst_ = div.rectDivisionSolver();

        PathCut cut(map_, region_, mst_, robot_init_pos_, config_.vehicle_params, config_.cover_and_return);
        cut.setOneTurnVal(config_.one_turn_value);
        paths_ = cut.cutSolver();
    }

    getPathInfo();
    return paths_;
}

Mat TMSTCStar::shortenPaths(Mat &checkpoints, int interval_max)
{
    if (paths_.empty())
    {
        return paths_;
    }

    int robot_num = paths_.size();
    Mat shortened_paths(robot_num, std::vector<int>{});
    checkpoints.resize(robot_num);

    for (int i = 0; i < robot_num; ++i)
    {
        // Always include the first point
        shortened_paths[i].push_back(paths_[i][0]);
        checkpoints[i].push_back(0);

        int interval = interval_max;
        bool is_checkpoint = false;

        // Process intermediate points
        for (size_t step = 1; step < paths_[i].size() - 1; step++)
        {
            // Check if this point is a turning point
            is_checkpoint = !isSameLine(paths_[i][step - 1], paths_[i][step], paths_[i][step + 1]);

            if (!is_checkpoint && interval)
            {
                // Skip this point if it's not a turn and we haven't reached the interval
                interval--;
            }
            else
            {
                // If this is a turning point, mark it as a checkpoint
                if (is_checkpoint)
                {
                    checkpoints[i].push_back(shortened_paths[i].size());
                }

                // Add this point to the shortened path
                shortened_paths[i].push_back(paths_[i][step]);
                interval = interval_max; // Reset the interval
            }
        }

        // Always include the last point
        checkpoints[i].push_back(shortened_paths[i].size());
        shortened_paths[i].push_back(paths_[i].back());
    }

    std::cout << "Path shortening: reduced from ";
    for (int i = 0; i < robot_num; ++i)
    {
        std::cout << paths_[i].size() << " to " << shortened_paths[i].size()
                  << " points (Robot " << i + 1 << "), ";
    }
    std::cout << std::endl;

    return shortened_paths;
}

TMSTCStar::PathStats TMSTCStar::getPathStatistics()
{
    PathStats stats;

    if (paths_.empty())
    {
        return stats;
    }

    stats.min_path_length = 1e9;

    for (const auto& path : paths_) {
        if (path.empty()) continue;
        int path_turns = 0;
        for (size_t j = 1; j < path.size() - 1; ++j) {
            if (!isSameLine(path[j - 1], path[j], path[j + 1])) {
                path_turns++;
            }
        }
        
        stats.total_length += path.size() > 0 ? path.size() - 1 : 0;
        stats.total_turns += path_turns;
        double path_cost = computePathCost(path, config_.vehicle_params, smallcols_);
        stats.total_cost += path_cost;
        stats.max_path_length = std::max(stats.max_path_length, path_cost);
        stats.min_path_length = std::min(stats.min_path_length, path_cost);
    }

    return stats;
}

void TMSTCStar::eliminateIslands()
{
    // Simple flood fill from the first robot position to eliminate unreachable areas
    std::vector<std::vector<bool>> visited(smallrows_, std::vector<bool>(smallcols_, false));
    std::queue<std::pair<int, int>> q;

    // Start from first robot position
    auto start_pos = indexToCoord(robot_init_pos_[0], bigcols_);
    start_pos.first /= 2; // Convert from region coordinates to map coordinates
    start_pos.second /= 2;

    q.push(start_pos);
    visited[start_pos.second][start_pos.first] = true;

    int dx[] = {0, 0, 1, -1};
    int dy[] = {1, -1, 0, 0};

    while (!q.empty())
    {
        auto [x, y] = q.front();
        q.pop();

        for (int i = 0; i < 4; i++)
        {
            int nx = x + dx[i];
            int ny = y + dy[i];

            if (nx >= 0 && ny >= 0 && nx < smallcols_ && ny < smallrows_ &&
                map_[ny][nx] && !visited[ny][nx])
            {
                visited[ny][nx] = true;
                q.push({nx, ny});
            }
        }
    }

    // Update map to only include connected areas
    for (int i = 0; i < smallrows_; ++i)
    {
        for (int j = 0; j < smallcols_; ++j)
        {
            if (map_[i][j] && !visited[i][j])
            {
                map_[i][j] = 0; // Mark unreachable free cells as obstacles
            }
        }
    }

    // Update the region map as well
    preprocessMap();
}

Mat TMSTCStar::removeDuplicateEdgesOnPath()
{
    for (int i = 0; i < paths_.size(); ++i)
    {
        if (paths_[i].size() == 0)
        {
            continue;
        }
        std::vector<int> new_path;
        new_path.push_back(paths_[i][0]);

        P prev_coords = TMSTCStar::indexToCoord(paths_[i][0], map_[0].size() * 2);
        prev_coords.first /= 2; // convert to map coordinates
        prev_coords.second /= 2;

        for (int j = 1; j < paths_[i].size() - 1; ++j)
        {

            P coords = TMSTCStar::indexToCoord(paths_[i][j], map_[0].size() * 2);
            coords.first /= 2; // convert to map coordinates
            coords.second /= 2;

            // std::cout << "First equal: " << (prev_coords.first==coords.first) << " Second equal: " << (prev_coords.second==coords.second) << std::endl;
            bool equal = (prev_coords.first == coords.first) && (prev_coords.second == coords.second);

            if (equal)
            {
                // std::cout << "Skipping " << paths_[i][j] << std::endl;
                continue;
            }

            std::cout << (equal ? "true" : "false") << " Adding " << paths_[i][j] << " (" << coords.first << ", " << coords.second << ")" << " prev: (" << prev_coords.first << ", " << prev_coords.second << ")" << std::endl;

            new_path.push_back(paths_[i][j]);
            prev_coords = coords;
        }
        new_path.push_back(paths_[i].back());
        paths_[i] = new_path;

        std::cout << "New path size: " << paths_[i].size() << std::endl;
    }

    return paths_;
}

std::vector<std::vector<std::pair<int, int>>> TMSTCStar::pathsIndxToRegionCoords(Mat paths_indx) const
{
    std::vector<std::vector<std::pair<int, int>>> paths;
    for (int i = 0; i < paths_indx.size(); ++i)
    {
        std::vector<std::pair<int, int>> path;
        for (int j = 0; j < paths_indx[i].size(); ++j)
        {
            path.push_back(indexToRegionCoord(paths_indx[i][j]));
        }
        paths.push_back(path);
    }
    return paths;
}