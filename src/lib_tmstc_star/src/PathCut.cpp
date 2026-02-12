#include "PathCut.h"

#include "../../lib_common/Logger.h"

#include <functional>
#include <limits>

#include<set>

#include "GeomUtils.h"

// Define a hash function for std::tuple
struct tuple_hash
{
	template <class T>
	std::size_t operator()(const T &tuple) const
	{
		return std::hash<std::string>{}(std::to_string(std::get<0>(tuple)) + "," +
										std::to_string(std::get<1>(tuple)) + "," +
										std::to_string(std::get<2>(tuple)));
	}
};

// Filter valid points from a path
std::vector<int> PathCut::filterValidPoints(const std::vector<int> &path)
{
	std::vector<int> valid_points;

	// If no filtering function is provided, consider all points valid
	if (!is_point_filtered_func)
	{
		return path;
	}

	// //print all depos
	// for (auto d : depot)
	// {
	// 	std::cout << "Depot: " << d << std::endl;
	// 	Logger::info("Depot: " + std::to_string(d));
	// }

	std::set<int> ignored_points;

	for (int i = 0; i < path.size(); ++i)
	{
		int point = path[i];

		if (!is_point_filtered_func(point))
		{
			valid_points.push_back(point);
		}
		else
		{
			ignored_points.insert(point); // Store ignored points for later reference
			// std::cout << "Filtered point: " << point << std::endl;
			
			// If point is any depot, change depot to next point
			auto it = std::find(depot.begin(), depot.end(), point);
			if (it != depot.end())
			{
				
				
				
				const int max_iterations = 1000;
				// If point is depot, change depot to next point
				// keep changing until next point is not already a depot
				auto inc = 1;
				auto next_depot = std::find(depot.begin(), depot.end(), path[(i + inc) % path.size()]); 
				while ( 
					( next_depot != depot.end() || ignored_points.find( path[(i + inc) % path.size()]) != ignored_points.end())
					&& (inc < max_iterations)
				)
				{
					// std::cout << "Changing depot to next point: " << path[(i + inc) % path.size()] << std::endl;
					// Logger::info("Changing depot to next point: " + std::to_string(path[(i + inc) % path.size()]));
					
					inc++;
					next_depot = std::find(depot.begin(), depot.end(), path[(i + inc) % path.size()]);
				}

				*it = path[(i + inc) % path.size()]; // Change depot to next point
				
				if (inc >= max_iterations)
				{
					std::cout << "Max iterations reached while changing depot to next point: " << path[(i + inc) % path.size()] << std::endl;
					Logger::error("Max iterations reached while changing depot to next point: " + std::to_string(path[(i + inc) % path.size()]));
				}
				
				std::cout << "Changed depot to next point: " << *it << std::endl;
				Logger::info("Changed depot to next point: " + std::to_string(*it));
			}
		}
	}

	//print all depos
	// for (auto d : depot)
	// {
	// 	std::cout << "Depot: " << d << std::endl;
	// 	Logger::info("Depot: " + std::to_string(d));
	// 	if(std::find(valid_points.begin(), valid_points.end(), d) == valid_points.end())
	// 	{
	// 		std::cout << "Depot not in valid points: " << d << std::endl;
	// 		Logger::error("Depot not in valid points: " + std::to_string(d));
	// 	}
	// }

	return valid_points;
}

// MST2Path is undirected graph, so it contains (i, j) and (j, i)
void PathCut::MST2Path()
{
	vector<unordered_set<int>> vis(Map.size() * Map[0].size(), std::unordered_set<int>{});
	pathEdge.resize(bigcols * bigrows, vector<int>{});

	// ROS uses x and y when arranging paths, be careful here, bigcols corresponds to x in mapServer map
	for (int from = 0; from < MST.size(); ++from)
	{
		for (auto to : MST[from])
		{
			if ((!vis[from].empty() && vis[from].find(to) != vis[from].end()) ||
				(!vis[to].empty() && vis[to].find(from) != vis[to].end()))
				continue;

			vis[from].insert(to);
			vis[to].insert(from);

			int x1, x2, y1, y2;
			get2DCoordinateMap(from, x1, y1);
			get2DCoordinateMap(to, x2, y2);
			int p3 = 2 * x1 * bigcols + 2 * y1 + 1;
			int p4 = 2 * x2 * bigcols + 2 * y2;
			int p1 = (2 * x1 + 1) * bigcols + (2 * y1 + 1);
			int p2 = (2 * x2 + 1) * bigcols + 2 * y2;
			int p6 = 2 * x1 * bigcols + 2 * y1;
			int p5 = (2 * x1 + 1) * bigcols + 2 * y1;
			int p8 = 2 * x2 * bigcols + 2 * y2 + 1;
			int p7 = (2 * x2 + 1) * bigcols + 2 * y2 + 1;

			if (abs(from - to) == 1)
			{
				// horizontal edges
				pathEdge[p1].push_back(p2);
				pathEdge[p3].push_back(p4);
				pathEdge[p2].push_back(p1);
				pathEdge[p4].push_back(p3);
			}
			else
			{
				// vertical edges (x, y) creates (2*x + col, 2*y) and (2*x+col+1, 2*y + 1)
				pathEdge[p4].push_back(p5);
				pathEdge[p1].push_back(p8);
				pathEdge[p5].push_back(p4);
				pathEdge[p8].push_back(p1);
			}
		}
	}

	for (int i = 0; i < Map.size(); ++i)
	{
		for (int j = 0; j < Map[0].size(); ++j)
		{
			if (!Map[i][j]) // Skip obstacles
				continue;
			int x = 2 * i, y = 2 * j;
			int cur = i * smallcols + j;
			int p1 = x * bigcols + y;
			int p2 = x * bigcols + y + 1;
			int p3 = (x + 1) * bigcols + y;
			int p4 = (x + 1) * bigcols + y + 1;

			if (j == 0 || vis[cur].find(cur - 1) == vis[cur].end())
			{
				pathEdge[p1].push_back(p3);
				pathEdge[p3].push_back(p1);
			}
			if (j == smallcols - 1 || vis[cur].find(cur + 1) == vis[cur].end())
			{
				pathEdge[p2].push_back(p4);
				pathEdge[p4].push_back(p2);
			}
			if (i == 0 || vis[cur].find(cur - smallcols) == vis[cur].end())
			{
				pathEdge[p1].push_back(p2);
				pathEdge[p2].push_back(p1);
			}
			if (i == smallrows - 1 || vis[cur].find(cur + smallcols) == vis[cur].end())
			{
				pathEdge[p3].push_back(p4);
				pathEdge[p4].push_back(p3);
			}
		}
	}

	std::cout << "Generating: Get path edges\n";

	// After getting pathEdge, next step is to obtain the path sequence pathSequence and its reverse sequence
	// pathSequence starts from the position of the first robot
	// pathSequence's length equals the circle length, invSequence is different
	vector<bool> inPath(Region[0].size() * Region.size(), false);
	int cur = depot[0];
	while (!inPath[depot[0]] || cur != depot[0])
	{
		inPath[cur] = true;
		pathSequence.push_back(cur);

		if (pathEdge[cur].size() == 0)
		{
			std::cout << "Generating: Edge set crash\n";
			throw std::runtime_error("Edge set crash");
		}

		cur = inPath[pathEdge[cur][0]] ? pathEdge[cur][1] : pathEdge[cur][0];
		if (inPath[cur])
			break;
	}

	// Get cyclic path (original implementation)
	circleLen = pathSequence.size();

	// STEP 1: Filter points if a filtering function is provided
	std::vector<int> original_path = pathSequence;
	std::vector<int> valid_points;

	std::cout << "Original points: " << original_path.size() << std::endl;
	Logger::info("TMSTC - Original points: " + std::to_string(original_path.size()));

	if (is_point_filtered_func)
	{
		std::cout << "Filtering points based on custom criteria..." << std::endl;
		valid_points = filterValidPoints(original_path);

		if (valid_points.size() < original_path.size())
		{
			std::cout << "Filtered out " << (original_path.size() - valid_points.size())
					  << " points (" << (100.0 * (original_path.size() - valid_points.size()) / original_path.size())
					  << "% reduction)" << std::endl;
			Logger::info("TMSTC - Filtered out " + std::to_string(original_path.size() - valid_points.size()) +
						 " points (" + std::to_string(100.0 * (original_path.size() - valid_points.size()) / original_path.size()) + "% reduction)");
			// If we have a very small number of valid points, maintain the original path
			if (valid_points.size() < 3)
			{
				std::cout << "Too few valid points. Using original path." << std::endl;
				valid_points = original_path;
			}
		}
		else
		{
			std::cout << "No points were filtered out." << std::endl;
		}
	}
	else
	{
		valid_points = original_path;
	}

	// STEP 2: If points were filtered, compute an optimized tour through valid points
	if (valid_points.size() < original_path.size() && valid_points.size() >= 3)
	{
		std::cout << "Optimizing path with outliers..." << std::endl;
		Logger::info("TMSTC - Optimizing path with outliers...");
		optimizePathWithOutliersAndUpdateSequence(valid_points);
		circleLen = pathSequence.size();

		Logger::info("TMSTC - Path optimized with outliers: " + std::to_string(pathSequence.size()) + " points");
		std::cout << "Path optimized with outliers: " << pathSequence.size() << " points" << std::endl;
	}

	std::cout << "Generating: Trim labels\n";
	// Convert label->sequential label
	invSequence.resize(Region.size() * Region[0].size(), -1);
	for (int i = 0; i < pathSequence.size(); ++i)
	{
		invSequence[pathSequence[i]] = i; // Maps each fine-grid cell index to its position in pathSequence, useful for lookups.
	}

	// construct path value vec
	// Turning once is equivalent to going through 2 more cells
	// Note: turning from the first to the last connected point also counts

#ifdef OLD_COST
	pathValue.resize(2 * circleLen, 1.0);
	for (int i = 1; i < 2 * circleLen - 1; ++i)
	{
		if (!isSameLine(pathSequence[(i - 1 + circleLen) % circleLen], pathSequence[i % circleLen], pathSequence[(i + 1) % circleLen]) && i != 2 * circleLen - 1)
			pathValue[i] += ONE_TURN_VAL;

		pathValue[i] += pathValue[i - 1];
	}
	pathValue[2 * circleLen - 1] += pathValue[2 * circleLen - 2];
#else
	std::cout << "Generating: Path value with new Cost definition to match speed parameters def\n";
	Logger::info("TMSTC - Generating path value with new Cost definition to match speed parameters def");

	double omega = vehicleParams.omega_rad;		// rad/s (angular velocity)
	double a = vehicleParams.acc;				// m/s^2 (acceleration)
	double vmax = vehicleParams.vmax;			// m/s (max velocity)
	double cellSize = vehicleParams.cellSize_m; // meters (grid cell size)
	double phi = vehicleParams.phi_max_rad;		// rad (max banking angle)

	// Computed turn radius
	const double turn_radius = (vmax * vmax) / (gravity * std::tan(phi)); // Turn radius in meters
	std::function<double(double)> turn_time = [&](double theta){return (turn_radius * theta) / vmax ; };			  // Time for a turn based on angle

	// Resize pathValue to 2 * circleLen to accommodate the double cycle for multiple robots
	pathValue.resize(2 * circleLen, 0.0);
	// Build pathValue with cumulative costs over two cycles
	for (int i = 0; i < 2 * circleLen - 1; ++i)
	{
		int prev = i % circleLen;
		int curr = (i + 1) % circleLen;

		// Calculate distance between consecutive points
		double distance = calculateDistance(pathSequence[prev], pathSequence[curr]);

		// Compute segment time tj
		double tj;
#ifdef USE_UAV_COST
		tj = distance / vmax;
#else
		// Segment time tj: sqrt(4d/a) if d < vmax^2/a, else d/vmax + vmax/a (Time under acceleration limit, else time with max velocity reached)
		tj = (distance < (vmax * vmax) / a) ? std::sqrt(4 * distance / a) : (distance / vmax + vmax / a);
#endif

		// Add segment time to pathValue
		pathValue[i + 1] = pathValue[i] + tj;

		// Check for a turn at curr (skip if at start or end of the double cycle)
		if (i < 2 * circleLen - 2)
		{
			int next = (i + 2) % circleLen;
			if (!isSameLine(pathSequence[prev], pathSequence[curr], pathSequence[next]))
			{
				double turnCost = 0.0;
// Add turn costs (assuming 90-degree turns, cost = turn_time)
#ifdef USE_UAV_COST
				// Vectors: v1 = P_{i-1} to P_i, v2 = P_i to P_{i+1}
				int p0 = pathSequence[prev], p1 = pathSequence[curr], p2 = pathSequence[next];
				double p0x = p0 % smallcols, p0y = p0 / smallcols; // x and y coordinates of p0
				double p1x = p1 % smallcols, p1y = p1 / smallcols; // x and y coordinates of p1
				double p2x = p2 % smallcols, p2y = p2 / smallcols; // x and y coordinates of p2

				// double v1x = (p1 % smallcols - p0 % smallcols) * cellSize; // x-component
				// double v1y = (p1 / smallcols - p0 / smallcols) * cellSize; // y-component
				// double v2x = (p2 % smallcols - p1 % smallcols) * cellSize;
				// double v2y = (p2 / smallcols - p1 / smallcols) * cellSize;
				
				// // Compute turn angle theta
				// double dot = v1x * v2x + v1y * v2y;
				// double det = v1x * v2y - v1y * v2x;
				// double theta = std::abs(std::atan2(det, dot)); // Absolute angle in [0, pi]

				
				double theta = std::abs(segmentAngle(p0x,p0y,p1x,p1y, p2x,p2y)); // Angle between two vectors in radians

				turnCost = turn_time(theta); // 90-degree turn time	// cost = turn_time)
#else
				turnCost = (M_PI / (2 * omega)); // 90-degree turn time // cost = pi/(2*omega))
#endif

				pathValue[i + 1] += turnCost; // Add turn cost after segment
				// Logger::info("TMSTC - Adding cost: " + std::to_string(pathValue[i + 1]) + " at index " + std::to_string(i + 1));
			}
		}
	}
#endif

	cout << "Finish Constructing Path from ideal spanning tree.\n";
	Logger::info("TMSTC - Finish constructing path from ideal spanning tree.");
	// checking path and its value
	/*cout << "display path sequence...\n";
	for (auto i : pathSequence)	cout << i << " ";
	cout << "----------------------------------------------" << endl;
	cout << "display path value...\n";
	for (auto i : pathValue)	cout << i << " ";
	cout << "----------------------------------------------" << endl;*/
}

double PathCut::euclidean_dis(double x1, double y1, double x2, double y2)
{
	return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

#ifdef OLD_COST
// u and v are one-dimensional coordinates under region, when generating edges, we need to convert to 2D coordinates to check boundaries
// To avoid this problem, we need to use flags to judge whether the diagonal edge surrounds an obstacle, otherwise we would later need to set the dis array to infinity; it's better to use A*
// A* can handle coordinate transformations properly
double PathCut::A_star(int u, int v)
{
	// Use A* on the original map and region
	std::priority_queue<node> que;
	int sx = u / bigcols, sy = u % bigcols;
	int ex = u / bigcols, ey = u % bigcols;
	vector<double> dis(bigcols * bigrows, 2e9);
	vector<int> pre(bigcols * bigrows, -1);
	dis[u] = 0;

	que.push({0 + euclidean_dis(1.0 * sx, 1.0 * sy, 1.0 * ex, 1.0 * ey), 0, u});
	while (!que.empty())
	{
		node cur = que.top();
		que.pop();
		if (cur.id == v)
			break;
		if (cur.gx - dis[cur.id] > eps)
			continue;

		int cx = cur.id / bigcols, cy = cur.id % bigcols;
		for (int i = 0; i < 4; ++i)
		{
			int dx = cx + dir[i][0], dy = cy + dir[i][1];
			if (dx < 0 || dx >= bigrows || dy < 0 || dy >= bigcols || !Region[dx][dy])
				continue;

			int nxt_id = dx * bigcols + dy;
			if (dis[nxt_id] > dis[cur.id] + 1.0)
			{
				pre[nxt_id] = cur.id;
				dis[nxt_id] = dis[cur.id] + 1.0;
				que.push({dis[nxt_id] + euclidean_dis(dx, dy, ex, ey), dis[nxt_id], nxt_id});
			}
		}
	}

	int hair = v, head = pre[v], cur;
	double turnVal = 0.0;
	if (head == -1 || head == u)
		return dis[v];

	cur = pre[head];
	do
	{
		if (!isSameLine(hair, head, cur))
			turnVal += ONE_TURN_VAL;
		hair = head;
		head = cur;
		cur = pre[cur];
	} while (head != u);

	return dis[v] + turnVal;
}

#else

double PathCut::A_star(int u, int v)
{
	std::priority_queue<node> que; // node: {fx, gx, id}

	double a = vehicleParams.acc;
	double omega = vehicleParams.omega_rad;
	double vmax = vehicleParams.vmax;
	double cellSize = vehicleParams.cellSize_m;

	vector<double> dis(bigcols * bigrows, 2e9);
	vector<int> pre(bigcols * bigrows, -1);
	dis[u] = 0;

	int sx = u / bigcols, sy = u % bigcols;
	int ex = v / bigcols, ey = v % bigcols;
	que.push({euclidean_dis(sx, sy, ex, ey), 0, u});

	while (!que.empty())
	{
		node cur = que.top();
		que.pop();
		if (cur.id == v)
			break;
		if (cur.gx > dis[cur.id] + eps)
			continue;

		int cx = cur.id / bigcols, cy = cur.id % bigcols;
		for (int i = 0; i < 4; ++i)
		{
			int dx = cx + dir[i][0], dy = cy + dir[i][1];
			if (dx < 0 || dx >= bigrows || dy < 0 || dy >= bigcols || !Region[dx][dy])
				continue;

			int nxt_id = dx * bigcols + dy;
			double distance = euclidean_dis(cx, cy, dx, dy) * cellSize;
			double tj = (distance < (vmax * vmax) / a) ? sqrt(4 * distance / a) : (distance / vmax + vmax / a);

			// Add turn cost if there's a previous point
			double turnCost = 0;
			if (pre[cur.id] != -1 && !isSameLine(pre[cur.id], cur.id, nxt_id))
			{
				turnCost = M_PI / (2 * omega);
			}

			double newCost = dis[cur.id] + tj + turnCost;
			if (newCost < dis[nxt_id])
			{
				dis[nxt_id] = newCost;
				pre[nxt_id] = cur.id;
				double heuristic = euclidean_dis(dx, dy, ex, ey) * cellSize / vmax; // Time-based heuristic
				que.push({newCost + heuristic, newCost, nxt_id});
			}
		}
	}

	return dis[v];
}

#endif

// Try to use a compressed A* path
vector<int> PathCut::A_star_path(int u, int v)
{
	// cout << "In A star. printing u and v: " << u << " " << v << endl;
	std::priority_queue<node> que;
	int sx = u / bigcols, sy = u % bigcols;
	int ex = v / bigcols, ey = v % bigcols;
	vector<double> dis(bigcols * bigrows, 2e9);
	vector<int> pre(bigcols * bigrows, -1);

	dis[u] = 0;
	// cout << sx << " " << sy << " " << ex << " " << ey << endl;
	// cout << euclidean_dis(1.0 * sx, 1.0 * sy, 1.0 * ex, 1.0 * ey) << endl;
	que.push({0 + euclidean_dis(1.0 * sx, 1.0 * sy, 1.0 * ex, 1.0 * ey), 0, u});
	while (!que.empty())
	{
		node cur = que.top();
		que.pop();
		if (cur.id == v)
		{
			// cout << "reaching the final point!\n";
			break;
		}
		if (cur.gx - dis[cur.id] > eps)
			continue;

		int cx = cur.id / bigcols, cy = cur.id % bigcols;
		for (int i = 0; i < 4; ++i)
		{
			int dx = cx + dir[i][0], dy = cy + dir[i][1];
			if (dx < 0 || dx >= bigrows || dy < 0 || dy >= bigcols || !Region[dx][dy])
				continue;

			int nxt_id = dx * bigcols + dy;
			if (dis[nxt_id] > dis[cur.id] + 1.0)
			{
				pre[nxt_id] = cur.id;
				dis[nxt_id] = dis[cur.id] + 1.0;
				que.push({dis[nxt_id] + euclidean_dis(dx, dy, ex, ey), dis[nxt_id], nxt_id});
			}
		}
	}

	// from tail to head
	vector<int> path;
	int it = v;
	while (it != u)
	{
		// cout << it << endl;
		path.push_back(it);
		it = pre[it];
	}
	path.push_back(u);
	reverse(path.begin(), path.end());

	// shorten the path ?
	// I don't think it's a good idea. ROS will A-star the path again.
	// over-shorten the path may cause the robot move into inflation layer more often

	return path;
}

#ifdef OLD_COST
double PathCut::getTurnAndLength(int i)
{
	// Note: if the end index is smaller than the start index, it means we've added a tail
	// Also, strictly speaking, if the cut contains a complete path, the number of turns should be -1, but we need to be careful with the modulo operation
	int start = cuts[i].start;
	int ending = (cuts[i].start + cuts[i].len - 1 + circleLen) % circleLen;
	double endingTurn = isSameLine(pathSequence[(ending - 1 + circleLen) % circleLen], pathSequence[ending], pathSequence[(ending + 1) % circleLen]) ? 1.0 : 0.0;
	if (start <= ending)
		return pathValue[ending] - pathValue[start] + endingTurn;
	else
		return pathValue[circleLen - 1] - pathValue[start] + pathValue[ending] + endingTurn;
}

#else

double PathCut::getTurnAndLength(int i)
{
	int start = cuts.at(i).start;
	int ending = (cuts.at(i).start + cuts.at(i).len - 1) % circleLen;

	// Handle wrap-around by adjusting ending to the second cycle if needed
	if (ending < start)
	{
		ending += circleLen;
	}

	// Cost is the difference in pathValue
	double cost = pathValue.at(ending) - pathValue.at(start);

	return cost;
}

#endif

// Helper function to compute distance between two grid points
double PathCut::calculateDistance(int idx1, int idx2)
{
	int x1, y1, x2, y2;
	get2DCoordinateMap(idx1, x1, y1);
	get2DCoordinateMap(idx2, x2, y2);

	return euclidean_dis(x1, y1, x2, y2) * vehicleParams.cellSize_m; // Adjust with actual cell size
																	 // Alternatively, if you want to use the original euclidean_dis function:

	// double dx = (x1 - x2) * cellSize; // Adjust with actual cell size
	// double dy = (y1 - y2) * cellSize;
	// return sqrt(dx * dx + dy * dy);
}

double PathCut::updateCutVal(int i)
{
	// Calculate depot to cut start + cut + cut end to depot weight
	int cut_start_region_label = pathSequence[cuts[i].start];
	int cut_end_region_label = pathSequence[(cuts[i].start + cuts[i].len - 1 + circleLen) % circleLen];

	double mainPathCost = getTurnAndLength(i);
	// Logger::info("TMSTC - Main path cost: " + std::to_string(mainPathCost));

	// cover without back to starting point
	if (!coverAndReturn)
	{
		auto cost_to = A_star(depot[cut_depot[i]], cut_start_region_label);

		return 0.5 * cost_to + mainPathCost;
	}
	else
	{
		auto cost_to = A_star(depot[cut_depot[i]], cut_start_region_label);
		auto cost_back = A_star(cut_end_region_label, depot[cut_depot[i]]);


		return 0.5*cost_to + mainPathCost + 0.5*cost_back;
	}
}

void PathCut::MSTC_Star()
{
	// Note: each robot's path starting length cannot be simply taken based on the previous and next, because on a circle, a robot might not be between its preceding and following robots
	// Therefore, we need a mapping that corresponds to the sequential numbers on the circle; the mapping corresponds one-to-one with robot_init_pos
	// So we need to know depot label -> circle label(cuts label)
	vector<int> tmp;
	for (auto x : depot)
		tmp.push_back(invSequence[x]);
	sort(tmp.begin(), tmp.end());

	for (int i = 0; i < depot.size(); ++i)
	{
		for (int j = 0; j < depot.size(); ++j)
		{
			if (invSequence[depot.at(i)] == tmp.at(j))
			{
				depot_cut.at(i) = j;
				cut_depot.at(j) = i;
			}
		}
	}

	double opt = 0, wst = 2e9;
	for (int i = 0; i < depot.size(); ++i)
	{
		cuts.at(i).start = invSequence[depot[cut_depot.at(i)]];
		cuts.at(i).len = (invSequence[depot[cut_depot[(i + 1) % depot.size()]]] - invSequence[depot[cut_depot.at(i)]] + circleLen) % circleLen; // nr - nl + 1 - 1
		cuts.at(i).val = updateCutVal(i);
		opt = std::max(opt, cuts.at(i).val);
		wst = std::min(wst, cuts.at(i).val);

		// Logger::info("TMSTC - opt: " + std::to_string(opt) + " wst: " + std::to_string(wst) + " cut[" + std::to_string(i) + "] val: " + std::to_string(cuts[i].val));
	}

	cout << "opt and wst: " << opt << "  " << wst << endl;
	Logger::info("TMSTC - Initial opt and wst: " + std::to_string(opt) + "  " + std::to_string(wst));

	// Note: MSTC* does not consider that in some cases the path weight of cutting a single path is greater than cutting multiple paths, so in practice it may not converge for some maps. We need to modify it
	int cur_iter = 0; // , max_iter = 10;
	// while (cur_iter < max_iter)
	// infinite loop

	double curr_diff = opt - wst;
	double prev_diff = opt - wst;

	while (opt - wst > 10.0 && cur_iter < maxIterations)
	{
		cur_iter++;
		cout << "MSTC_Star Iteration: " << cur_iter << "\n";
		cout << "cutting for balancing...\n"; // just a sign
		Logger::info("TMSTC - Cutting for balancing... at iteration " + std::to_string(cur_iter));

		double minn = 2e9, maxx = -1;
		int min_cut = -1, max_cut = -1;
		for (int i = 0; i < cuts.size(); ++i)
		{
			if (minn > cuts.at(i).val)
			{
				minn = cuts.at(i).val;
				min_cut = i;
			}
			if (maxx < cuts.at(i).val)
			{
				maxx = cuts.at(i).val;
				max_cut = i;
			}
		}

		curr_diff = maxx - minn;

		cout << "before adjustment opt and wst: " << maxx << "  " << minn << " diff: (" << (maxx - minn) << ")\n";
		Logger::info("TMSTC - Before adjustment opt and wst: " + std::to_string(maxx) + "  " + std::to_string(minn) + " diff: (" + std::to_string(maxx - minn) + ")");
		// Judge whether to go clockwise or counter-clockwise
		vector<int> clw = getHalfCuts(min_cut, max_cut, 1);
		vector<int> ccw = getHalfCuts(min_cut, max_cut, -1);
		Logger::info("before balanced cut");
		clw.size() < ccw.size() ? Balanced_Cut(clw) : Balanced_Cut(ccw);


		opt = 0, wst = 2e9;
		for (int i = 0; i < cuts.size(); ++i)
		{
			opt = std::max(opt, cuts.at(i).val);
			wst = std::min(wst, cuts.at(i).val);
		}

		cout << "after adjustment opt and wst: " << opt << "  " << wst << " diff: (" << (opt - wst) << ")\n";
		Logger::info("TMSTC - After adjustment opt and wst: " + std::to_string(opt) + "  " + std::to_string(wst) + " diff: (" + std::to_string(opt - wst) + ")");

		prev_diff = curr_diff;
		curr_diff = opt - wst;
		if (std::abs(prev_diff - curr_diff) < 10 && opt - wst < 500)
		{
			cout << "MSTC_Star Cutoff finished!\n\n\n";
			Logger::info("TMSTC - MSTC_Star cutoff finished at iteration " + std::to_string(cur_iter));
			break;
		}
		
	}
}

vector<int> PathCut::getHalfCuts(int cut_min, int cut_max, int dir)
{
	vector<int> res;
	int cur_cut = cut_min;
	while (cur_cut != cut_max)
	{
		res.push_back(cur_cut);
		cur_cut = (cur_cut + dir + depot.size()) % depot.size();
	}
	res.push_back(cut_max);

	if (dir == -1)
		std::reverse(res.begin(), res.end()); // If we reverse the order, then from counter-clockwise to clockwise, things will get complicated

	return res;
}

// Only need to update cut's starting point, length and weight, and there's no need to modify the cut vec structure
void PathCut::Balanced_Cut(vector<int> &adjustCuts)
{

	int r_first = adjustCuts.front(), r_last = adjustCuts.back();
	double old_val_max = -1, old_val_sum = 0;
	for (auto &x : adjustCuts)
	{
		old_val_max = std::max(old_val_max, cuts.at(x).val);
		old_val_sum += cuts.at(x).val;
	}



	pair<double, double> res{old_val_max, old_val_sum};
	int old_len_r_first = cuts.at(r_first).len, old_len_r_last = cuts.at(r_last).len;
	double cur_val_max = -1, cur_val_sum = 0;
	bool update_success = false;

	// int lef = 0, rig = cuts.at(r_first).len + cuts.at(r_last).len - 1;  // -1 ensures the divided length is not 0
	double lef = 0, rig = getTurnAndLength(r_first) + getTurnAndLength(r_last);

	// Originally we only used length comparison to divide, now we changed to weights, i.e., path length + turn weight to divide, then update the length
	// Updated length can only start from the beginning and divide the path length
	
	while (rig - lef > eps)
	{
		double mid = (lef + rig) / 2;
		int firstCutLen = std::lower_bound(pathValue.begin() + cuts.at(r_first).start, pathValue.end(), mid + pathValue.at(cuts.at(r_first).start)) - pathValue.begin() - cuts.at(r_first).start + 1;
		cuts.at(r_first).len = firstCutLen;
		cuts.at(r_last).len = old_len_r_first + old_len_r_last - firstCutLen;

		vector<int>::iterator it = adjustCuts.begin();
		while (it != adjustCuts.end())
		{
			if (it != adjustCuts.begin())
				cuts.at(*it).start = (cuts.at(*(it - 1)).start + cuts.at(*(it - 1)).len) % circleLen;

			cuts.at(*it).val = updateCutVal(*it);
			cur_val_max = std::max(cur_val_max, cuts.at(*it).val);
			cur_val_sum += cuts.at(*it).val;

			it++;
		}

		if (cur_val_max < old_val_max || (cur_val_max == old_val_max && cur_val_sum < old_val_sum))
		{ //
			update_success = true;
			old_val_max = cur_val_max;
			old_val_sum = cur_val_sum;
		}

		if (cuts.at(r_first).val < cuts.at(r_last).val)
			lef = mid + 1;
		else if (cuts.at(r_first).val > cuts.at(r_last).val)
			rig = mid - 1;
		else
			break;
	}

	if (!update_success)
	{
		// cout << "Did not find Optimal Cut.\n";
	}
	else
	{
		// cout << "Found an optimal cut. maximum length = " << old_val_max << " total path length = " << old_val_sum << "\n";
	}
}

void PathCut::get2DCoordinateMap(int index, int &x, int &y)
{
	x = index / smallcols;
	y = index % smallcols;
}

// Based on the optimized cuts, generate the final path. Each robot's path uses one-dimensional coordinates, which we'll convert to actual coordinates later
// Actually, the A* generated path doesn't need to be output because we can let ROS's move_base do navigation
Mat PathCut::generatePath()
{
	Mat path_for_each_robot(depot.size(), vector<int>{});
	for (int i = 0; i < cuts.size(); ++i)
	{
		// //cout << "using A star path...\n";
		// //cout << depot[i] << ", " << pathSequence[cuts[i].start] << endl;
		// vector<int> p1 = A_star_path(depot[cut_depot[i]], pathSequence[cuts[i].start]);
		// //cout << pathSequence[(cuts[i].start + cuts[i].len - 1 + circleLen) % circleLen] << ", " << depot[i] << endl;
		// vector<int> p2 = A_star_path(pathSequence[(cuts[i].start + cuts[i].len - 1 + circleLen) % circleLen], depot[cut_depot[i]]);
		// //cout << "A star path ending...\n";
		// for (int j = 0; j < cuts[i].len; ++j)	p1.push_back(pathSequence[(cuts[i].start + j) % circleLen]);

		// p1.insert(p1.end(), p2.begin(), p2.end());
		vector<int> p1;
		for (int j = 0; j < cuts[i].len; ++j)
			p1.push_back(pathSequence[(cuts[i].start + j) % circleLen]);
		path_for_each_robot[i] = p1;
	}

	Mat path_final(depot.size(), vector<int>{});
	for (int i = 0; i < cuts.size(); ++i)
	{
		path_final[cut_depot[i]] = path_for_each_robot[i];

		// If it's required to return to the original point, add the starting position
		if (coverAndReturn)
		{
			path_final[cut_depot[i]].push_back(depot[cut_depot[i]]);
		}
	}

	return path_final;
}

Mat PathCut::cutSolver()
{
	cout << "Entering MSTC Cut Solver...\n";
	MST2Path();

	if (depot.size() == 1)
	{
		cout << "Only one robot, no need to balance, return...\n";
		return Mat(1, pathSequence);
	}

	cout << "Begin MSTC_Star Algorithm to find the best cut for each robot...\n";
	MSTC_Star();

	return generatePath();
}

int PathCut::getTurnsNum()
{
	int turns = 0;
	for (int i = 1; i < pathSequence.size() - 1; ++i)
	{
		turns += isSameLine(pathSequence[i - 1], pathSequence[i], pathSequence[i + 1]) ? 0 : 1;
	}

	return turns;
}

double computePathCost(const std::vector<int> &path, const VehicleParameters &vehicleParams, int mapCols)
{
	// Assuming path is a vector of (cover) grid indices, compute the cost based on the vehicle parameters
	double omega = vehicleParams.omega_rad; // rad/s (angular velocity)
	double a = vehicleParams.acc;			// m/s^2 (acceleration)
	double vmax = vehicleParams.vmax;		// m/s (max velocity)
	double phi = vehicleParams.phi_max_rad; // rad (max banking angle)



	// Computed turn radius
	const double turn_radius = (vmax * vmax) / (gravity * std::tan(phi)); // Turn radius in meters
	std::function<double(double)> turn_time = [&](double theta){return (turn_radius * theta) / vmax ; };			  // Time for a turn based on angle

	// Check if path is valid
	if (path.size() < 2)
		return 0.0;

	std::function<double(int, int)> calculateDistanceMap = [smallcols = mapCols, cellSize = vehicleParams.cellSize_m](int idx1, int idx2)
	{
		int x1, y1, x2, y2;
		x1 = idx1 / smallcols;
		y1 = idx1 % smallcols;
		x2 = idx2 / smallcols;
		y2 = idx2 % smallcols;
		double dx = (x1 - x2) * cellSize; // Adjust with actual cell size
		double dy = (y1 - y2) * cellSize;
		return sqrt(dx * dx + dy * dy);
	};

	double totalCost = 0.0;
	int turnCount = 0;
	double turnCost = 0.0;
	// Compute segment traversal times
	for (size_t j = 0; j < path.size() - 1; ++j)
	{
		double distance = calculateDistanceMap(path[j], path[j + 1]);
		double tj = 0;
#ifdef USE_UAV_COST
		tj = distance / vmax;
#else
		// Segment time tj: sqrt(4d/a) if d < vmax^2/a, else d/vmax + vmax/a
		tj = (distance < (vmax * vmax) / a) ? std::sqrt(4 * distance / a) : (distance / vmax + vmax / a);
#endif

		totalCost += tj;

		// Count turns (check if next segment changes direction)
		if (j < path.size() - 2 && !PathCut::isSameLine(path[j], path[j + 1], path[j + 2]))
		{
			// Vectors: v1 = P_{i-1} to P_i, v2 = P_i to P_{i+1}
			int p0 = path[j], p1 = path[j+1], p2 = path[j + 2];

			double p0x = p0 % mapCols, p0y = p0 / mapCols; // x and y coordinates of p0
			double p1x = p1 % mapCols, p1y = p1 / mapCols; // x and y coordinates of p1
			double p2x = p2 % mapCols, p2y = p2 / mapCols; // x and y coordinates of p2

			double theta = std::abs(segmentAngle(p0x,p0y,p1x,p1y, p2x,p2y)); // Angle between two vectors in radians


			// double v1x = (p1 % mapCols - p0 % mapCols) * vehicleParams.cellSize_m; // x-component
			// double v1y = (p1 / mapCols - p0 / mapCols) * vehicleParams.cellSize_m; // y-component
			// double v2x = (p2 % mapCols - p1 % mapCols) * vehicleParams.cellSize_m;
			// double v2y = (p2 / mapCols - p1 / mapCols) * vehicleParams.cellSize_m;
			
			// // Compute turn angle theta
			// double dot = v1x * v2x + v1y * v2y;
			// double det = v1x * v2y - v1y * v2x;
			// double theta = std::abs(std::atan2(det, dot)); // Absolute angle in [0, pi]



			turnCost += turn_time(theta); // Time for the turn based on angle
			turnCount++;
		}
	}

	
#ifdef USE_UAV_COST
	// Add turn costs (assuming 90-degree turns, cost = turn_time)
	// turnCost = turn_time * turnCount;
#else
	// Add turn costs (assuming 90-degree turns, cost = pi/(2*omega))
	turnCost = (M_PI / (2 * omega)) * turnCount;
#endif

	totalCost += turnCost;

	return totalCost;
}

void PathCut::optimizePathWithOutliersAndUpdateSequence(const std::vector<int> &valid_points)
{
	// Create a working copy of the valid points
	std::vector<int> working_path = valid_points;

	// Create a lookup table for segment costs
	static std::unordered_map<std::tuple<int, int, int>, double, tuple_hash> cost_lookup;
	auto computeSegmentCost = [&](int prev, int curr, int next)
	{
		auto key = std::make_tuple(prev, curr, next);
		if (cost_lookup.find(key) == cost_lookup.end())
		{
			
			// std::function<double(int, int)> calculateDistanceMap = [smallcols = smallcols, cellSize = vehicleParams.cellSize_m](int idx1, int idx2)
			// {
			// 	int x1, y1, x2, y2;
			// 	x1 = idx1 / smallcols;
			// 	y1 = idx1 % smallcols;
			// 	x2 = idx2 / smallcols;
			// 	y2 = idx2 % smallcols;
			// 	double dx = (x1 - x2) * cellSize; // Adjust with actual cell size
			// 	double dy = (y1 - y2) * cellSize;
			// 	return sqrt(dx * dx + dy * dy);
			// };
			
			// cost_lookup[key] = calculateDistanceMap(prev, curr) + calculateDistanceMap(curr, next); // computePathCost(segment, vehicleParams, smallcols);
			
			std::vector<int> segment = {prev, curr, next};
			cost_lookup[key] = computePathCost(segment, vehicleParams, smallcols);
		}
		return cost_lookup[key];
	};

	// Iterate through the working path and adjust points
	for (size_t i = 0; i < working_path.size(); ++i)
	{

		auto j= i; // Wrap around to handle circular paths
		int current_point = working_path[j];
		int prev_index = (j - 1 + working_path.size()) % working_path.size();
		int next_index = (j + 1) % working_path.size();

		int prev_point = working_path[prev_index];
		int next_point = working_path[next_index];

		// Calculate current cost
		double current_cost = computeSegmentCost(prev_point, current_point, next_point);

		// Calculate neighbor costs
		int prev_prev_index = (j - 2 + working_path.size()) % working_path.size();
		int prev_prev_prev_index = (j - 3 + working_path.size()) % working_path.size();
		int next_next_index = (j + 2) % working_path.size();
		int next_next_next_index = (j + 3) % working_path.size();

		int prev_prev_point = working_path[prev_prev_index];
		int prev_prev_prev_point = working_path[prev_prev_prev_index];
		int next_next_point = working_path[next_next_index];
		int next_next_next_point = working_path[next_next_next_index];

		double neighbor_cost_prev = computeSegmentCost(prev_prev_prev_point, prev_prev_point, prev_point);
		double neighbor_cost_next = computeSegmentCost(next_point, next_next_point, next_next_next_point);

		//print current and neighbor costs and ratio
		// std::cout << "Current cost: " << current_cost << ", Neighbor costs: " << neighbor_cost_prev << ", " << neighbor_cost_next << std::endl;
		// std::cout << "Ratio: " << current_cost / neighbor_cost_prev << ", " << current_cost / neighbor_cost_next << std::endl;
		// Logger::info("Current cost: " + std::to_string(current_cost) + ", Neighbor costs: " + std::to_string(neighbor_cost_prev) + ", " + std::to_string(neighbor_cost_next));
		// Logger::info("Ratio: " + std::to_string(current_cost / neighbor_cost_prev) + ", " + std::to_string(current_cost / neighbor_cost_next));
		
		// Check if the current cost is substantially higher than neighbor costs
		size_t iteration_count = 0;
		auto threshold = 15.0; // Adjust this threshold as needed
		while ((current_cost > threshold * neighbor_cost_prev || current_cost > threshold * neighbor_cost_next) && iteration_count < working_path.size() - 2)
		{
			Logger::info("TMSTC - Iteration: " + std::to_string(iteration_count) + ", Current cost: " + std::to_string(current_cost) + ", Neighbor costs: " + std::to_string(neighbor_cost_prev) + ", " + std::to_string(neighbor_cost_next));
			std::cout << "Iteration: " << iteration_count << ", Current cost: " << current_cost << ", Neighbor costs: " << neighbor_cost_prev << ", " << neighbor_cost_next << std::endl;
			// Try moving the current point either past next or before prev
			double cost_move_past_next = computeSegmentCost(next_point, current_point, next_next_point);
			double cost_move_before_prev = computeSegmentCost(prev_prev_point, current_point, prev_point);

			if (cost_move_past_next < cost_move_before_prev && cost_move_past_next < current_cost)
			{
				// Move current point past next
				working_path.erase(working_path.begin() + j);
				working_path.insert(working_path.begin() + next_index, current_point);
				j = next_index; 
				Logger::info("TMSTC - Moved current point past next");
				std::cout << "Moved current point past next" << std::endl;
			}
			else if (cost_move_before_prev < current_cost)
			{
				// Move current point before prev
				working_path.erase(working_path.begin() + j);
				working_path.insert(working_path.begin() + prev_index, current_point);
				j = prev_index;
				Logger::info("TMSTC - Moved current point before prev");
				std::cout << "Moved current point before prev" << std::endl;
			}
			else
			{
				break; // Stop if no improvement is found
			}

			// Update costs after moving
			prev_index = (j - 1 + working_path.size()) % working_path.size();
			next_index = (j + 1) % working_path.size();
			prev_point = working_path[prev_index];
			next_point = working_path[next_index];
			current_cost = computeSegmentCost(prev_point, current_point, next_point);

			prev_prev_index = (j - 2 + working_path.size()) % working_path.size();
			prev_prev_prev_index = (j - 3 + working_path.size()) % working_path.size();
			next_next_index = (j + 2) % working_path.size();
			next_next_next_index = (j + 3) % working_path.size();

			prev_prev_point = working_path[prev_prev_index];
			prev_prev_prev_point = working_path[prev_prev_prev_index];
			next_next_point = working_path[next_next_index];
			next_next_next_point = working_path[next_next_next_index];

			neighbor_cost_prev = computeSegmentCost(prev_prev_prev_point, prev_prev_point, prev_point);
			neighbor_cost_next = computeSegmentCost(next_point, next_next_point, next_next_next_point);

			++iteration_count;
		}
	}

	// Overwrite the original pathSequence with the optimized working path
	pathSequence = working_path;
	circleLen = pathSequence.size();
}