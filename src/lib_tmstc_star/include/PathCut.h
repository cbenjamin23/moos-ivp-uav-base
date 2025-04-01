#ifndef _PATH_CUT_H
#define _PATH_CUT_H

#include <vector>
#include <queue>
#include <iostream>
#include <unordered_set>
#include <algorithm>
#include <cmath>

using std::cout;
using std::endl;
using std::ifstream;
using std::pair;
using std::unordered_set;
using std::vector;

const double eps = 1e-7;

typedef vector<vector<int>> Mat;
typedef pair<int, int> P;
typedef struct Cut
{
	int start; // Note: This is the index on the circle
	int len;
	double val; // Use a small epsilon value to prevent floating-point precision errors when comparing
} cut;

typedef struct Node
{
	double fx;
	double gx;
	int id;
	bool operator<(const Node &n) const
	{
		return fx - n.fx > eps;
		// return fx < n.fx;
	}
} node;

#define reshape(i, j) (int)((i) * bigcols + (j))

#ifndef PI
#define PI 3.1415926
#endif

static double ONE_TURN_VAL = 2.0;

struct VehicleParameters
{
	double omega;	 // rad/s (angular velocity)
	double a;		 // m/s^2 (acceleration)
	double vmax;	 // m/s (max velocity)
	double cellSize; // meters (grid cell size)
};

class PathCut
{
private:
	int bigrows, bigcols;
	int smallrows, smallcols;
	int circleLen;
	Mat MST, Map, Region;
	Mat pathEdge;
	vector<int> depot;
	vector<int> pathSequence, invSequence;
	vector<cut> cuts;
	vector<double> pathValue;
	int dir[4][2] = {{0, 1}, {0, -1}, {1, 0}, {-1, 0}};

	vector<int> depot_cut; // from depot to cuts
	vector<int> cut_depot;

	bool coverAndReturn;

	VehicleParameters vehicleParams;
	int maxIterations = 10000;

public:
	PathCut(Mat &map, Mat &region, Mat &tree, vector<int> &robotInitPos, VehicleParameters vp, int maxIter, bool _coverAndReturn = false) : Map(map), Region(region), MST(tree), depot(robotInitPos), vehicleParams(vp), maxIterations(maxIter), coverAndReturn(_coverAndReturn)
	{
		bigrows = Region.size();
		bigcols = Region[0].size();
		smallrows = Map.size();
		smallcols = Map[0].size();
		circleLen = 0;
		cuts.resize(depot.size(), {});
		depot_cut.resize(depot.size(), 0); // The index of the i-th robot on the circle
		cut_depot.resize(depot.size(), 0); // The original index of the robot corresponding to the i-th cut on the circle
	}

	void MST2Path();
	void get2DCoordinateMap(int index, int &x, int &y);
	
	void MSTC_Star();
	void Balanced_Cut(vector<int> &adjustCuts);
	double updateCutVal(int i);
	double A_star(int u, int v);
	vector<int> A_star_path(int u, int v);
	vector<int> getHalfCuts(int cut_min, int cut_max, int dir);
	Mat generatePath();

	double euclidean_dis(double x1, double y1, double x2, double y2);
	double calculateDistance(int idx1, int idx2);

	Mat cutSolver();
	int getTurnsNum();
	double getTurnAndLength(int i);

	friend double computePathCost(std::vector<int> &path);

	void setOneTurnVal(double val)
	{
		ONE_TURN_VAL = val;
	}

	static bool isSameLine(int a, int b, int c)
	{
		return a + c == 2 * b;
	}
	
	
};

double computePathCost(const std::vector<int> &path, const VehicleParameters &vehicleParams, int mapCols);

#endif