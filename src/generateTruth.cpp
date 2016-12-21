#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <set>
#include "cxxopts.hpp"
#include "json.hpp"
#include "planning/AStar.hpp"
#include "planning/OnlineFmtStar.hpp"
#include "planning/OnlineRrtStar.hpp"
#include "planning/PrmStar.hpp"
#include "planning/utils.hpp"

using namespace std;
using json = nlohmann::json;

const double MOVERATE = 3;
const double OBSTACLE_PADDING = 5;

void saveJson(string filename, json& j) {
	ofstream o(filename);
	o << j << endl;
	o.close();
}

void saveMap(string filename, int width, int height, vector<shared_ptr<Rect>>& obstacleRects) {
	json m;
	m["height"] = height;
	m["width"] = width;

	for (auto& rect : obstacleRects) {
		json r = {{rect->topLeft.x(), rect->topLeft.y()}, {rect->bottomRight.x(), rect->bottomRight.y()}};
		m["obstacles"].push_back(r);
	}

	cout << m << endl;
	saveJson(filename, m);
}

json jsonifyPath(deque<Coord>& path, double cost) {
	json p;

	for (auto& point : path) {
		p["path"].push_back({point.x(), point.y()});
	}

	p["cost"] = cost;

	return p;
}

int main(int argc, char* argv[]) {
	int width = 700;
	int height = 700;
	bool isFullscreen = false;
	int monitorNum = 0;
	bool useFmt = false;
	bool usePseudoRandom = false;
	double replanFrequency = -1;

	// clang-format off
	cxxopts::Options options("OnlineRRT*", "A cool program for cool things");
	options.add_options()
		("f,fullscreen", "Enable Fullscreen", cxxopts::value(isFullscreen))
		("m,monitor", "Set Monitor Number", cxxopts::value(monitorNum))
		("fmt", "Use FMT*", cxxopts::value(useFmt))
		("p,pr", "Use pseudo-random numbers", cxxopts::value(usePseudoRandom))
		("r,replan", "Replan frequency", cxxopts::value(replanFrequency));
	// clang-format on

	options.parse(argc, argv);

	vector<shared_ptr<Rect>> obstacleRects;
	generateObstacleRects(width, height, 10, obstacleRects, OBSTACLE_PADDING);

	vector<vector<bool>> obstacleHash(height, vector<bool>(width, false));
	generateObstacleHash(obstacleRects, obstacleHash);

	/*SamplingPlanner* planner;
	if (useFmt) {
	    planner = new OnlineFmtStar(&obstacleHash, &obstacleRects, 6, width, height, usePseudoRandom);
	} else {
	    planner = new OnlineRrtStar(&obstacleHash, &obstacleRects, 6, width, height, usePseudoRandom);
	}*/
	AStar* planner = new AStar(&obstacleHash, &obstacleRects, width, height, usePseudoRandom);
	// PrmStar* planner = new PrmStar(&obstacleHash, &obstacleRects, width, height, usePseudoRandom, GraphType::Grid);

	auto moveInterval = 1.0 / 30.0;               // seconds
	auto replanInterval = 1.0 / replanFrequency;  // seconds
	/*auto iterations = 0;
	auto frames = 0;
	double averageIterations = 0;*/
	printf("%.2f\n", replanInterval);
	double currentTime = 0.0;
	double lastMove = 0.0;
	double lastReplan = 0.0;

	saveMap("data/map0.json", width, height, obstacleRects);
	cout << jsonifyPath(planner->bestPath, planner->endNode->cumulativeCost) << endl;
	json movePaths;
	json replanPaths;

	while (currentTime < 120.0) {
		currentTime += 0.0000001;
		if (currentTime - lastMove >= moveInterval) {
			lastMove = currentTime;
			planner->followPath();
			movePaths.push_back(jsonifyPath(planner->bestPath, planner->calculatePathCost()));

		} else if (replanFrequency != -1 && currentTime - lastReplan >= replanInterval) {
			lastReplan = currentTime;
			planner->randomReplan();
			replanPaths.push_back(jsonifyPath(planner->bestPath, planner->calculatePathCost()));

		} else {
			// iterations++;
			// planner->sample();
		}
	}

	saveJson("data/map0move.json", movePaths);
	saveJson("data/map0replan.json", replanPaths);
}
