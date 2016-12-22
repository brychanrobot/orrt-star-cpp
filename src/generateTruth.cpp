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

vector<double> linspace(double a, double b, int n) {
	vector<double> array;
	double step = (b - a) / (n - 1);

	while (a <= b) {
		array.push_back(a);
		a += step;  // could recode to better handle rounding errors
	}
	return array;
}

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
	bool usePseudoRandom = true;
	// double replanFrequency = -1;

	// clang-format off
	cxxopts::Options options("OnlineRRT*", "A cool program for cool things");
	options.add_options()
		("f,fullscreen", "Enable Fullscreen", cxxopts::value(isFullscreen))
		("m,monitor", "Set Monitor Number", cxxopts::value(monitorNum))
		("fmt", "Use FMT*", cxxopts::value(useFmt))
		("p,pr", "Use pseudo-random numbers", cxxopts::value(usePseudoRandom));
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

	saveMap("data/map0.json", width, height, obstacleRects);
	json allPaths;

	for (auto& preReplanFrequency : linspace(0.1, 5, 30)) {
		auto replanFrequency = pow(preReplanFrequency, 5);
		auto moveInterval = 1.0 / 30.0;               // seconds
		auto replanInterval = 1.0 / replanFrequency;  // seconds
		/*auto iterations = 0;
		auto frames = 0;
		double averageIterations = 0;*/
		printf("%.6f, %.4f\n", replanFrequency, replanInterval);
		double lastMove = 0.0;
		double lastReplan = 0.0;

		json paths;
		paths["frequency"] = replanFrequency;

		for (double currentTime = 0.0; currentTime < 120; currentTime += 0.0000001) {
			if (currentTime - lastMove >= moveInterval) {
				lastMove = currentTime;
				planner->followPath();
				paths["paths"]["move"].push_back(jsonifyPath(planner->bestPath, planner->calculatePathCost()));

			} else if (replanFrequency != -1 && currentTime - lastReplan >= replanInterval) {
				lastReplan = currentTime;
				planner->randomReplan();
				paths["paths"]["replan"].push_back(jsonifyPath(planner->bestPath, planner->calculatePathCost()));

			} else {
				// iterations++;
				// planner->sample();
			}
		}
		allPaths.push_back(paths);
	}

	saveJson("data/map0paths.json", allPaths);
}
