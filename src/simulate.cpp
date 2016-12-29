#include <stdio.h>
#include <stdlib.h>
#include <chrono>
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

void readMap(string filename, int& width, int& height, vector<shared_ptr<Rect>>& obstacleRects) {
	json m;
	ifstream f(filename);
	f >> m;
	f.close();
	height = m["height"];
	width = m["width"];

	for (auto& r : m["obstacles"]) {
		auto rect = make_shared<Rect>(r[0][0], r[0][1], r[1][0], r[1][1]);
		obstacleRects.push_back(rect);
		// printf("[[%.2f, %.2f], [%.2f, %.2f]]\n", rect->topLeft.x(), rect->topLeft.y(), rect->bottomRight.x(), rect->bottomRight.y());
	}
}

json readAllPaths(string filename) {
	json p;
	ifstream f(filename);
	f >> p;
	f.close();

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
	// generateObstacleRects(width, height, 10, obstacleRects, OBSTACLE_PADDING);
	readMap("data/map0.json", width, height, obstacleRects);

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

	auto allPaths = readAllPaths("data/map0paths.json");

	printf("%.2f\n", (double)allPaths[28]["frequency"]);

	for (auto& paths : allPaths) {
		auto replanFrequency = (double)paths["frequency"];  // pow(preReplanFrequency, 5);
		auto moveInterval = 1.0 / 30.0;                     // seconds
		auto replanInterval = 1.0 / replanFrequency;        // seconds
		printf("%.6f, %.4f\n", replanFrequency, replanInterval);
		double lastMove = 0.0;
		double lastReplan = 0.0;

		int moveIdx = 0;
		int replanIdx = 0;

		json currentPath;

		for (double currentTime = 0.0; currentTime < 120; currentTime += 0.0000001) {
			if (currentTime - lastMove >= moveInterval) {
				lastMove = currentTime;
				// planner->followPath();
				currentPath = paths["paths"]["move"][moveIdx]["path"];
				moveIdx++;

				// printf("moving start\n");
				// printf("%.2f\n", (double)currentPath[0][0]);
				planner->moveStart((double)currentPath[0][0] - planner->root->coord.x(), (double)currentPath[0][1] - planner->root->coord.y());
				// printf("start moved\n");

			} else if (replanFrequency != -1 && currentTime - lastReplan >= replanInterval) {
				lastReplan = currentTime;

				currentPath = paths["paths"]["replan"][replanIdx]["path"];
				replanIdx++;

				auto p = Coord((double)currentPath.back()[0], (double)currentPath.back()[1]);

				auto begin = chrono::high_resolution_clock::now();

				planner->replan(p);  // code to benchmark

				auto end = chrono::high_resolution_clock::now();
				chrono::duration_cast<chrono::seconds>(end - begin).count();

				planner->replan(p);
			} else {
				// iterations++;
				// planner->sample();
			}
		}
	}
}
