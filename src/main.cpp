
#include <stdio.h>
#include <cstdlib>
#include <set>
#include "planning-utils/libs/cxxopts.hpp"
#include "planning/AStar.hpp"
#include "planning/OnlineFmtStar.hpp"
#include "planning/OnlineRrtStar.hpp"
#include "planning/PrmStar.hpp"
#include "planning-utils/display.hpp"
#include "planning-utils/utils.hpp"
#include "Waldo.hpp"

using namespace std;

const double OBSTACLE_PADDING = 5;

void drawWaldos(vector<unique_ptr<Waldo>>& waldos) {
	for (const auto& waldo : waldos) {
		if (waldo->replanMtx.try_lock()) {
			// drawPath(waldo->currentPath, HSL(200, 1.0, 0.3), HSL(200, 1.0, 0.5));
			drawPoint(waldo->coord(), 15, HSL(200, 1.0, 0.5));
			waldo->replanMtx.unlock();
		}
	}
}

void display(const shared_ptr<RrtNode> root, const shared_ptr<RrtNode>& endNode, deque<Coord>& bestPath, vector<shared_ptr<Rect>>* obstacleRects,
             vector<unique_ptr<Waldo>>& waldos) {
	drawObstacles(obstacleRects, OBSTACLE_PADDING, HSL(275, 1.0, 0.5));
	// drawTree(root);
	// drawGraph(root, visibilityGraph);
	drawWaldos(waldos);

	drawPath(bestPath, HSL(100, 1.0, 0.3), HSL(150, 1.0, 0.5));

	// printf("%.2f, %.2f\n", root->coord.x, root->coord.y);
	drawPoint(root->coord, 20, HSL(25, 1.0, 0.5));
	drawPoint(endNode->coord, 20, HSL(50, 1.0, 0.5));
}

int main(int argc, char* argv[]) {
	srand(time(0));

	int width = 700;
	int height = 700;
	bool isFullscreen = false;
	int monitorNum = 0;
	bool useFmt = false;
	bool usePseudoRandom = false;
	double replanFrequency = -1;
	int numWaldos = 0;

	// clang-format off
	cxxopts::Options options("OnlineRRT*", "A cool program for cool things");
	options.add_options()
		("f,fullscreen", "Enable Fullscreen", cxxopts::value(isFullscreen))
		("m,monitor", "Set Monitor Number", cxxopts::value(monitorNum))
		("fmt", "Use FMT*", cxxopts::value(useFmt))
		("p,pr", "Use pseudo-random numbers", cxxopts::value(usePseudoRandom))
		("r,replan", "Replan frequency", cxxopts::value(replanFrequency))
		("w,waldos", "number of Waldos", cxxopts::value(numWaldos));
	// clang-format on

	options.parse(argc, argv);

	auto window = initWindow(isFullscreen, monitorNum, width, height);

	vector<shared_ptr<Rect>> obstacleRects;
	generateObstacleRects(width, height, 10, obstacleRects, OBSTACLE_PADDING);

	vector<vector<bool>> obstacleHash(height, vector<bool>(width, false));
	generateObstacleHash(obstacleRects, obstacleHash);

	vector<unique_ptr<Waldo>> waldos;

	SamplingPlanner* planner;
	if (useFmt) {
		planner = new OnlineFmtStar(&obstacleHash, &obstacleRects, 6, width, height, usePseudoRandom, nullptr);
	} else {
		planner = new OnlineRrtStar(&obstacleHash, &obstacleRects, 6, width, height, usePseudoRandom, nullptr);
	}
	// AStar* planner = new AStar(&obstacleHash, &obstacleRects, width, height, usePseudoRandom);
	// PrmStar* planner = new PrmStar(&obstacleHash, &obstacleRects, width, height, usePseudoRandom, GraphType::Grid);

	for (int w = 0; w < numWaldos; w++) {
		waldos.push_back(make_unique<Waldo>(&obstacleHash, &obstacleRects, width, height));
	}

	auto displayCallback = [&planner, &waldos]() { display(planner->root, planner->endNode, planner->bestPath, planner->obstacleRects, waldos); };

	auto lastReplan = glfwGetTime();
	auto lastMove = glfwGetTime();
	auto replanInterval = 1.0 / replanFrequency;
	auto moveInterval = 1.0 / 30.0;

	auto remainderCallback = [&planner, &waldos, &replanInterval, &moveInterval, &lastReplan, &lastMove]() {
		auto currentTime = glfwGetTime();
		if (currentTime - lastMove >= moveInterval) {
			lastMove = currentTime;

			for (const auto& waldo : waldos) {
				waldo->followPath();
			}

			planner->followPath();
		} else if (replanInterval != -1 && currentTime - lastReplan >= replanInterval) {
			lastReplan = currentTime;
			planner->randomReplan();
		} else {
			planner->sample();
		}
	};

	displayLoop(window, 30.0, displayCallback, remainderCallback);
}
