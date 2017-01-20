
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
const double VIEW_RADIUS = 100;

void drawWaldos(vector<unique_ptr<Waldo>>& waldos) {
	for (const auto& waldo : waldos) {
		if (waldo->replanMtx.try_lock()) {
			// drawPath(waldo->currentPath, HSL(200, 1.0, 0.3), HSL(200, 1.0, 0.5));
			if (waldo->importance > 0 && waldo->distanceToUav < VIEW_RADIUS) {
				drawHollowCircle(waldo->coord(), 7, HSL(0, 1.0, 1.0));
			}

			/*
			if (waldo->importance > 0) {
			    auto future = waldo->predictFutureFromVelocity(60);
			    drawPoint(future, 4, HSL(25, 1.0, 0.5));
			    drawLine(waldo->coord(), future, 1, HSL(5, 1.0, 0.5));
			}
			*/

			drawSolidCircle(waldo->coord(), 7, HSL(100 + 100 * waldo->importance, 1.0, 0.3));
			waldo->replanMtx.unlock();
		}
	}
}

void display(const shared_ptr<RrtNode> root, const shared_ptr<RrtNode>& endNode, deque<Coord>& bestPath, vector<shared_ptr<Rect>>* obstacleRects,
             vector<unique_ptr<Waldo>>& waldos, bool shouldDrawTree) {
	// drawPoint(root->coord, 10, HSL(25, 1.0, 0.5));
	drawSolidCircle(root->coord, VIEW_RADIUS, HSL(25, 1.0, 0.5), 0.2);
	drawHollowCircle(root->coord, VIEW_RADIUS, HSL(25, 1.0, 0.5));

	drawPoint(endNode->coord, 4, HSL(50, 1.0, 0.5));

	drawPath(bestPath, HSL(100, 1.0, 0.3), HSL(150, 1.0, 0.5));

	drawWaldos(waldos);

	drawObstacles(obstacleRects, OBSTACLE_PADDING, HSL(0, 0.01, 0.4));
	if (shouldDrawTree) {
		drawTree(root, HSL(325, 1.0, 0.2));
	}
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
	int waldoHistorySize = 20;
	bool shouldDrawTree = false;
	int voteCellSize = 2;

	// clang-format off
	cxxopts::Options options("OnlineRRT*", "A cool program for cool things");
	options.add_options()
		("f,fullscreen", "Enable Fullscreen", cxxopts::value(isFullscreen))
		("m,monitor", "Set Monitor Number", cxxopts::value(monitorNum))
		("fmt", "Use FMT*", cxxopts::value(useFmt))
		("p,pr", "Use pseudo-random numbers", cxxopts::value(usePseudoRandom))
		("r,replan", "Replan frequency", cxxopts::value(replanFrequency))
		("w,waldos", "number of Waldos", cxxopts::value(numWaldos))
		("t,tree", "draw tree", cxxopts::value(shouldDrawTree));
	// clang-format on

	options.parse(argc, argv);

	auto window = initWindow(isFullscreen, monitorNum, width, height);

	vector<shared_ptr<Rect>> obstacleRects;
	generateObstacleRects(width, height, 10, obstacleRects, OBSTACLE_PADDING);

	vector<vector<bool>> obstacleHash(height, vector<bool>(width, false));
	generateObstacleHash(obstacleRects, obstacleHash);

	vector<unique_ptr<Waldo>> waldos;

	for (int w = 0; w < numWaldos; w++) {
		waldos.push_back(make_unique<Waldo>(&obstacleHash, &obstacleRects, width, height, waldoHistorySize));
	}

	SamplingPlanner* planner;
	if (useFmt) {
		planner = new OnlineFmtStar(&obstacleHash, &obstacleRects, 6, width, height, usePseudoRandom, nullptr);
	} else {
		planner = new OnlineRrtStar(&obstacleHash, &obstacleRects, 6, width, height, usePseudoRandom, nullptr);
	}
	// AStar* planner = new AStar(&obstacleHash, &obstacleRects, width, height, usePseudoRandom);
	// PrmStar* planner = new PrmStar(&obstacleHash, &obstacleRects, width, height, usePseudoRandom, GraphType::Grid);

	auto displayCallback = [&planner, &waldos, shouldDrawTree]() {
		display(planner->root, planner->endNode, planner->bestPath, planner->obstacleRects, waldos, shouldDrawTree);
	};

	auto lastReplan = glfwGetTime();
	auto lastMove = glfwGetTime();
	auto replanInterval = 1.0 / replanFrequency;
	auto moveInterval = 1.0 / 30.0;

	auto remainderCallback = [&obstacleHash, &planner, &waldos, &replanInterval, &moveInterval, &lastReplan, &lastMove, width, height,
	                          voteCellSize]() {
		auto currentTime = glfwGetTime();
		if (currentTime - lastMove >= moveInterval) {
			lastMove = currentTime;

			vector<vector<int>> waldoVotes(height / voteCellSize, vector<int>(width / voteCellSize, 0));

			for (const auto& waldo : waldos) {
				waldo->followPath();

				waldo->distanceToUav = euclideanDistance(planner->root->coord, waldo->coord());

				if (waldo->importance > 0 && waldo->distanceToUav < VIEW_RADIUS) {
					if (!lineIntersectsObstacles(waldo->coord(), planner->root->coord, &obstacleHash, width, height)) {
						vector<Coord> predictedCoords{
						    // waldo->predictFutureFromRandWalk(60),
						    waldo->predictFutureFromVelocity(60), waldo->coord(),
						};
						for (const auto& coord : predictedCoords) {
							for (int dx = -VIEW_RADIUS; dx < VIEW_RADIUS; dx += voteCellSize) {
								for (int dy = -VIEW_RADIUS; dy < VIEW_RADIUS; dy += voteCellSize) {
									int x = coord.x + dx;
									int y = coord.y + dy;
									double dist = sqrt(dx * dx + dy * dy);
									if (x > 0 && x < width && y > 0 && y < height) {
										waldoVotes[y / voteCellSize][x / voteCellSize] +=
										    std::max(0.0, (10 + waldo->importance) * (VIEW_RADIUS - dist));
									}
								}
							}
						}
					}
				}
			}

			/*auto loc = max_element(waldoVotes.begin(), waldoVotes.end()) - waldoVotes.begin();
			printf("%lu\n", loc);
			auto end = Coord((loc % width) * 10, (loc / height) * 10);
			*/
			auto bestScore = 0;
			auto bestEnd = Coord(0, 0);
			for (int y = 0; y < height / voteCellSize; y++) {
				for (int x = 0; x < width / voteCellSize; x++) {
					if (waldoVotes[y][x] > bestScore) {
						bestEnd.x = x * voteCellSize;
						bestEnd.y = y * voteCellSize;

						bestScore = waldoVotes[y][x];
					}
					// printf("%d ", waldoVotes[y][x]);
				}
				// printf("\n");
			}

			planner->replan(bestEnd);

			planner->followPath();
		} else if (replanInterval != -1 && currentTime - lastReplan >= replanInterval) {
			lastReplan = currentTime;
			planner->randomReplan();
		} else {
			// printf("sampling\n");
			planner->sample();
		}
	};

	displayLoop(window, 30.0, displayCallback, remainderCallback);
}
