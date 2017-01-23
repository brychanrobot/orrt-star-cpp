#pragma once

#include <numeric>
#include <thread>
#include <mutex>
#include "planning/OnlineRrtStar.hpp"

class Waldo {
   private:
	int width;
	int height;
	int mapArea;
	std::vector<std::vector<bool>> *obstacleHash;
	std::vector<std::shared_ptr<Rect>> *obstacleRects;
	double maxTravel = 1;
	std::thread replanThread;
	int velocityHistorySize = 0;
	std::vector<std::vector<double>> velocityHistory;
	int velocityHistoryIndex = 0;

	// const unsigned int NUM_IMPORTANCE_LEVELS = 3;
	const std::vector<int> IMPORTANCE_PORTIONS = {0, 5, 5};

	std::deque<Coord> tryReplan() {
		std::shared_ptr<Coord> start = nullptr;
		if (this->currentPath.size() > 0) {
			start = std::make_shared<Coord>(this->currentPath[0].x, this->currentPath[0].y);  //&this->currentPath[0];
			                                                                                  // printf("%.2f, %.2f\n", start->x, start->y);
		}
		OnlineRrtStar planner(this->obstacleHash, this->obstacleRects, 12, this->width, this->height, true, start, 0.01);

		for (int i = 0; i < planner.nodeAddThreshold && planner.bestPath.size() == 0; i++) {
			// printf("%d\n", i);
			planner.sample();
		}

		// std::copy(planner.bestPath.begin(), planner.bestPath.end(), std::back_inserter(this->currentPath));
		/*this->currentPath.clear();
		for (auto& point : planner.bestPath) {
		    this->currentPath.push_back(make_shared<Coord>(point.x, point.y));
		}
		*/
		return planner.bestPath;
	}

	void replan() {
		if (this->replanMtx.try_lock()) {
			if (replanThread.joinable()) {
				replanThread.join();
			}
			replanThread = std::thread([this]() {
				std::deque<Coord> path;
				while (path.size() == 0) {
					path = this->tryReplan();
				}

				this->currentPath = path;
				this->replanMtx.unlock();
			});
		}
	}

   public:
	std::mutex replanMtx;
	std::deque<Coord> currentPath;
	unsigned int importance = 0;
	double distanceToUav = 0.0;

	Waldo(std::vector<std::vector<bool>> *obstacleHash, std::vector<std::shared_ptr<Rect>> *obstacleRects, int width, int height,
	      int velocityHistorySize) {
		this->obstacleHash = obstacleHash;
		this->obstacleRects = obstacleRects;
		this->width = width;
		this->height = height;
		this->velocityHistorySize = velocityHistorySize;

		this->velocityHistory = std::vector<std::vector<double>>{std::vector<double>(this->velocityHistorySize, 0.0),
		                                                         std::vector<double>(this->velocityHistorySize, 0.0)};

		auto totalImportance = std::accumulate(IMPORTANCE_PORTIONS.begin(), IMPORTANCE_PORTIONS.end(), 0);
		// printf("%d\n", totalImportance);
		int preImportance = rand() % totalImportance;
		for (unsigned int i = 0; i < IMPORTANCE_PORTIONS.size(); i++) {
			preImportance -= IMPORTANCE_PORTIONS[i];
			if (preImportance < 0) {
				this->importance = i;
				break;
			}
		}

		this->replan();
	}

	void followPath() {
		if (this->currentPath.size() < 2) {
			this->replan();
			return;
		}

		if (euclideanDistance(this->currentPath[0], this->currentPath[1]) <= this->maxTravel) {
			this->velocityHistory[0][velocityHistoryIndex] = this->currentPath[1].x - this->currentPath[0].x;
			this->velocityHistory[1][velocityHistoryIndex] = this->currentPath[1].y - this->currentPath[0].y;

			this->currentPath.pop_front();
		} else {
			auto angle = angleBetweenCoords(this->currentPath[0], this->currentPath[1]);
			auto travel = this->maxTravel * randDouble(0.6, 1.0);
			auto dx = travel * cos(angle);
			auto dy = travel * sin(angle);
			this->currentPath[0].change(this->currentPath[0].x + dx, this->currentPath[0].y + dy);

			this->velocityHistory[0][velocityHistoryIndex] = dx;
			this->velocityHistory[1][velocityHistoryIndex] = dy;
		}

		this->velocityHistoryIndex = (this->velocityHistoryIndex + 1) % this->velocityHistorySize;
	}

	Coord predictFutureFromVelocity(int numTimesteps) {
		auto dx = std::accumulate(this->velocityHistory[0].begin(), this->velocityHistory[0].end(), 0.0) / this->velocityHistorySize;
		auto dy = std::accumulate(this->velocityHistory[1].begin(), this->velocityHistory[1].end(), 0.0) / this->velocityHistorySize;

		return Coord(this->coord().x + dx * numTimesteps, this->coord().y + dy * numTimesteps);
	}

	Coord predictFutureFromRandWalk(int numTimesteps) {
		auto x = this->coord().x;
		auto y = this->coord().y;

		for (int t = 0; t < numTimesteps; t++) {
			auto center = Coord(0, 0);
			auto vectorEnd = Coord(this->velocityHistory[0][this->velocityHistoryIndex], this->velocityHistory[1][this->velocityHistoryIndex]);
			auto heading = angleBetweenCoords(center, vectorEnd);

			auto angle = randDouble(heading - 1, heading + 1);

			auto potX = x + this->maxTravel * cos(angle);
			auto potY = y + this->maxTravel * sin(angle);

			if (!(*this->obstacleHash)[potY][potX]) {
				x = potX;
				y = potY;
			}
		}

		return Coord(x, y);
	}

	Coord coord() { return this->currentPath.front(); }
};