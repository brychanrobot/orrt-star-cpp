#pragma once

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
	double maxTravel = 2;
	std::thread replanThread;

	unsigned int importance;

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

	Waldo(std::vector<std::vector<bool>> *obstacleHash, std::vector<std::shared_ptr<Rect>> *obstacleRects, int width, int height) {
		this->obstacleHash = obstacleHash;
		this->obstacleRects = obstacleRects;
		this->width = width;
		this->height = height;

		this->replan();
	}

	void followPath() {
		if (this->currentPath.size() < 2) {
			this->replan();
			return;
		}

		if (euclideanDistance(this->currentPath[0], this->currentPath[1]) <= this->maxTravel) {
			this->currentPath.pop_front();
		} else {
			auto angle = angleBetweenCoords(this->currentPath[0], this->currentPath[1]);
			auto dx = this->maxTravel * cos(angle);
			auto dy = this->maxTravel * sin(angle);
			this->currentPath[0].change(this->currentPath[0].x + dx, this->currentPath[0].y + dy);
		}
	}

	Coord coord() { return this->currentPath.front(); }
};