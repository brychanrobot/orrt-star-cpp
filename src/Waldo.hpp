#pragma once

#include "planning/OnlineRrtStar.hpp"

class Waldo {
   private:
	int width;
	int height;
	int mapArea;
	std::vector<std::vector<bool>> *obstacleHash;
	std::vector<std::shared_ptr<Rect>> *obstacleRects;
	double maxTravel = 2;

	unsigned int importance;

	void replan() {
		std::shared_ptr<Coord> start = nullptr;
		if (this->currentPath.size() > 0) {
			start = std::make_shared<Coord>(this->currentPath[0].x(), this->currentPath[0].y());  //&this->currentPath[0];
		}
		OnlineRrtStar planner(this->obstacleHash, this->obstacleRects, 12, this->width, this->height, true, start, 0.01);
		while (planner.bestPath.size() == 0) {
			planner.sample();
		}

		// std::copy(planner.bestPath.begin(), planner.bestPath.end(), std::back_inserter(this->currentPath));
		/*this->currentPath.clear();
		for (auto& point : planner.bestPath) {
		    this->currentPath.push_back(make_shared<Coord>(point.x(), point.y()));
		}
		*/
		this->currentPath = planner.bestPath;
	}

   public:
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
		}

		if (euclideanDistance(this->currentPath[0], this->currentPath[1]) <= this->maxTravel) {
			this->currentPath.pop_front();
		} else {
			auto angle = angleBetweenCoords(this->currentPath[0], this->currentPath[1]);
			auto dx = this->maxTravel * cos(angle);
			auto dy = this->maxTravel * sin(angle);
			this->currentPath[0].change(this->currentPath[0].x() + dx, this->currentPath[0].y() + dy);
		}
	}

	Coord coord() { return this->currentPath.front(); }
};