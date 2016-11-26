#pragma once

#include <deque>
#include <vector>

#include "../geom/Coord.hpp"
#include "../geom/Rect.hpp"
#include "Halton.hpp"
#include "Node.hpp"

class Planner {
   protected:
	int width;
	int height;
	int mapArea;
	std::vector<std::vector<bool>> *obstacleHash;
	double maxTravel = 2;

	HaltonSampler haltonX;
	HaltonSampler haltonY;

	Coord randomOpenAreaPoint();
	double getCost(std::shared_ptr<Node> &start, std::shared_ptr<Node> &end);
	double getCost(Coord &start, Coord &end);

	bool lineIntersectsObstacle(Coord &p1, Coord &p2);

	void refreshBestPath();
	virtual void replan(Coord &newEndpoint);

   public:
	std::shared_ptr<Node> root;
	std::shared_ptr<Node> endNode;
	std::vector<std::shared_ptr<Rect>> *obstacleRects;
	std::deque<Coord> bestPath;
	bool usePseudoRandom;
	Planner(std::vector<std::vector<bool>> *obstacleHash, std::vector<std::shared_ptr<Rect>> *obstacleRects, int width, int height,
	        bool usePseudoRandom);
	virtual ~Planner();
	virtual void moveStart(double dx, double dy);
	void randomReplan();
	void followPath();
};