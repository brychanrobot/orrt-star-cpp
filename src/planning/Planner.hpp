#pragma once

#include <deque>
#include <vector>

#include "Halton.hpp"
#include "Node.hpp"
#include "../geom/Coord.hpp"
#include "../geom/Rect.hpp"

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
	double getCost(Node *start, Node *end);
	double getCost(Coord &start, Coord &end);

	bool lineIntersectsObstacle(Coord &p1, Coord &p2);

	void refreshBestPath();
	virtual void replan(Coord &newEndpoint);

   public:
	Node *root;
	Node *endNode;
	std::vector<Rect *> *obstacleRects;
	std::deque<Coord> bestPath;
	bool usePseudoRandom;
	Planner(std::vector<std::vector<bool>> *obstacleHash, std::vector<Rect *> *obstacleRects, int width, int height, bool usePseudoRandom);
	virtual void moveStart(double dx, double dy);
	void randomReplan();
	void followPath();
};
