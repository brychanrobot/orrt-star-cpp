#pragma once

#include "Planner.hpp"

#include <deque>
#include <queue>
#include <vector>

class OnlineFmtStar : public Planner {
   private:
	std::priority_queue<Node *> open;

	void findBestOpenNeighbor(Node *node, Node *&bestNeighbor, double &bestCost);

	void sampleAndAdd();

   public:
	OnlineFmtStar(std::vector<std::vector<bool>> *obstacleHash, std::vector<Rect *> *obstacleRects, double maxSegment, int width, int height);

	void sample();
};
