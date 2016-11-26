#pragma once

#include "SamplingPlanner.hpp"

#include <deque>
#include <queue>
#include <vector>

class OnlineFmtStar : public SamplingPlanner {
   private:
	std::priority_queue<std::shared_ptr<Node>> open;

	void findBestOpenNeighbor(std::shared_ptr<Node> &node, std::shared_ptr<Node> &bestNeighbor, double &bestCost);

	void sampleAndAdd();

   public:
	OnlineFmtStar(std::vector<std::vector<bool>> *obstacleHash, std::vector<std::shared_ptr<Rect>> *obstacleRects, double maxSegment, int width,
	              int height, bool usePseudoRandom);

	void sample();
};
