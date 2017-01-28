#pragma once

#include "SamplingPlanner.hpp"

#include <deque>
#include <queue>
#include <vector>

class OnlineFmtStar : public SamplingPlanner {
   private:
	std::priority_queue<std::shared_ptr<RrtNode>> open;

	void findBestOpenNeighbor(std::shared_ptr<RrtNode> &node, std::shared_ptr<RrtNode> &bestNeighbor, double &bestCost);

	void sampleAndAdd();

   public:
	OnlineFmtStar(std::vector<std::vector<bool>> *obstacleHash, std::vector<std::shared_ptr<Rect>> *obstacleRects, double maxSegment, int width,
	              int height, bool usePseudoRandom, std::shared_ptr<Coord> start, double pruneRadius = 5, double percentCoverage = 0.02);

	bool isDoneBuilding();
};
