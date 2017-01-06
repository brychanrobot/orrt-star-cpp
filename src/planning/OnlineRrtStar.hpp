#pragma once

#include "SamplingPlanner.hpp"

#include <vector>

class OnlineRrtStar : public SamplingPlanner {
   private:
	long numNodes;

	void sampleAndAdd();

   public:
	OnlineRrtStar(std::vector<std::vector<bool>> *obstacleHash, std::vector<std::shared_ptr<Rect>> *obstacleRects, double maxSegment, int width,
	              int height, bool usePseudoRandom, Coord *start);

	bool isDoneBuilding();
};
