#pragma once

#include "SamplingPlanner.hpp"

#include <vector>

class OnlineRrtStar : public SamplingPlanner {
   private:
	void sampleAndAdd();

   public:
	OnlineRrtStar(std::vector<std::vector<bool>> *obstacleHash, std::vector<std::shared_ptr<Rect>> *obstacleRects, double maxSegment, int width,
	              int height, bool usePseudoRandom, std::shared_ptr<Coord> start, double percentCoverage = 0.02);

	bool isDoneBuilding();
};
