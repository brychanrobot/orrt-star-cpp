#pragma once

#include "SamplingPlanner.hpp"

#include <vector>

class OnlineRrtStar : public SamplingPlanner {
   private:
	void sampleAndAdd();

   public:
	OnlineRrtStar(std::shared_ptr<std::vector<std::vector<bool>>> obstacleHash, std::shared_ptr<std::vector<std::shared_ptr<Rect>>> obstacleRects, double maxSegment, int width,
	              int height, bool usePseudoRandom, std::shared_ptr<Coord> start, double pruneRadius = 5, double percentCoverage = 0.02);

	bool isDoneBuilding();
};
