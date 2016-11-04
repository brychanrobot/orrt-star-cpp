#pragma once

#include "Planner.hpp"

class AStar : public Planner {
   protected:
	void replan(Coord &newEndpoint);
	void buildBaseVisibilityGraph();

   public:
	AStar(std::vector<std::vector<bool>> *obstacleHash, std::vector<Rect *> *obstacleRects, int width, int height, bool usePseudoRandom)
};
