#pragma once

#include <unordered_map>

#include "Planner.hpp"

class AStar : public Planner {
   protected:
	std::unordered_map<Coord *, std::vector<Coord *>> baseVisibilityGraph;

	void replan(Coord &newEndpoint);
	void buildBaseVisibilityGraph();

   public:
	AStar(std::vector<std::vector<bool>> *obstacleHash, std::vector<Rect *> *obstacleRects, int width, int height, bool usePseudoRandom)
};
