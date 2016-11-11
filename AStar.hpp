#pragma once

#include <unordered_map>

#include "Planner.hpp"
#include "geom/Coord.hpp"

class AStar : public Planner {
   protected:
	// std::unordered_map<Node *, std::vector<Node *>> baseVisibilityGraph;
	void plan();
	void replan(Coord &newEndpoint);
	void buildBaseVisibilityGraph();

   public:
	AStar(std::vector<std::vector<bool>> *obstacleHash, std::vector<Rect *> *obstacleRects, int width, int height, bool usePseudoRandom);
};
