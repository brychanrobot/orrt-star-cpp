#pragma once

#include <set>
#include <unordered_map>

#include "../geom/Coord.hpp"
#include "Planner.hpp"

class AStar : public Planner {
   protected:
	std::unordered_map<Node *, std::set<Node *>> baseVisibilityGraph;
	void plan();
	void replan(Coord &newEndpoint);
	void buildBaseVisibilityGraph();

   public:
	AStar(std::vector<std::vector<bool>> *obstacleHash, std::vector<Rect *> *obstacleRects, int width, int height, bool usePseudoRandom);
	void moveStart(double dx, double dy);
	void randomStart();
};
