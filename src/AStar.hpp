#pragma once

#include <set>
#include <unordered_map>

#include "Planner.hpp"
#include "geom/Coord.hpp"

class AStar : public Planner {
   protected:
	// void plan();
	void replan(Coord &newEndpoint);
	void buildBaseVisibilityGraph();

   public:
	AStar(std::vector<std::vector<bool>> *obstacleHash, std::vector<Rect *> *obstacleRects, int width, int height, bool usePseudoRandom);

	void plan();
	std::vector<Node *> q;

	std::unordered_map<Node *, std::set<Node *>> baseVisibilityGraph;

	void moveStart(double dx, double dy);
};
