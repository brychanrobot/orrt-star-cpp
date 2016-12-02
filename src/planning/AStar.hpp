#pragma once

#include <set>
#include <unordered_map>

#include "../geom/Coord.hpp"
#include "Planner.hpp"

class AStar : public Planner {
   protected:
	std::unordered_map<std::shared_ptr<Node>, std::set<std::shared_ptr<Node>>> baseVisibilityGraph;
	void plan();
	virtual void replan(Coord &newEndpoint);
	virtual void buildBaseVisibilityGraph();

   public:
	AStar(std::vector<std::vector<bool>> *obstacleHash, std::vector<std::shared_ptr<Rect>> *obstacleRects, int width, int height,
	      bool usePseudoRandom, bool initialize = true);
	~AStar();
	void moveStart(double dx, double dy);
	void randomStart();
};
