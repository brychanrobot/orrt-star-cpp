#pragma once

#include <set>
#include <unordered_map>

#include "../planning-utils/geom/Coord.hpp"
#include "Planner.hpp"

class AStar : public Planner {
   protected:
	std::unordered_map<std::shared_ptr<RrtNode>, std::set<std::shared_ptr<RrtNode>>> baseVisibilityGraph;
	void plan();
	virtual void buildBaseVisibilityGraph();
	int dx;
	int dy;

   public:
	AStar(std::vector<std::vector<bool>> *obstacleHash, std::vector<std::shared_ptr<Rect>> *obstacleRects, int width, int height,
	      bool usePseudoRandom, bool initialize = true);
	~AStar();
	virtual void moveStart(double dx, double dy);
	void randomStart();
	virtual void replan(Coord &newEndpoint);
};
