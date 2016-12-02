#pragma once

#include "AStar.hpp"

class PrmStar : public AStar {
   protected:
	void replan(Coord &newEndpoint);
	void buildBaseVisibilityGraph();

   public:
	PrmStar(std::vector<std::vector<bool>> *obstacleHash, std::vector<std::shared_ptr<Rect>> *obstacleRects, int width, int height,
	        bool usePseudoRandom);
	~PrmStar();
};
