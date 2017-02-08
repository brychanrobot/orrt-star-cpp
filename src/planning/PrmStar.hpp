#pragma once

#include "AStar.hpp"

enum GraphType { Random, Grid };

class PrmStar : public AStar {
   protected:
	void buildBaseVisibilityGraph();

	void sampleRandom(std::vector<std::shared_ptr<RrtNode>> &allNodes);
	void sampleGrid(std::vector<std::shared_ptr<RrtNode>> &allNodes);

	GraphType graphType;

   public:
	PrmStar(std::shared_ptr<std::vector<std::vector<bool>>> obstacleHash, std::shared_ptr<std::vector<std::shared_ptr<Rect>>> obstacleRects, int width, int height,
	        bool usePseudoRandom, GraphType graphType);
	~PrmStar();

	void moveStart(double dx, double dy);
	void replan(Coord &newEndpoint);
};
