#pragma once

#include "AStar.hpp"

enum GraphType { Random, Grid };

class PrmStar : public AStar {
   protected:
	void buildBaseVisibilityGraph();
	void replan(Coord &newEndpoint);

	void sampleRandom(std::vector<std::shared_ptr<Node>> &allNodes);
	void sampleGrid(std::vector<std::shared_ptr<Node>> &allNodes);

	GraphType graphType;

   public:
	PrmStar(std::vector<std::vector<bool>> *obstacleHash, std::vector<std::shared_ptr<Rect>> *obstacleRects, int width, int height,
	        bool usePseudoRandom, GraphType graphType);
	~PrmStar();

	void moveStart(double dx, double dy);
};
