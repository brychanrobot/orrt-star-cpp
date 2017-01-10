#pragma once

#include "Planner.hpp"
/*#include <vector>

#include "../geom/Coord.hpp"
#include "../geom/Rect.hpp"*/
#include "Halton.hpp"
#include "Node.hpp"

// template<typename Params>;

class SamplingPlanner : public Planner {
   protected:
	int maxSegment;
	int rewireNeighborhood;
	int nodeAddThreshold;

	std::shared_ptr<Node> getNearestNeighbor(Coord &point);
	void findBestNeighbor(Coord point, std::shared_ptr<Node> &bestNeighbor, double &bestCost, std::vector<std::shared_ptr<Node>> &neighbors,
	                      std::vector<double> &neighborCosts);
	void findBestNeighborWithoutCost(Coord point, std::shared_ptr<Node> &bestNeighbor, std::vector<std::shared_ptr<Node>> &neighbors);

	virtual void sampleAndAdd() = 0;
	void sampleWithRewire();

   public:
	SamplingPlanner(std::vector<std::vector<bool>> *obstacleHash, std::vector<std::shared_ptr<Rect>> *obstacleRects, double maxSegment, int width,
	                int height, bool usePseudoRandom, std::shared_ptr<Coord> start, double percentCoverage);
	// virtual void sample() = 0;
	virtual bool isDoneBuilding() = 0;
	void sample();
	void moveStart(double dx, double dy);
	void replan(Coord &newEndpoint);
};
