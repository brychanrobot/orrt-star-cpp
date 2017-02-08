#pragma once

#include "Planner.hpp"
/*#include <vector>

#include "../planning-utils/geom/Coord.hpp"
#include "../planning-utils/geom/Rect.hpp"*/
#include "Halton.hpp"
#include "RrtNode.hpp"

// template<typename Params>;

class SamplingPlanner : public Planner {
   protected:
	int maxSegment;
	int rewireNeighborhood;
	double pruneRadius;

	std::shared_ptr<RrtNode> getNearestNeighbor(Coord &point);
	std::shared_ptr<RrtNode> getNearestNeighbor(std::shared_ptr<Node> n);
	void findBestNeighbor(Coord point, std::shared_ptr<RrtNode> &bestNeighbor, double &bestCost, std::vector<std::shared_ptr<RrtNode>> &neighbors,
	                      std::vector<double> &neighborCosts);
	void findBestNeighborWithoutCost(Coord point, std::shared_ptr<RrtNode> &bestNeighbor, std::vector<std::shared_ptr<RrtNode>> &neighbors);
	double calculateParticleEntropy(std::shared_ptr<Node> r);

	virtual void sampleAndAdd() = 0;
	void sampleWithRewire();

   public:
	int nodeAddThreshold;
	long numNodes = 0;

	SamplingPlanner(std::shared_ptr<std::vector<std::vector<bool>>> obstacleHash, std::shared_ptr<std::vector<std::shared_ptr<Rect>>> obstacleRects, double maxSegment, int width,
	                int height, bool usePseudoRandom, std::shared_ptr<Coord> start, double pruneRadius, double percentCoverage);
	// virtual void sample() = 0;
	virtual bool isDoneBuilding() = 0;
	void sample();
	void moveStart(double dx, double dy);
	void replan(Coord &newEndpoint);
	double calculateTotalEntropy();
};
