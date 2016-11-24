#pragma once

#include "Planner.hpp"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/index/rtree.hpp>
/*#include <vector>

#include "Halton.hpp"
#include "Node.hpp"
#include "geom/Coord.hpp"
#include "geom/Rect.hpp"*/

// template<typename Params>;
typedef boost::geometry::model::box<point> box;
typedef std::pair<point, Node *> RtreeValue;
typedef boost::geometry::index::rtree<RtreeValue, boost::geometry::index::rstar<16>> Rtree;

class SamplingPlanner : public Planner {
   protected:
	int maxSegment;
	int rewireNeighborhood;
	int nodeAddThreshold;
	Rtree rtree;

	Node *getNearestNeighbor(Coord &point);
	void getNeighbors(Coord center, double radius, std::vector<RtreeValue> &results);
	void findBestNeighbor(Coord point, Node *&bestNeighbor, double &bestCost, std::vector<Node *> &neighbors, std::vector<double> &neighborCosts);
	void findBestNeighborWithoutCost(Coord point, Node *&bestNeighbor, std::vector<Node *> &neighbors);

	void replan(Coord &newEndpoint);
	void sampleWithRewire();

   public:
	SamplingPlanner(std::vector<std::vector<bool>> *obstacleHash, std::vector<Rect *> *obstacleRects, double maxSegment, int width, int height,
	                bool usePseudoRandom);
	virtual void sample() = 0;
	void moveStart(double dx, double dy);
};
