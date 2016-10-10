#pragma once

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <vector>

#include "Halton.hpp"
#include "Node.hpp"
#include "geom/Coord.hpp"
#include "geom/Rect.hpp"

// template<typename Params>;
typedef boost::geometry::model::box<point> box;
typedef std::pair<point, Node *> RtreeValue;
typedef boost::geometry::index::rtree<RtreeValue, boost::geometry::index::rstar<16>> Rtree;

class Planner {
   protected:
	int width;
	int height;
	int mapArea;
	std::vector<std::vector<bool>> *obstacleHash;
	int maxSegment;
	int rewireNeighborhood;
	int nodeAddThreshold;
	Rtree rtree;

	HaltonSampler haltonX;
	HaltonSampler haltonY;

	Coord randomOpenAreaPoint();
	double getCost(Node *start, Node *end);
	double getCost(Coord &start, Coord &end);
	void getNeighbors(Coord center, double radius, std::vector<RtreeValue> &results);
	void findBestNeighbor(Coord point, Node *&bestNeighbor, double &bestCost, std::vector<Node *> &neighbors, std::vector<double> &neighborCosts);
	void findBestNeighborInNeighborhood(Coord point, Node *&bestNeighbor, std::vector<Node *> &neighbors);
	bool lineIntersectsObstacle(Coord &p1, Coord &p2);

	void sampleWithRewire();
	void refreshBestPath();

   public:
	Node *root;
	Node *endNode;
	std::vector<Rect *> *obstacleRects;
	std::deque<Coord> bestPath;
	Planner(std::vector<std::vector<bool>> *obstacleHash, std::vector<Rect *> *obstacleRects, double maxSegment, int width, int height);
	virtual void sample() = 0;
	void moveStart(double dx, double dy);
};
