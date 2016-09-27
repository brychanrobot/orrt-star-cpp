#pragma once

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <deque>
#include <queue>
#include <vector>
#include "Node.hpp"
#include "geom/Coord.hpp"
#include "geom/Rect.hpp"

// template<typename Params>;
typedef boost::geometry::model::box<point> box;
typedef std::pair<point, Node *> RtreeValue;
typedef boost::geometry::index::rtree<RtreeValue, boost::geometry::index::rstar<16>> Rtree;

class OnlineFmtStar {
   private:
	int width;
	int height;
	int mapArea;
	std::vector<std::vector<bool>> *obstacleHash;
	int maxSegment;
	int rewireNeighborhood;
	int nodeAddThreshold;
	Rtree rtree;
	std::priority_queue<Node *> open;

	Coord randomOpenAreaPoint();
	double getCost(Node *start, Node *end);
	void getNeighbors(Coord center, double radius, std::vector<RtreeValue> &results);
	void findBestOpenNeighbor(Node *node, Node *&bestNeighbor, double &bestCost);
	void findBestNeighbor(Coord point, Node *&bestNeighbor, std::vector<Node *> &neighbors);
	bool lineIntersectsObstacle(Coord &p1, Coord &p2);

	void sampleAndAdd();
	void sampleWithRewire();
	void refreshBestPath();

   public:
	Node *root;
	Node *endNode;
	std::vector<Rect *> *obstacleRects;
	std::deque<Coord> bestPath;
	OnlineFmtStar(std::vector<std::vector<bool>> *obstacleHash, std::vector<Rect *> *obstacleRects, double maxSegment, int width, int height);

	void sample();
	void moveStart(double dx, double dy);
};
