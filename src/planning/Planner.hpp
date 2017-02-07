#pragma once

#include <deque>
#include <unordered_map>
#include <vector>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/index/rtree.hpp>

#include "../planning-utils/geom/Coord.hpp"
#include "../planning-utils/geom/Rect.hpp"
#include "../planning-utils/visibility/EndPoint.hpp"
#include "../planning-utils/visibility/Segment.hpp"
#include "Halton.hpp"
#include "RrtNode.hpp"

typedef boost::geometry::model::box<point> box;
typedef std::pair<point, std::shared_ptr<RrtNode>> RtreeValue;
typedef boost::geometry::index::rtree<RtreeValue, boost::geometry::index::rstar<16>> Rtree;

class Planner {
   protected:
	int width;
	int height;
	double mapArea;
	double obstacleArea;
	std::vector<std::vector<bool>> *obstacleHash;
	double maxTravel = 1.5;
	std::unordered_map<Coord, double> unseenAreaMap;

	HaltonSampler haltonX;
	HaltonSampler haltonY;

	Rtree rtree;

	std::vector<std::shared_ptr<Segment>> segments;
	std::vector<std::shared_ptr<EndPoint>> endpoints;

	Coord randomOpenAreaPoint();
	double getViewArea(Coord point);
	double getUnseenArea(Coord point);
	double getEdgeUnseenArea(Coord start, Coord end, double dist);
	double getCost(std::shared_ptr<RrtNode> start, std::shared_ptr<RrtNode> end);
	double getCost(Coord start, Coord end);

	bool lineIntersectsObstacle(Coord p1, Coord p2);

	void refreshBestPath();

	void getNeighbors(Coord &center, double radius, std::vector<RtreeValue> &results);

   public:
	std::shared_ptr<RrtNode> root;
	std::shared_ptr<RrtNode> endNode;
	std::vector<std::shared_ptr<Rect>> *obstacleRects;
	std::deque<Coord> bestPath;
	bool usePseudoRandom;
	std::string name = "unset";

	std::vector<Coord> badPolygon;
	Coord badCenter;

	const double distanceK = 1.0;
	double unseenAreaK = 100.0;

	Planner(std::vector<std::vector<bool>> *obstacleHash, std::vector<std::shared_ptr<Rect>> *obstacleRects, int width, int height,
	        bool usePseudoRandom);
	virtual ~Planner();
	virtual void moveStart(double dx, double dy);
	virtual void replan(Coord &newEndpoint);
	void randomReplan();
	void followPath();
	double calculatePathCost();

	std::deque<Coord> getBestPath() { return this->bestPath; }

	std::vector<Coord> calculateVisibilityPolygon(Coord origin);
};
