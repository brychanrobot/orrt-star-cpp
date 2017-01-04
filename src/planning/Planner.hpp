#pragma once

#include <deque>
#include <vector>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/index/rtree.hpp>

#include "../geom/Coord.hpp"
#include "../geom/Rect.hpp"
#include "Halton.hpp"
#include "Node.hpp"

typedef boost::geometry::model::box<point> box;
typedef std::pair<point, std::shared_ptr<Node>> RtreeValue;
typedef boost::geometry::index::rtree<RtreeValue, boost::geometry::index::rstar<16>> Rtree;

class Planner {
   protected:
	int width;
	int height;
	int mapArea;
	std::vector<std::vector<bool>> *obstacleHash;
	double maxTravel = 2;

	HaltonSampler haltonX;
	HaltonSampler haltonY;

	Rtree rtree;

	Coord randomOpenAreaPoint();
	double getCost(std::shared_ptr<Node> &start, std::shared_ptr<Node> &end);
	double getCost(Coord &start, Coord &end);

	bool lineIntersectsObstacle(Coord &p1, Coord &p2);

	void refreshBestPath();

	void getNeighbors(Coord center, double radius, std::vector<RtreeValue> &results);

   public:
	std::shared_ptr<Node> root;
	std::shared_ptr<Node> endNode;
	std::vector<std::shared_ptr<Rect>> *obstacleRects;
	std::deque<Coord> bestPath;
	bool usePseudoRandom;
	std::string name = "unset";

	Planner(std::vector<std::vector<bool>> *obstacleHash, std::vector<std::shared_ptr<Rect>> *obstacleRects, int width, int height,
	        bool usePseudoRandom);
	virtual ~Planner();
	virtual void moveStart(double dx, double dy);
	virtual void replan(Coord &newEndpoint);
	void randomReplan();
	void followPath();
	double calculatePathCost();
};
