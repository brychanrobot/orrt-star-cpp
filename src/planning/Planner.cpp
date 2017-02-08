#include "Planner.hpp"

#include <stdio.h>
#include <cstdlib>
#include <ctime>
#include <limits>
#include <vector>

#include "../planning-utils/geom/Coord.hpp"
#include "../planning-utils/geom/utils.hpp"
#include "../planning-utils/utils.hpp"

#include "../planning-utils/visibility/loadMap.hpp"
#include "../planning-utils/visibility/visibility.hpp"

using namespace std;

Planner::Planner(vector<vector<bool>> *obstacleHash, vector<shared_ptr<Rect>> *obstacleRects, int width, int height, bool usePseudoRandom)
    : haltonX(19), haltonY(31), rtree() {
	// srand(time(NULL));  // initialize the random number generator so it happens

	this->width = width;
	this->height = height;
	this->mapArea = width * height;
	this->obstacleHash = obstacleHash;
	this->obstacleRects = obstacleRects;

	this->usePseudoRandom = usePseudoRandom;

	this->obstacleArea = 0.0;
	for (auto obstacle : *(this->obstacleRects)) {
		obstacleArea += obstacle->width() * obstacle->height();
	}

	auto startPoint = this->randomOpenAreaPoint();
	Coord endPoint;

	do {
		endPoint = this->randomOpenAreaPoint();
	} while (euclideanDistance(startPoint, endPoint) < width / 2.0);

	this->root = make_shared<RrtNode>(startPoint, shared_ptr<RrtNode>(nullptr), 0.0);
	this->endNode = make_shared<RrtNode>(endPoint, shared_ptr<RrtNode>(nullptr), std::numeric_limits<double>::max() / 2.0);

	loadMap(make_shared<Rect>(Coord(-1, -1), Coord(width + 1, height + 1)), this->obstacleRects, this->segments, this->endpoints);
}

Planner::~Planner() {}

bool Planner::lineIntersectsObstacle(Coord p1, Coord p2) { return lineIntersectsObstacles(p1, p2, this->obstacleHash, this->width, this->height); }

void Planner::moveStart(double dx, double dy) {
	if (dx != 0 || dy != 0) {
		Coord point(clamp(this->root->coord.x + dx, 0, this->width - 1), clamp(this->root->coord.y + dy, 0, this->height - 1));

		if (!this->obstacleHash->at((int)point.y).at((int)point.x)) {
			this->root->coord = point;
		}
	}
}

void Planner::refreshBestPath() {
	if (this->endNode->parent) {
		this->bestPath.clear();
		auto currentNode = static_pointer_cast<Node>(this->endNode);
		while (currentNode) {
			this->bestPath.push_front(currentNode->coord);
			currentNode = currentNode->parent;
		}
	}
}

void Planner::followPath() {
	// auto distanceLeft = this->maxTravel;
	double dx = 0, dy = 0;
	double distanceLeft = this->maxTravel;
	int i = 0;
	while (this->bestPath.size() - i > 1 && distanceLeft > 0.000001) {
		double dist = euclideanDistance(this->bestPath[0].x + dx, this->bestPath[0].y + dy, this->bestPath[i + 1].x, this->bestPath[i + 1].y);
		double travel = min(dist, distanceLeft);
		auto angle = angleBetweenCoords(this->bestPath[i], this->bestPath[i + 1]);
		dx += travel * cos(angle);
		dy += travel * sin(angle);

		distanceLeft -= travel;
		i++;
		// printf("travel: %.6f, left: %.6f\n", travel, distanceLeft);
	}

	// this->bestPath[0] = this->root->coord;
	this->moveStart(dx, dy);
	this->bestPath[0] = this->root->coord;
}

double Planner::calculatePathCost() {
	double cost = 0.0;
	for (unsigned int i = 0; i < this->bestPath.size() - 1; i++) {
		cost += this->getCost(bestPath[i], bestPath[i + 1]);
	}
	return cost;
}

void Planner::replan(Coord &newEndpoint) { this->endNode->coord = newEndpoint; }

void Planner::randomReplan() {
	auto p = this->randomOpenAreaPoint();
	this->replan(p);
}

Coord Planner::randomOpenAreaPoint() {
	while (true) {
		Coord point;
		if (this->usePseudoRandom) {
			point = randomPoint(this->width, this->height);
		} else {
			point.change(this->haltonX.next() * this->width, this->haltonY.next() * this->height);
		}
		if (!this->obstacleHash->at((int)point.y).at((int)point.x)) {
			return point;
		}
	}
}

double Planner::getViewArea(Coord point) {
	auto polygon = calculateVisibilityPolygon(point);
	return getPolygonArea(polygon);
}

double Planner::getUnseenArea(Coord point) {
	auto value = this->unseenAreaMap[point];

	if (value == 0.0) {
		value = (this->mapArea - this->obstacleArea - this->getViewArea(point)) / (this->mapArea - this->obstacleArea);
		this->unseenAreaMap[point] = value;
	}

	return value;
}

double Planner::getEdgeUnseenArea(Coord start, Coord end, double dist) {
	auto a1 = this->getUnseenArea(start);
	auto a2 = this->getUnseenArea(end);

	auto unseenArea = ((a1 + a2) / 2.0) * dist;

	return unseenArea;
}

double Planner::getCost(shared_ptr<RrtNode> start, shared_ptr<RrtNode> end) { return this->getCost(start->coord, end->coord); }
double Planner::getCost(Coord start, Coord end) {
	double d = euclideanDistance(start, end);
	double unseenArea = this->unseenAreaK > 0 ? this->getEdgeUnseenArea(start, end, d) : 0.0;

	return d * this->distanceK + unseenArea * this->unseenAreaK;
}

void Planner::getNeighbors(Coord &center, double radius, vector<RtreeValue> &results) {
	box query_box(point(center.x - radius, center.y - radius), point(center.x + radius, center.y + radius));
	this->rtree.query(boost::geometry::index::intersects(query_box), back_inserter(results));
}

vector<Coord> Planner::calculateVisibilityPolygon(Coord origin) { return calculateVisibility(origin, this->segments, this->endpoints); }
