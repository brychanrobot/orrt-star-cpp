#include "Planner.hpp"

#include <stdio.h>
#include <cstdlib>
#include <ctime>
#include <limits>
#include <vector>

#include "../geom/Coord.hpp"
#include "../geom/utils.hpp"

using namespace std;

Planner::Planner(vector<vector<bool>> *obstacleHash, vector<shared_ptr<Rect>> *obstacleRects, int width, int height, bool usePseudoRandom)
    : haltonX(19), haltonY(23) {
	srand(time(NULL));  // initialize the random number generator so it happens

	this->width = width;
	this->height = height;
	this->mapArea = width * height;
	this->obstacleHash = obstacleHash;
	this->obstacleRects = obstacleRects;

	this->usePseudoRandom = usePseudoRandom;

	auto startPoint = this->randomOpenAreaPoint();
	Coord endPoint;

	do {
		endPoint = this->randomOpenAreaPoint();
	} while (euclideanDistance(startPoint, endPoint) < width / 2.0);

	this->root = make_shared<Node>(startPoint, shared_ptr<Node>(nullptr), 0.0);

	this->endNode = make_shared<Node>(endPoint, shared_ptr<Node>(nullptr), std::numeric_limits<double>::max() / 2.0);
}

Planner::~Planner() {
	/*printf("starting planner desturctor\n");

	// delete this->obstacleHash;
	for (auto obstacle : *(this->obstacleRects)) {
	    delete obstacle;
	}
	// delete this->obstacleRects;
	printf("deleted obstacle stuff\n");
	*/
	// delete this->obstacleHash;
	// delete this->root;
	// delete this->endNode;

	// printf("destructed planner\n");
}

bool Planner::lineIntersectsObstacle(Coord &p1, Coord &p2) {
	auto dx = p2.x() - p1.x();
	auto dy = p2.y() - p1.y();

	if (p1.x() < 0 || p1.y() < 0 || p2.x() < 0 || p2.y() < 0) {
		return true;
	}

	/*
	auto m = 20000.0;  // a big number for vertical slope

	if (abs(dx) > 0.0001) {
	    m = dy / dx;
	}
	*/
	auto m = clamp(dy / dx, -20000, 20000);

	// printf("m: %f\n", m);

	auto b = -m * p1.x() + p1.y();

	if (abs(m) != 20000) {
		auto minX = std::min(p1.x(), p2.x());
		auto maxX = std::max(p1.x(), p2.x());

		for (int ix = minX; ix <= maxX; ix++) {
			auto y = m * ix + b;
			// printf("[%.2f, %.2f]:[%.2f, %.2f] %.3f, (%d, %f)\n", p1.x(), p1.y(), p2.x(), p2.y(), m, ix, y);
			// printf("h: [%.2f, %.2f]:[%.2f, %.2f] --------dx: %f m: %f, b: %.2f, (%d, %.2f)\n", p1.x(), p1.y(), p2.x(), p2.y(), dx, m, b, ix, y);
			// printf("%.2d, %.2d\n", (int)y, ix);

			// printf("%.2f, %.2f : %d\n", p1.x(), p2.x(), ix);
			if (y > 0 && y < this->height && (*this->obstacleHash)[(int)y][ix]) {
				// printf("returning true\n");
				return true;
			}
		}
	}

	if (abs(m) != 0) {
		auto minY = std::min(p1.y(), p2.y());
		auto maxY = std::max(p1.y(), p2.y());

		for (int iy = minY; iy < maxY; iy++) {
			auto x = (iy - b) / m;
			// printf("v: [%.2f, %.2f]:[%.2f, %.2f] --------dx: %f m: %f, b: %.2f, (%.2f, %d)\n", p1.x(), p1.y(), p2.x(), p2.y(), dx, m, b, x, iy);
			if (x > 0 && x < this->width && (*this->obstacleHash)[iy][(int)x]) {
				// printf("returning true\n");
				return true;
			}
		}
	}

	return false;
}

void Planner::moveStart(double dx, double dy) {
	if (dx != 0 || dy != 0) {
		Coord point(clamp(this->root->coord.x() + dx, 0, this->width - 1), clamp(this->root->coord.y() + dy, 0, this->height - 1));

		if (!this->obstacleHash->at((int)point.y()).at((int)point.x())) {
			this->root->coord = point;
		}
	}
}

void Planner::refreshBestPath() {
	if (this->endNode->parent) {
		this->bestPath.clear();
		auto currentNode = this->endNode;
		while (currentNode) {
			this->bestPath.push_front(currentNode->coord);
			currentNode = currentNode->parent;
		}
	}
}

void Planner::followPath() {
	if (this->bestPath.size() > 1) {
		auto currentWaypoint = this->bestPath[1];
		auto angle = angleBetweenCoords(this->root->coord, currentWaypoint);
		auto dx = this->maxTravel * cos(angle);
		auto dy = this->maxTravel * sin(angle);

		this->moveStart(dx, dy);
	}
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
			point = Coord(this->haltonX.next() * this->width, this->haltonY.next() * this->height);
		}
		if (!this->obstacleHash->at((int)point.y()).at((int)point.x())) {
			return point;
		}
	}
}

double Planner::getCost(shared_ptr<Node> &start, shared_ptr<Node> &end) { return this->getCost(start->coord, end->coord); }
double Planner::getCost(Coord &start, Coord &end) { return euclideanDistance(start, end); }
