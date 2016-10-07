#include "Planner.hpp"

#include <stdio.h>
#include <cstdlib>
#include <ctime>
#include <limits>
#include <vector>

#include "geom/Coord.hpp"
#include "geom/utils.hpp"

using namespace std;

Planner::Planner(vector<vector<bool>> *obstacleHash, vector<Rect *> *obstacleRects, double maxSegment, int width, int height) {
	srand(time(NULL));  // initialize the random number generator so it happens

	this->width = width;
	this->height = height;
	this->mapArea = width * height;
	this->obstacleHash = obstacleHash;
	this->obstacleRects = obstacleRects;
	this->maxSegment = maxSegment;
	this->rewireNeighborhood = maxSegment * 6;
	this->nodeAddThreshold = 0.015 * width * height;

	auto startPoint = this->randomOpenAreaPoint();
	Coord endPoint;

	do {
		endPoint = this->randomOpenAreaPoint();
	} while (euclideanDistance(startPoint, endPoint) < width / 2.0);

	this->root = new Node(startPoint, NULL, 0.0);
	this->root->status = Status::Open;
	this->rtree.insert(RtreeValue(startPoint, this->root));

	this->endNode = new Node(endPoint, NULL, 0.0);
	this->rtree.insert(RtreeValue(endPoint, endNode));
}

bool Planner::lineIntersectsObstacle(Coord &p1, Coord &p2) {
	auto dx = p2.x() - p1.x();
	auto dy = p2.y() - p1.y();

	auto m = 20000.0;  // a big number for vertical slope

	if (dx != 0) {
		m = dy / dx;
	}

	// printf("m: %f\n", m);

	auto b = -m * p1.x() + p1.y();

	auto minX = std::min(p1.x(), p2.x());
	auto maxX = std::max(p1.x(), p2.x());

	for (int ix = minX; ix <= maxX; ix++) {
		auto y = m * ix + b;
		if ((*this->obstacleHash)[y][ix]) {
			// printf("returning true\n");
			return true;
		}
	}

	auto minY = std::min(p1.y(), p2.y());
	auto maxY = std::max(p1.y(), p2.y());

	for (int iy = minY; iy < maxY; iy++) {
		auto x = (iy - b) / m;
		if ((*this->obstacleHash)[iy][x]) {
			// printf("returning true\n");
			return true;
		}
	}

	return false;
}

void Planner::sampleWithRewire() {
	auto point = this->randomOpenAreaPoint();
	vector<Node *> neighbors;
	Node *bestNeighbor;
	this->findBestNeighbor(point, bestNeighbor, neighbors);
	for (auto neighbor : neighbors) {
		if (neighbor->status == Status::Closed && neighbor != bestNeighbor) {
			auto cost = this->getCost(bestNeighbor, neighbor);
			if (bestNeighbor->cumulativeCost + cost < neighbor->cumulativeCost &&
			    !this->lineIntersectsObstacle(neighbor->coord, bestNeighbor->coord)) {
				neighbor->rewire(bestNeighbor, cost);
			}
		}
	}
}

double clamp(double val, double lo, double hi) { return std::min(std::max(val, lo), hi); }

void Planner::moveStart(double dx, double dy) {
	if (dx != 0 || dy != 0) {
		Coord point(clamp(this->root->coord.x() + dx, 0, this->width - 1), clamp(this->root->coord.y() + dy, 0, this->height - 1));

		if (!this->obstacleHash->at((int)point.y()).at((int)point.x())) {
			auto newRoot = new Node(point, NULL, 0.0);
			newRoot->status = Status::Closed;
			rtree.insert(RtreeValue(newRoot->coord, newRoot));

			auto rtr_cost = this->getCost(newRoot, this->root);

			this->root->rewire(newRoot, rtr_cost);
			this->root = newRoot;

			// maybe this doesn't really add value
			std::vector<RtreeValue> neighbor_tuples;
			this->getNeighbors(newRoot->coord, this->rewireNeighborhood, neighbor_tuples);
			for (auto neighbor_tuple : neighbor_tuples) {
				auto neighbor = neighbor_tuple.second;
				if (neighbor != newRoot && neighbor->status == Status::Closed && !this->lineIntersectsObstacle(newRoot->coord, neighbor->coord)) {
					auto cost = this->getCost(newRoot, neighbor);
					neighbor->rewire(newRoot, cost);
				}
			}
		}
	}
}

void Planner::refreshBestPath() {
	if (this->endNode->parent != NULL) {
		this->bestPath.clear();
		auto currentNode = this->endNode;
		while (currentNode != NULL) {
			this->bestPath.push_front(currentNode->coord);
			currentNode = currentNode->parent;
		}
	}
}

Coord Planner::randomOpenAreaPoint() {
	while (true) {
		auto point = randomPoint(this->width, this->height);
		if (!this->obstacleHash->at((int)point.y()).at((int)point.x())) {
			return point;
		}
	}
}

double Planner::getCost(Node *start, Node *end) { return euclideanDistance(start->coord, end->coord); }

void Planner::getNeighbors(Coord center, double radius, vector<RtreeValue> &results) {
	box query_box(point(center.x() - radius, center.y() - radius), point(center.x() + radius, center.y() + radius));
	this->rtree.query(boost::geometry::index::intersects(query_box), std::back_inserter(results));
}

void Planner::findBestNeighbor(Coord point, Node *&bestNeighbor, vector<Node *> &neighbors) {
	vector<RtreeValue> neighbor_tuples;
	this->getNeighbors(point, this->rewireNeighborhood, neighbor_tuples);

	double bestCumulativeCost = std::numeric_limits<double>::max();
	// double bestCost = std::numeric_limits<double>::max();

	for (auto neighbor_tuple : neighbor_tuples) {
		auto neighbor = neighbor_tuple.second;
		neighbors.push_back(neighbor);
		// auto cost = this->getCost(neighbor, node);
		if (neighbor->status == Status::Closed && neighbor->cumulativeCost < bestCumulativeCost) {
			// bestCost = cost;
			bestCumulativeCost = neighbor->cumulativeCost;
			bestNeighbor = neighbor;
		}
	}
}
