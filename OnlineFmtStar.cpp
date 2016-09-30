#include "OnlineFmtStar.hpp"
#include <stdio.h>
#include <cstdlib>
#include <ctime>
#include <limits>
#include "geom/Coord.hpp"
#include "geom/utils.hpp"

using namespace std;

OnlineFmtStar::OnlineFmtStar(vector<vector<bool>> *obstacleHash, vector<Rect *> *obstacleRects, double maxSegment, int width, int height,
                             double obstacleHashWidth, double obstacleHashHeight) {
	srand(time(0));  // initialize the random number generator so it happens

	this->width = width;
	this->height = height;
	this->obstacleHashWidth = obstacleHashWidth;
	this->obstacleHashHeight = obstacleHashHeight;
	this->mapArea = width * height;
	this->obstacleHash = obstacleHash;
	this->obstacleRects = obstacleRects;
	this->maxSegment = maxSegment;
	this->rewireNeighborhood = maxSegment * 6;
	this->nodeAddThreshold = 0.015 * obstacleHashWidth * obstacleHashHeight;

	auto startPoint = this->randomOpenAreaPoint();
	Coord endPoint;

	do {
		endPoint = this->randomOpenAreaPoint();
	} while (euclideanDistance(startPoint, endPoint) < width / 2.0);

	this->root = new Node(startPoint, NULL, 0.0);
	this->root->status = Status::Open;
	this->rtree.insert(RtreeValue(startPoint, this->root));
	this->open.push(this->root);

	// printf("root: (%.2f, %.2f), %d\n", this->root.coord.x(), this->root.coord.y(), this->root.status);

	this->endNode = new Node(endPoint, NULL, 0.0);
	this->rtree.insert(RtreeValue(endPoint, endNode));

	// printf("end: (%.2f, %.2f), %d\n", endNode.coord.x(), endNode.coord.y(), endNode.status);

	for (int n = 0; n < this->nodeAddThreshold; n++) {
		auto point = this->randomOpenAreaPoint();

		auto node = new Node(point, NULL, numeric_limits<double>::max());

		this->rtree.insert(RtreeValue(point, node));
	}
}

double clamp(double val, double lo, double hi) { return std::min(std::max(val, lo), hi); }

bool OnlineFmtStar::pointInObstacle(Coord p) { return (*this->obstacleHash)[p.y() * this->obstacleHashHeight][p.x() * this->obstacleHashWidth]; }

bool OnlineFmtStar::lineIntersectsObstacle(Coord &p1, Coord &p2) {
	auto dx = p2.x() - p1.x();
	auto dy = p2.y() - p1.y();

	// printf("(%.2f, %.2f), (%.2f, %.2f)\n", p1.x(), p1.y(), p2.x(), p2.y());

	auto m = 20000.0;  // a big number for vertical slope

	if (abs(dx) > 0.00001) {
		m = dy / dx;
	}

	// printf("m: %f\n", m);

	auto b = -m * p1.x() * this->obstacleHashWidth + p1.y() * this->obstacleHashHeight;

	// printf("m: %.2f, b: %.2f\n", m, b);

	auto minX = std::min(p1.x(), p2.x()) * obstacleHashWidth;
	auto maxX = std::max(p1.x(), p2.x()) * obstacleHashWidth;

	// printf("x: %.2f, %.2f\n", minX, maxX);

	for (double ix = minX; ix <= maxX; ix++) {
		double y = m * ix + b;
		y = clamp(y, 0, this->obstacleHashHeight - 1);
		// printf("%.2f, %.2f\n", ix, y);
		if ((*this->obstacleHash)[ix][y]) {
			// printf("returning true\n");
			return true;
		}
	}

	auto minY = std::min(p1.y(), p2.y()) * this->obstacleHashHeight;
	auto maxY = std::max(p1.y(), p2.y()) * this->obstacleHashHeight;

	// printf("doingy\n");
	for (double iy = minY; iy < maxY; iy++) {
		auto x = (iy - b) / m;
		x = clamp(x, 0, this->obstacleHashWidth - 1);
		if ((*this->obstacleHash)[x][iy]) {
			// printf("returning true\n");
			return true;
		}
	}

	// printf("returning false\n");
	return false;
}

void OnlineFmtStar::sample() {
	if (!this->open.empty()) {
		this->sampleAndAdd();
	} else {
		this->sampleWithRewire();
	}

	this->refreshBestPath();
}

void OnlineFmtStar::sampleAndAdd() {
	auto bestOpenNode = this->open.top();
	this->open.pop();

	vector<RtreeValue> neighbors;
	this->getNeighbors(bestOpenNode->coord, this->rewireNeighborhood, neighbors);

	for (auto neighbor_tuple : neighbors) {
		auto neighbor = neighbor_tuple.second;
		if (neighbor->status == Status::Unvisited) {
			Node *bestParent = NULL;
			double bestCost;
			findBestOpenNeighbor(neighbor, bestParent, bestCost);
			if (bestParent != NULL && !this->lineIntersectsObstacle(neighbor->coord, bestParent->coord)) {
				bestParent->addChild(neighbor, bestCost);
				neighbor->status = Status::Open;
				this->open.push(neighbor);
			}
		}
	}

	// bestOpenNode->printChildren();

	bestOpenNode->status = Status::Closed;
}

void OnlineFmtStar::sampleWithRewire() {
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

void OnlineFmtStar::moveStart(double dx, double dy) {
	if (dx != 0 || dy != 0) {
		Coord point(clamp(this->root->coord.x() + dx, 0, this->width), clamp(this->root->coord.y() + dy, 0, this->height));

		if (!this->pointInObstacle(point)) {
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

void OnlineFmtStar::refreshBestPath() {
	if (this->endNode->parent != NULL) {
		this->bestPath.clear();
		auto currentNode = this->endNode;
		while (currentNode != NULL) {
			this->bestPath.push_front(currentNode->coord);
			currentNode = currentNode->parent;
		}
	}
}

Coord OnlineFmtStar::randomOpenAreaPoint() {
	while (true) {
		auto point = randomPoint(this->width, this->height);
		if (!this->pointInObstacle(point)) {
			return point;
		}
	}
}

double OnlineFmtStar::getCost(Node *start, Node *end) { return euclideanDistance(start->coord, end->coord); }

void OnlineFmtStar::getNeighbors(Coord center, double radius, vector<RtreeValue> &results) {
	box query_box(point(center.x() - radius, center.y() - radius), point(center.x() + radius, center.y() + radius));
	this->rtree.query(boost::geometry::index::intersects(query_box), std::back_inserter(results));
}

void OnlineFmtStar::findBestOpenNeighbor(Node *node, Node *&bestNeighbor, double &bestCost) {
	vector<RtreeValue> neighbor_tuples;
	this->getNeighbors(node->coord, this->rewireNeighborhood, neighbor_tuples);

	double bestCumulativeCost = std::numeric_limits<double>::max();
	for (auto neighbor_tuple : neighbor_tuples) {
		auto neighbor = neighbor_tuple.second;
		auto cost = this->getCost(neighbor, node);
		if (neighbor->status == Status::Open && cost + neighbor->cumulativeCost < bestCumulativeCost) {
			bestCost = cost;
			bestCumulativeCost = cost + neighbor->cumulativeCost;
			bestNeighbor = neighbor;
		}
	}
}

void OnlineFmtStar::findBestNeighbor(Coord point, Node *&bestNeighbor, vector<Node *> &neighbors) {
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
