#include "OnlineFmtStar.hpp"
#include <stdio.h>
#include <cstdlib>
#include <ctime>
#include <limits>
#include "geom/Coord.hpp"
#include "geom/utils.hpp"

using namespace std;

OnlineFmtStar::OnlineFmtStar(vector<vector<bool>> *obstacleHash, vector<Rect> *obstacleRects, double maxSegment, int width, int height) {
	srand(time(NULL));  // initialize the random number generator so it happens

	this->width = width;
	this->height = height;
	this->mapArea = width * height;
	this->obstacleHash = obstacleHash;
	this->obstacleRects = obstacleRects;
	this->maxSegment = maxSegment;
	this->rewireNeighborhood = maxSegment * 6;
	this->nodeAddThreshold = 0.015 * width * height;

	this->startPoint = this->randomOpenAreaPoint();

	do {
		this->endPoint = this->randomOpenAreaPoint();
	} while (euclideanDistance(this->startPoint, this->endPoint) < width / 2.0);

	this->root = Node(this->startPoint, NULL, 0.0);
	this->root.status = Status::Open;
	this->rtree.insert(RtreeValue(this->startPoint, &(this->root)));
	this->open.push(&(this->root));

	// printf("root: (%.2f, %.2f), %d\n", this->root.coord.x(), this->root.coord.y(), this->root.status);

	this->endNode = Node(this->endPoint, NULL, 0.0);
	this->rtree.insert(RtreeValue(this->endPoint, &endNode));

	// printf("end: (%.2f, %.2f), %d\n", endNode.coord.x(), endNode.coord.y(), endNode.status);

	for (int n = 0; n < this->nodeAddThreshold; n++) {
		auto point = this->randomOpenAreaPoint();

		auto node = new Node(point, NULL, numeric_limits<double>::max());

		this->rtree.insert(RtreeValue(point, node));
	}
}

void OnlineFmtStar::sample() {
	if (!this->open.empty()) {
		this->sampleAndAdd();
	} else {
		this->sampleWithRewire();
	}
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
			if (bestParent != NULL) {
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
		if (neighbor != bestNeighbor) {
			auto cost = this->getCost(bestNeighbor, neighbor);
			if (bestNeighbor->cumulativeCost + cost < neighbor->cumulativeCost) {
				neighbor->rewire(bestNeighbor, cost);
			}
		}
	}
}

Coord OnlineFmtStar::randomOpenAreaPoint() {
	while (true) {
		auto point = randomPoint(this->width, this->height);
		if (!this->obstacleHash->at((int)point.y()).at((int)point.x())) {
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
		if (neighbor->cumulativeCost < bestCumulativeCost) {
			// bestCost = cost;
			bestCumulativeCost = neighbor->cumulativeCost;
			bestNeighbor = neighbor;
		}
	}
}
