#include "OnlineFmtStar.hpp"
#include <stdio.h>
#include <cstdlib>
#include "geom/Coord.hpp"
#include "geom/utils.hpp"

using namespace std;

OnlineFmtStar::OnlineFmtStar(vector<vector<bool>> *obstacleHash, vector<Rect *> *obstacleRects, double maxSegment, int width, int height,
                             bool usePseudoRandom)
    : Planner(obstacleHash, obstacleRects, maxSegment, width, height, usePseudoRandom) {
	this->open.push(this->root);

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
