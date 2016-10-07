#include "OnlineRrtStar.hpp"
#include <stdio.h>
#include <cmath>
#include <cstdlib>
#include "geom/Coord.hpp"
#include "geom/utils.hpp"

//#include <boost/geometry.hpp>
//#include <boost/geometry/geometries/geometries.hpp>
//#include <boost/geometry/index/rtree.hpp>

using namespace std;

OnlineRrtStar::OnlineRrtStar(vector<vector<bool>> *obstacleHash, vector<Rect *> *obstacleRects, double maxSegment, int width, int height)
    : Planner(obstacleHash, obstacleRects, maxSegment, width, height) {
	this->root->status = Status::Closed;
	this->endNode->status = Status::Closed;

	this->numNodes = 2;
}

void OnlineRrtStar::sample() {
	if (this->numNodes < this->nodeAddThreshold) {
		this->sampleAndAdd();
	} else {
		this->sampleWithRewire();
	}

	this->refreshBestPath();
}

void OnlineRrtStar::sampleAndAdd() {
	auto p = this->randomOpenAreaPoint();
	vector<RtreeValue> result;
	this->rtree.query(boost::geometry::index::nearest((point)p, 1), back_inserter(result));
	auto nn = result[0].second;
	auto dist = euclideanDistance(p, nn->coord);

	if (dist > this->maxSegment) {
		auto angle = angleBetweenCoords(nn->coord, p);
		auto x = this->maxSegment * cos(angle) + nn->coord.x();
		auto y = this->maxSegment * sin(angle) + nn->coord.y();

		printf("nn: (%.2f, %.2f), p: (%.2f, %.2f), np: (%.2f, %.2f), a:%.2f\n", nn->coord.x(), nn->coord.y(), p.x(), p.y(), x, y, 6.282 * angle);
		p = Coord(x, y);
	}

	if (!this->obstacleHash->at((int)p.y()).at((int)p.x())) {
		vector<Node *> neighbors;
		vector<double> neighborCosts;
		Node *bestNeighbor = NULL;
		double bestCost;
		this->findBestNeighbor(p, bestNeighbor, bestCost, neighbors, neighborCosts);
		if (bestNeighbor != NULL && bestNeighbor->status == Status::Closed) {
			Node *node = new Node(p, NULL, 0.0);
			node->status = Status::Closed;
			bestNeighbor->addChild(node, bestCost);
			rtree.insert(RtreeValue(p, node));
			this->numNodes++;

			for (unsigned long i = 0; i < neighbors.size(); i++) {
				if (neighbors[i]->status == Status::Closed && neighbors[i] != bestNeighbor) {
					// auto cost = this->getCost(bestNeighbor, neighbors[i]);
					if (bestNeighbor->cumulativeCost + neighborCosts[i] < neighbors[i]->cumulativeCost &&
					    !this->lineIntersectsObstacle(neighbors[i]->coord, p)) {
						neighbors[i]->rewire(node, neighborCosts[i]);
					}
				}
			}
		}
	}
}
