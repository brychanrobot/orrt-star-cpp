#include "OnlineRrtStar.hpp"
#include <stdio.h>
#include <cmath>
#include <cstdlib>
#include "../planning-utils/geom/Coord.hpp"
#include "../planning-utils/geom/utils.hpp"

//#include <boost/geometry.hpp>
//#include <boost/geometry/geometries/geometries.hpp>
//#include <boost/geometry/index/rtree.hpp>

using namespace std;

OnlineRrtStar::OnlineRrtStar(vector<vector<bool>> *obstacleHash, vector<shared_ptr<Rect>> *obstacleRects, double maxSegment, int width, int height,
                             bool usePseudoRandom, shared_ptr<Coord> start, double pruneRadius, double percentCoverage)
    : SamplingPlanner(obstacleHash, obstacleRects, maxSegment, width, height, usePseudoRandom, start, pruneRadius, percentCoverage) {
	this->name = "orrtstar";
	this->root->status = Status::Closed;
	this->endNode->status = Status::Closed;

	this->numNodes = 2;
}

bool OnlineRrtStar::isDoneBuilding() { return this->numNodes > this->nodeAddThreshold; }

void OnlineRrtStar::sampleAndAdd() {
	auto p = this->randomOpenAreaPoint();
	auto nn = this->getNearestNeighbor(p);
	auto dist = euclideanDistance(p, nn->coord);

	if (dist > this->maxSegment) {
		auto angle = angleBetweenCoords(nn->coord, p);
		auto x = this->maxSegment * cos(angle) + nn->coord.x;
		auto y = this->maxSegment * sin(angle) + nn->coord.y;

		p.change(x, y);
	}

	if (!(*this->obstacleHash)[(int)p.y][(int)p.x]) {
		vector<shared_ptr<RrtNode>> neighbors;
		vector<double> neighborCosts;
		shared_ptr<RrtNode> bestNeighbor;
		double bestCost;
		this->findBestNeighbor(p, bestNeighbor, bestCost, neighbors, neighborCosts);
		if (bestNeighbor && bestNeighbor->status == Status::Closed && !this->lineIntersectsObstacle(p, bestNeighbor->coord)) {
			auto node = make_shared<RrtNode>(p, nullptr, numeric_limits<double>::max());
			node->status = Status::Closed;
			bestNeighbor->addChild(node, bestCost);
			this->rtree.insert(RtreeValue(p.getBoostPoint(), node));
			this->numNodes++;

			for (unsigned long i = 0; i < neighbors.size(); i++) {
				if (neighbors[i]->status == Status::Closed && neighbors[i] != bestNeighbor) {
					// printf("nc: %.2f\n", neighbors[i]->cumulativeCost);
					// printf("pot: %.2f, curr: %.2f\n", min(node->cumulativeCost + neighborCosts[i], 10000.0),
					//       min(neighbors[i]->cumulativeCost, 10000.0));
					// auto cost = this->getCost(bestNeighbor, neighbors[i]);

					if (node->cumulativeCost + neighborCosts[i] < neighbors[i]->cumulativeCost) {
						if (!this->lineIntersectsObstacle(neighbors[i]->coord, node->coord)) {
							neighbors[i]->rewire(node, neighborCosts[i]);
						}
					}
				}
			}
		}
	}
}
