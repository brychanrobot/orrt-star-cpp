#include "SamplingPlanner.hpp"

#include <stdio.h>
#include <cstdlib>
#include <ctime>
#include <limits>
#include <vector>

#include "../planning-utils/geom/Coord.hpp"
#include "../planning-utils/geom/utils.hpp"

using namespace std;

SamplingPlanner::SamplingPlanner(shared_ptr<vector<vector<bool>>> obstacleHash, shared_ptr<vector<shared_ptr<Rect>>> obstacleRects, double maxSegment, int width,
                                 int height, bool usePseudoRandom, shared_ptr<Coord> start, double pruneRadius, double percentCoverage)
    : Planner(obstacleHash, obstacleRects, width, height, usePseudoRandom) {
	this->maxSegment = maxSegment;
	this->rewireNeighborhood = maxSegment * 6;
	this->nodeAddThreshold = percentCoverage * width * height;
	this->pruneRadius = pruneRadius;

	if (start) {
		this->root->coord.change(start->x, start->y);
	}

	this->root->status = Status::Open;
	this->rtree.insert(RtreeValue(this->root->coord.getBoostPoint(), this->root));

	this->rtree.insert(RtreeValue(this->endNode->coord.getBoostPoint(), this->endNode));
}

void SamplingPlanner::sample() {
	if (!this->isDoneBuilding()) {
		this->sampleAndAdd();
	} else {
		this->sampleWithRewire();
	}

	this->refreshBestPath();
}

void SamplingPlanner::sampleWithRewire() {
	auto point = this->randomOpenAreaPoint();
	vector<shared_ptr<RrtNode>> neighbors;
	shared_ptr<RrtNode> bestNeighbor;
	this->findBestNeighborWithoutCost(point, bestNeighbor, neighbors);
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

void SamplingPlanner::moveStart(double dx, double dy) {
	if (dx != 0 || dy != 0) {
		Coord point(clamp(this->root->coord.x + dx, 0, this->width - 1), clamp(this->root->coord.y + dy, 0, this->height - 1));

		if (!this->obstacleHash->at((int)point.y).at((int)point.x) || true) {
			auto newRoot = make_shared<RrtNode>(point, nullptr, 0.0);
			newRoot->status = Status::Closed;
			this->rtree.insert(RtreeValue(newRoot->coord.getBoostPoint(), newRoot));
			this->numNodes++;

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

			for (auto neighbor_tuple : neighbor_tuples) {
				auto neighbor = neighbor_tuple.second;
				if (neighbor != newRoot && neighbor->status == Status::Closed && !this->lineIntersectsObstacle(newRoot->coord, neighbor->coord)) {
					if (neighbor->children.size() == 0 && euclideanDistance(newRoot->coord, neighbor->coord) < this->pruneRadius) {
						neighbor->parent->removeChild(neighbor);
						this->rtree.remove(RtreeValue(neighbor->coord.getBoostPoint(), neighbor));
						this->numNodes--;
						if (this->numNodes <= this->nodeAddThreshold) {  //++numRemoved > 0) {
							return;
						}
					}
				}
			}
		}
	}
}

void SamplingPlanner::replan(Coord &newEndpoint) {
	this->endNode = this->getNearestNeighbor(newEndpoint);
	this->refreshBestPath();
}

shared_ptr<RrtNode> SamplingPlanner::getNearestNeighbor(Coord &p) {
	vector<RtreeValue> result;
	this->rtree.query(boost::geometry::index::nearest(p.getBoostPoint(), 1), back_inserter(result));
	return result[0].second;
}

shared_ptr<RrtNode> SamplingPlanner::getNearestNeighbor(shared_ptr<Node> n) {
	vector<RtreeValue> result;
	this->rtree.query(boost::geometry::index::nearest(n->coord.getBoostPoint(), 2), back_inserter(result));

	return result[0].second != n ? result[0].second : result[1].second;
}

void SamplingPlanner::findBestNeighborWithoutCost(Coord point, shared_ptr<RrtNode> &bestNeighbor, vector<shared_ptr<RrtNode>> &neighbors) {
	vector<RtreeValue> neighbor_tuples;
	this->getNeighbors(point, this->rewireNeighborhood, neighbor_tuples);

	double bestCumulativeCost = 9999999999999999;

	for (auto neighbor_tuple : neighbor_tuples) {
		auto neighbor = neighbor_tuple.second;
		neighbors.push_back(neighbor);
		if (neighbor->status == Status::Closed && neighbor->cumulativeCost < bestCumulativeCost) {
			bestCumulativeCost = neighbor->cumulativeCost;
			bestNeighbor = neighbor;
		}
	}
}

void SamplingPlanner::findBestNeighbor(Coord point, shared_ptr<RrtNode> &bestNeighbor, double &bestCost, vector<shared_ptr<RrtNode>> &neighbors,
                                       vector<double> &neighborCosts) {
	vector<RtreeValue> neighbor_tuples;
	this->getNeighbors(point, this->rewireNeighborhood, neighbor_tuples);

	double bestCumulativeCost = 999999999999999;

	for (auto neighbor_tuple : neighbor_tuples) {
		auto neighbor = neighbor_tuple.second;
		neighbors.push_back(neighbor);
		auto cost = this->getCost(neighbor->coord, point);
		neighborCosts.push_back(cost);
		if (neighbor->status == Status::Closed && (neighbor->cumulativeCost + cost < bestCumulativeCost)) {
			bestCost = cost;
			bestCumulativeCost = neighbor->cumulativeCost + cost;
			bestNeighbor = neighbor;
		}
	}
}

double SamplingPlanner::calculateTotalEntropy() { return (1.0 / this->numNodes) * calculateParticleEntropy(this->root) + log(2) + exp(1.0); }

double SamplingPlanner::calculateParticleEntropy(shared_ptr<Node> r) {
	auto n = this->getNearestNeighbor(r);
	auto dist = euclideanDistance(r->coord, n->coord);
	double entropy = dist != 0.0 ? log(this->numNodes * dist) : 1.0;

	for (auto child : r->children) {
		entropy += calculateParticleEntropy(child);
	}

	return entropy;
}
