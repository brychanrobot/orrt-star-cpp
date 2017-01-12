#include "SamplingPlanner.hpp"

#include <stdio.h>
#include <cstdlib>
#include <ctime>
#include <limits>
#include <vector>

#include "../geom/Coord.hpp"
#include "../geom/utils.hpp"

using namespace std;

SamplingPlanner::SamplingPlanner(vector<vector<bool>> *obstacleHash, vector<shared_ptr<Rect>> *obstacleRects, double maxSegment, int width,
                                 int height, bool usePseudoRandom, shared_ptr<Coord> start, double percentCoverage)
    : Planner(obstacleHash, obstacleRects, width, height, usePseudoRandom) {
	this->maxSegment = maxSegment;
	this->rewireNeighborhood = maxSegment * 6;
	this->nodeAddThreshold = percentCoverage * width * height;

	if (start) {
		this->root->coord.change(start->x(), start->y());
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
	vector<shared_ptr<Node>> neighbors;
	shared_ptr<Node> bestNeighbor;
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
		Coord point(clamp(this->root->coord.x() + dx, 0, this->width - 1), clamp(this->root->coord.y() + dy, 0, this->height - 1));

		// printf("clamped point %.2f, %.2f\n", point.x(), point.y());
		if (!this->obstacleHash->at((int)point.y()).at((int)point.x())) {
			auto newRoot = make_shared<Node>(point, nullptr, 0.0);
			newRoot->status = Status::Closed;
			this->rtree.insert(RtreeValue(newRoot->coord.getBoostPoint(), newRoot));

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

void SamplingPlanner::replan(Coord &newEndpoint) {
	this->endNode = this->getNearestNeighbor(newEndpoint);
	this->refreshBestPath();
}

shared_ptr<Node> SamplingPlanner::getNearestNeighbor(Coord &p) {
	vector<RtreeValue> result;
	this->rtree.query(boost::geometry::index::nearest(p.getBoostPoint(), 1), back_inserter(result));
	return result[0].second;
}

void SamplingPlanner::findBestNeighborWithoutCost(Coord point, shared_ptr<Node> &bestNeighbor, vector<shared_ptr<Node>> &neighbors) {
	vector<RtreeValue> neighbor_tuples;
	this->getNeighbors(point, this->rewireNeighborhood, neighbor_tuples);

	double bestCumulativeCost = 9999999999999999;  /// std::numeric_limits<double>::max();
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

void SamplingPlanner::findBestNeighbor(Coord point, shared_ptr<Node> &bestNeighbor, double &bestCost, vector<shared_ptr<Node>> &neighbors,
                                       vector<double> &neighborCosts) {
	vector<RtreeValue> neighbor_tuples;
	this->getNeighbors(point, this->rewireNeighborhood, neighbor_tuples);

	double bestCumulativeCost = 999999999999999;  // std::numeric_limits<double>::max();
	// double bestCost = std::numeric_limits<double>::max();

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
