#include "PrmStar.hpp"

#include "../planning-utils/geom/utils.hpp"

using namespace std;

PrmStar::PrmStar(vector<vector<bool>> *obstacleHash, vector<shared_ptr<Rect>> *obstacleRects, int width, int height, bool usePseudoRandom,
                 GraphType graphType)
    : AStar(obstacleHash, obstacleRects, width, height, usePseudoRandom, false) {
	if (graphType == GraphType::Random) {
		this->name = "prmstar";
	} else {
		this->name = "astar_grid";
	}
	this->graphType = graphType;
	this->buildBaseVisibilityGraph();
	this->plan();  // make an initial plan
}

PrmStar::~PrmStar() {}

void PrmStar::sampleRandom(vector<shared_ptr<RrtNode>> &allNodes) {
	for (auto j = 0; j < 0.02 * this->width * this->height; j++) {
		auto node = make_shared<RrtNode>(this->randomOpenAreaPoint(), shared_ptr<RrtNode>(nullptr), -1.0);
		allNodes.push_back(node);
		this->rtree.insert(RtreeValue(node->coord.getBoostPoint(), node));
	}
}

void PrmStar::sampleGrid(vector<shared_ptr<RrtNode>> &allNodes) {
	auto ratio = this->width / (double)this->height;
	this->dx = width / 100;
	this->dy = height / (100 * ratio);

	for (int i = this->dx / 2; i < width; i += this->dx) {
		for (int j = this->dy / 2; j < height; j += this->dy) {
			if (!(*this->obstacleHash)[j][i]) {
				auto node = make_shared<RrtNode>(Coord(i, j), shared_ptr<RrtNode>(nullptr), -1.0);
				allNodes.push_back(node);
				this->rtree.insert(RtreeValue(node->coord.getBoostPoint(), node));
			}
		}
	}
}

void PrmStar::buildBaseVisibilityGraph() {
	vector<shared_ptr<RrtNode>> allNodes;
	if (this->graphType == GraphType::Random) {
		this->sampleRandom(allNodes);
	} else {
		this->sampleGrid(allNodes);
	}

	allNodes.push_back(this->root);
	this->endNode->cumulativeCost = -1.0;
	allNodes.push_back(this->endNode);

	this->rtree.insert(RtreeValue(this->root->coord.getBoostPoint(), this->root));
	this->rtree.insert(RtreeValue(this->endNode->coord.getBoostPoint(), this->endNode));

	for (auto n : allNodes) {
		vector<RtreeValue> results;
		this->getNeighbors(n->coord, 24, results);
		for (auto pair : results) {
			auto neighbor = pair.second;
			if (n != neighbor && !this->lineIntersectsObstacle(n->coord, neighbor->coord)) {
				baseVisibilityGraph[n].insert(neighbor);
			}
		}
	}
}

void PrmStar::moveStart(double dx, double dy) {
	if (dx != 0 || dy != 0) {
		this->rtree.remove(RtreeValue(this->root->coord.getBoostPoint(), this->root));
		AStar::moveStart(dx, dy);
		this->rtree.insert(RtreeValue(this->root->coord.getBoostPoint(), this->root));
	}
}

void PrmStar::replan(Coord &newEndpoint) {
	this->rtree.remove(RtreeValue(this->endNode->coord.getBoostPoint(), this->endNode));
	Planner::replan(newEndpoint);
	this->rtree.insert(RtreeValue(this->endNode->coord.getBoostPoint(), this->endNode));

	for (auto neighbor : this->baseVisibilityGraph[this->root]) {
		this->baseVisibilityGraph[neighbor].erase(this->root);
	}

	for (auto neighbor : this->baseVisibilityGraph[this->endNode]) {
		this->baseVisibilityGraph[neighbor].erase(this->endNode);
	}

	this->baseVisibilityGraph[this->root].clear();
	this->baseVisibilityGraph[this->endNode].clear();

	for (auto pair : this->baseVisibilityGraph) {
		auto neighbor = pair.first;
		neighbor->cumulativeCost = -1;
	}

	vector<RtreeValue> rootNeighbors;
	this->getNeighbors(this->root->coord, 24, rootNeighbors);
	for (auto pair : rootNeighbors) {
		auto neighbor = pair.second;
		if (neighbor != this->root && !this->lineIntersectsObstacle(this->root->coord, neighbor->coord)) {
			this->baseVisibilityGraph[this->root].insert(neighbor);
			this->baseVisibilityGraph[neighbor].insert(this->root);
		}
	}

	vector<RtreeValue> endNodeNeighbors;
	this->getNeighbors(this->endNode->coord, 24, endNodeNeighbors);
	for (auto pair : endNodeNeighbors) {
		auto neighbor = pair.second;
		if (neighbor != this->endNode && !this->lineIntersectsObstacle(this->endNode->coord, neighbor->coord)) {
			this->baseVisibilityGraph[this->endNode].insert(neighbor);
			this->baseVisibilityGraph[neighbor].insert(this->endNode);
		}
	}

	this->root->cumulativeCost = 0.0;

	this->plan();
}
