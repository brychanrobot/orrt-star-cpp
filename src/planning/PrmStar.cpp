#include "PrmStar.hpp"

#include "../geom/utils.hpp"

using namespace std;

PrmStar::PrmStar(vector<vector<bool>> *obstacleHash, vector<shared_ptr<Rect>> *obstacleRects, int width, int height, bool usePseudoRandom)
    : AStar(obstacleHash, obstacleRects, width, height, usePseudoRandom, false) {
	this->buildBaseVisibilityGraph();
	this->plan();  // make an initial plan
}

PrmStar::~PrmStar() {}

void PrmStar::buildBaseVisibilityGraph() {
	vector<shared_ptr<Node>> allNodes;
	for (auto j = 0; j < 0.02 * this->width * this->height; j++) {
		allNodes.push_back(make_shared<Node>(this->randomOpenAreaPoint(), shared_ptr<Node>(nullptr), -1.0));
	}

	allNodes.push_back(this->root);
	this->endNode->cumulativeCost = -1.0;
	allNodes.push_back(this->endNode);

	for (auto n1 : allNodes) {
		for (auto n2 : allNodes) {
			if (n1 != n2 && euclideanDistance(n1->coord, n2->coord) < 24 && !this->lineIntersectsObstacle(n1->coord, n2->coord)) {
				baseVisibilityGraph[n1].insert(n2);
				// n1->children.push_back(n2);
			}
		}
	}
	printf("prm*\n");
}

void PrmStar::replan(Coord &newEndpoint) {
	Planner::replan(newEndpoint);

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

		if (neighbor != this->root && euclideanDistance(this->root->coord, neighbor->coord) < 24 &&
		    !this->lineIntersectsObstacle(this->root->coord, neighbor->coord)) {
			this->baseVisibilityGraph[this->root].insert(neighbor);
			this->baseVisibilityGraph[neighbor].insert(this->root);
		}

		if (neighbor != this->endNode && euclideanDistance(this->endNode->coord, neighbor->coord) < 24 &&
		    !this->lineIntersectsObstacle(this->endNode->coord, neighbor->coord)) {
			this->baseVisibilityGraph[this->endNode].insert(neighbor);
			this->baseVisibilityGraph[neighbor].insert(this->endNode);
		}
	}

	this->root->cumulativeCost = 0.0;

	this->plan();
}
