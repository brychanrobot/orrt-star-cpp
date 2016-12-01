#include "PrmStar.hpp"

using namespace std;

PrmStar::PrmStar(vector<vector<bool>> *obstacleHash, vector<shared_ptr<Rect>> *obstacleRects, int width, int height, bool usePseudoRandom)
    : AStar(obstacleHash, obstacleRects, width, height, usePseudoRandom) {
	buildBaseVisibilityGraph();
	plan();  // make an initial plan
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
			if (n1 != n2 && !this->lineIntersectsObstacle(n1->coord, n2->coord)) {
				baseVisibilityGraph[n1].insert(n2);
				// n1->children.push_back(n2);
			}
		}
	}
}
