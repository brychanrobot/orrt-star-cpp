#include "AStar.hpp"

#include <algorithm>
//#include <queue>

#include "geom/utils.hpp"

using namespace std;

AStar::AStar(vector<vector<bool>> *obstacleHash, vector<Rect *> *obstacleRects, int width, int height, bool usePseudoRandom)
    : Planner(obstacleHash, obstacleRects, width, height, usePseudoRandom) {
	buildBaseVisibilityGraph();
	// plan();  // make an initial plan
}

void AStar::buildBaseVisibilityGraph() {
	printf("building visibility graph\n");
	vector<Node *> allNodes;
	for (auto obstacleRect : *this->obstacleRects) {
		vector<Coord> points;
		obstacleRect->getPoints(points);
		for (auto point : points) {
			allNodes.push_back(new Node(point, NULL, 0.0));
		}
	}

	allNodes.push_back(this->root);
	allNodes.push_back(this->endNode);

	for (auto n1 : allNodes) {
		for (auto n2 : allNodes) {
			if (n1 != n2 && !this->lineIntersectsObstacle(n1->coord, n2->coord)) {
				n1->children.push_back(n2);
			}
		}
	}

	printf("finished building visibility graph\n");
}

void AStar::plan() {
	vector<Node *> q;
	this->root->cumulativeCost = 0.0;
	this->root->heuristic = euclideanDistance(this->root->coord, this->endNode->coord);
	q.push_back(this->root);
	push_heap(q.begin(), q.end());

	while (!q.empty()) {
		auto n = q.back();
		pop_heap(q.begin(), q.end());
		q.pop_back();
		n->status = Status::Closed;
		if (n == this->endNode) {
			break;
		}

		for (auto neighbor : n->children) {
			if (neighbor->status == Status::Unvisited) {
				neighbor->status = Status::Open;
				neighbor->cumulativeCost = n->cumulativeCost + euclideanDistance(n->coord, neighbor->coord);  // this->getCost(n, neighbor);
				neighbor->parent = n;
				neighbor->heuristic = neighbor->cumulativeCost + euclideanDistance(neighbor->coord, this->endNode->coord);
				q.push_back(neighbor);
				push_heap(q.begin(), q.end());
			} else if (neighbor->status == Status::Open) {
				auto potentialCost = euclideanDistance(n->coord, neighbor->coord)  // this->getCost(n, neighbor);
				    if (n->cumulativeCost + potentialCost < neighbor->cumulativeCost) {
					neighbor->cumulativeCost = n->cumulativeCost + potentialCost;
					neighbor->parent = n;
					neighbor->heuristic = neighbor->cumulativeCost + euclideanDistance(neighbor->coord, this->endNode->coord);
					make_heap(q.begin(), q.end());
				}
			}
		}
	}

	this->refreshBestPath();
}

void AStar::replan(Coord &newEndpoint) {
	printf("replanning\n");
	Planner::replan(newEndpoint);

	this->plan();
}
