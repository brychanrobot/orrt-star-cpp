#include <queue>

using namespace std;

AStar::AStar(vector<vector<bool>> *obstacleHash, vector<Rect *> *obstacleRects, int width, int height, bool usePseudoRandom)
    : Planner(obstacleHash, obstacleRects, width, height, usePseudoRandom) {
	buildBaseVisibilityGraph();
}

void AStar::buildBaseVisibilityGraph() {
	vector<Node> allNodes;
	for (auto obstacleRect : obstacleRects) {
		vector<Coord> points;
		obstacleRect.points(points);
		for (auto point : points) {
			allNodes.push_back(Node(point, NULL, 0.0));
		}
	}

	for (auto n1 : allNodes) {
		for (auto n2 : allNodes) {
			if (!this->lineIntersectsObstacle(n1.coord, n2.coord)) {
				baseVisibilityGraph[&n1].children.push_back(&n2);
			}
		}
	}
}

void AStar::replan(Coord &newEndpoint) {
	Planner::replan(newEndpoint);

	priority_queue<Coord *> q;
	this->root->cumulativeCost = 0.0;
	auto heuristic = euclideanDistance(this->root, this->endNode);
	q.push(this->root);

	while (!q.empty()) {
		n = q.top();
		q.pop();
		n->node.status = Status::Closed;
		if (n == this->endNode) {
			break;
		}

		for (auto neighbor : n->children) {
			if (neighbor.cumulativeCost == 0.0) {
				neighbor.status = Status::Open;
				neighbor.cumulativeCost = n.cumulativeCost + this->getCost(n, neighbor);
				neighbor.parent = n;
			}
		}
	}
}
