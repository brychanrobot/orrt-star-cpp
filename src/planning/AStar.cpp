#include "AStar.hpp"

#include <algorithm>
#include <queue>

#include "../planning-utils/geom/utils.hpp"

using namespace std;

//////////////////////////
// Adapted from Red Blob Games example
////////////////////////////
template <typename T, typename priority_t>
struct PriorityQueue {
	typedef pair<priority_t, T> PQElement;
	priority_queue<PQElement, vector<PQElement>, std::greater<PQElement>> elements;

	inline bool empty() const { return this->elements.empty(); }

	inline void put(T item, priority_t priority) { this->elements.emplace(priority, item); }

	inline T get() {
		T best_item = this->elements.top().second;
		this->elements.pop();
		return best_item;
	}
};
/////////////////////////////
////////////////////////////

AStar::AStar(shared_ptr<vector<vector<bool>>> obstacleHash, shared_ptr<vector<shared_ptr<Rect>>> obstacleRects, int width, int height, bool usePseudoRandom,
             bool initialize)
    : Planner(obstacleHash, obstacleRects, width, height, usePseudoRandom) {
	this->name = "astar_visibility";
	if (initialize) {
		buildBaseVisibilityGraph();
		plan();  // make an initial plan
	}
}

AStar::~AStar() {}

void AStar::buildBaseVisibilityGraph() {
	vector<shared_ptr<RrtNode>> allNodes;
	for (auto obstacleRect : *this->obstacleRects) {
		vector<Coord> points;
		obstacleRect->getPoints(points);
		for (auto point : points) {
			allNodes.push_back(make_shared<RrtNode>(point, shared_ptr<RrtNode>(nullptr), -1.0));
		}
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
	printf("A*\n");
}

/*
void AStar::buildBaseVisibilityGraph() {
    vector<shared_ptr<RrtNode>> allNodes;
    auto ratio = this->width / (double)this->height;
    this->dx = width / 100;
    this->dy = height / (100 * ratio);

    for (int i = this->dx / 2; i < width; i += this->dx) {
        for (int j = this->dy / 2; j < height; j += this->dy) {
            if (!(*this->obstacleHash)[j][i]) {
                auto node = make_shared<RrtNode>(Coord(i, j), shared_ptr<RrtNode>(nullptr), -1.0);
                allNodes.push_back(node);
                this->rtree.insert(RtreeValue(node->coord, node));
            }
        }
    }

    allNodes.push_back(this->root);
    this->endNode->cumulativeCost = -1.0;
    allNodes.push_back(this->endNode);

    for (auto n1 : allNodes) {
        vector<RtreeValue> results;
        this->getNeighbors(n1->coord, (this->dx + this->dy), results);
        for (auto pair : results) {
            auto n2 = pair.second;
            if (n1 != n2 && euclideanDistance(n1->coord, n2->coord) < this->dx + this->dy && !this->lineIntersectsObstacle(n1->coord, n2->coord)) {
                baseVisibilityGraph[n1].insert(n2);
            }
        }
    }
}
*/

void AStar::plan() {
	PriorityQueue<shared_ptr<RrtNode>, double> frontier;
	frontier.put(this->root, 0.0);

	while (!frontier.empty()) {
		auto current = frontier.get();

		if (current == this->endNode) {
			break;
		}

		for (auto next : baseVisibilityGraph[current]) {
			auto new_cost = current->cumulativeCost + this->getCost(current, next);
			if (next->cumulativeCost < 0 || new_cost < next->cumulativeCost) {
				next->cumulativeCost = new_cost;
				double priority = new_cost + euclideanDistance(next->coord, this->endNode->coord);
				frontier.put(next, priority);
				next->parent = current;
			}
		}
	}

	this->refreshBestPath();
}

void AStar::moveStart(double dx, double dy) {
	if (dx != 0 || dy != 0) {
		Coord point(clamp(this->root->coord.x + dx, 0, this->width - 1), clamp(this->root->coord.y + dy, 0, this->height - 1));

		if (!this->obstacleHash->at((int)point.y).at((int)point.x)) {
			this->root->coord = point;

			this->bestPath[0] = point;

			if (euclideanDistance(this->root->coord, this->bestPath[1]) < 1.5) {
				this->bestPath.pop_front();
				this->bestPath[0] = this->root->coord;
			}
		}
	}
}

void AStar::randomStart() { this->root->coord = this->randomOpenAreaPoint(); }

void AStar::replan(Coord &newEndpoint) {
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

		if (neighbor != this->root && !this->lineIntersectsObstacle(this->root->coord, neighbor->coord)) {
			this->baseVisibilityGraph[this->root].insert(neighbor);
			this->baseVisibilityGraph[neighbor].insert(this->root);
		}

		if (neighbor != this->endNode && !this->lineIntersectsObstacle(this->endNode->coord, neighbor->coord)) {
			this->baseVisibilityGraph[this->endNode].insert(neighbor);
			this->baseVisibilityGraph[neighbor].insert(this->endNode);
		}
	}

	this->root->cumulativeCost = 0.0;

	this->plan();
}
