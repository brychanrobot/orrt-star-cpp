#include "Node.hpp"

using namespace std;

Node::Node(Coord coord, shared_ptr<Node> parent, double cumulativeCost) {
	this->coord = coord;
	this->parent = parent;
	this->cumulativeCost = cumulativeCost;
}

bool Node::operator<(const shared_ptr<Node> rhs) { return this->heuristic < rhs->heuristic; }

void Node::addChild(shared_ptr<Node> child, double cost) {
	this->children.push_back(child);
	child->parent = shared_ptr<Node>(this);
	child->cumulativeCost = this->cumulativeCost + cost;
}

void Node::removeChild(shared_ptr<Node> child) {
	for (auto iter = this->children.begin(); iter != this->children.end(); ++iter) {
		if (*iter == child) {
			this->children.erase(iter);
			break;
		}
	}
}

void Node::rewire(shared_ptr<Node> newParent, double cost) {
	if (this->parent) {
		this->parent->removeChild(shared_ptr<Node>(this));
	}
	newParent->children.push_back(shared_ptr<Node>(this));
	this->parent = newParent;
	this->updateCumulativeCost(newParent->cumulativeCost + cost);
}

void Node::updateCumulativeCost(double newCumulativeCost) {
	auto oldCumulativeCost = this->cumulativeCost;
	this->cumulativeCost = newCumulativeCost;
	for (auto child : this->children) {
		auto costToParent = child->cumulativeCost - oldCumulativeCost;
		child->updateCumulativeCost(newCumulativeCost + costToParent);
	}
}

void Node::printChildren() {
	for (auto child : this->children) {
		printf("(%.2f, %.2f) ", child->coord.x(), child->coord.y());
	}
	printf("\n");
}
