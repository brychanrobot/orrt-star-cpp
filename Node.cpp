#include "Node.hpp"

Node::Node(Coord coord, Node* parent, double cumulativeCost) {
	this->coord = coord;
	this->parent = parent;
	this->cumulativeCost = cumulativeCost;
}

bool Node::operator<(const Node* rhs) { return this->cumulativeCost < rhs->cumulativeCost; }

void Node::addChild(Node* child, double cost) {
	this->children.push_back(child);
	child->parent = this;
	child->cumulativeCost = this->cumulativeCost + cost;
}

void Node::removeChild(Node* child) {
	for (auto iter = this->children.begin(); iter != this->children.end(); ++iter) {
		if (*iter == child) {
			this->children.erase(iter);
			break;
		}
	}
}

void Node::rewire(Node* newParent, double cost) {
	if (this->parent != NULL) {
		auto size = this->parent->children.size();
		this->parent->removeChild(this);
	}
	newParent->children.push_back(this);
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
