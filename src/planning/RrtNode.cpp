#include "RrtNode.hpp"

using namespace std;

RrtNode::RrtNode(Coord coord, shared_ptr<RrtNode> parent, double cumulativeCost) : Node(coord, parent) { this->cumulativeCost = cumulativeCost; }

bool RrtNode::operator<(shared_ptr<RrtNode> rhs) { return this->heuristic < rhs->heuristic; }

void RrtNode::addChild(shared_ptr<RrtNode> child, double cost) {
	this->children.push_back(child);
	child->parent = shared_from_this();
	child->cumulativeCost = this->cumulativeCost + cost;
}

void RrtNode::rewire(shared_ptr<RrtNode> newParent, double cost) {
	if (this->parent) {
		this->parent->removeChild(shared_from_this());
	}
	newParent->children.push_back(shared_from_this());
	this->parent = newParent;
	this->updateCumulativeCost(newParent->cumulativeCost + cost);
}

void RrtNode::updateCumulativeCost(double newCumulativeCost) {
	auto oldCumulativeCost = this->cumulativeCost;
	this->cumulativeCost = newCumulativeCost;
	for (const shared_ptr<Node>& childBase : this->children) {
		auto child = static_pointer_cast<RrtNode>(childBase);
		auto costToParent = child->cumulativeCost - oldCumulativeCost;
		child->updateCumulativeCost(newCumulativeCost + costToParent);
	}
}
