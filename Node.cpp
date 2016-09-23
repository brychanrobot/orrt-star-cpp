#include "Node.hpp"

Node::Node(Coord coord, Node* parent, double cumulativeCost)
{
  this->coord = coord;
  this->parent = parent;
  this->cumulativeCost = cumulativeCost;
}

bool Node::operator<(const Node* rhs)
{
  return this->cumulativeCost < rhs->cumulativeCost;
}

void Node::addChild(Node* child, double cost)
{
  this->children.push_back(child);
  child->parent = this;
  child->cumulativeCost = this->cumulativeCost + cost;
}
