#include "Node.hpp"

Node::Node(Coord coord, Node*parent, double cumulativeCost) {
  this->coord = coord;
  this->parent = parent;
  this->cumulativeCost = cumulativeCost;
}
