#pragma once

#include "geom/Coord.hpp"
#include <vector>

enum Status {
  Unvisited,
  Open,
  Closed
};

class Node {
private:
  Coord coord;
  Node* parent;
  std::vector<Node*> children;
  double cumulativeCost;

public:
  Status status = Status::Unvisited;
  Node(){}
  Node(Coord coord, Node*parent, double cumulativeCost);
};
