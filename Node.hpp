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

public:
  Status status = Status::Unvisited;
  Coord coord;
  double cumulativeCost;
  Node* parent;
  std::vector<Node*> children;

  Node(){}
  Node(Coord coord, Node*parent, double cumulativeCost);
  bool operator<(const Node* rhs);
  void addChild(Node* child, double cost);
};
