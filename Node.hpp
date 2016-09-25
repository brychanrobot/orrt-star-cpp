#pragma once

#include <vector>
#include "geom/Coord.hpp"

enum Status { Unvisited, Open, Closed };

class Node {
   private:
	void removeChild(Node* child);
	void updateCumulativeCost(double newCumulativeCost);

   public:
	Status status = Status::Unvisited;
	Coord coord;
	double cumulativeCost;
	Node* parent;
	std::vector<Node*> children;

	Node() {}
	Node(Coord coord, Node* parent, double cumulativeCost);
	bool operator<(const Node* rhs);
	void addChild(Node* child, double cost);
	void printChildren();
	void rewire(Node* newParent, double cost);
};
