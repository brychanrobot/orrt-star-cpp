#pragma once

#include <vector>
#include "../geom/Coord.hpp"

enum Status { Unvisited, Open, Closed };

class Node {
   private:
	void removeChild(std::shared_ptr<Node> child);
	void updateCumulativeCost(double newCumulativeCost);

   public:
	Status status = Status::Unvisited;
	Coord coord;
	double cumulativeCost;
	double heuristic;
	std::shared_ptr<Node> parent;
	std::vector<std::shared_ptr<Node>> children;

	Node() {}
	Node(Coord coord, std::shared_ptr<Node> parent, double cumulativeCost);
	bool operator<(const std::shared_ptr<Node> rhs);
	void addChild(std::shared_ptr<Node> child, double cost);
	void printChildren();
	void rewire(std::shared_ptr<Node> newParent, double cost);
};
