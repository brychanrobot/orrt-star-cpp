#pragma once

#include <list>
#include "../planning-utils/geom/Coord.hpp"

enum Status { Unvisited, Open, Closed };

class Node : public std::enable_shared_from_this<Node> {
   private:
	void updateCumulativeCost(double newCumulativeCost);

   public:
	Status status = Status::Unvisited;
	Coord coord;
	double cumulativeCost;
	double heuristic;
	std::shared_ptr<Node> parent;
	std::list<std::shared_ptr<Node>> children;

	Node() {}
	~Node() {}
	Node(Coord coord, std::shared_ptr<Node> parent, double cumulativeCost);
	bool operator<(std::shared_ptr<Node> rhs);
	void addChild(std::shared_ptr<Node> child, double cost);
	void removeChild(std::shared_ptr<Node> child);
	void printChildren();
	void rewire(std::shared_ptr<Node> newParent, double cost);
};
