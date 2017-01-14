#pragma once

#include "../planning-utils/Node.hpp"

class RrtNode : public Node {
   private:
	void updateCumulativeCost(double newCumulativeCost);

   public:
	double cumulativeCost;
	double heuristic;

	RrtNode() {}
	//~RrtNode() {}
	RrtNode(Coord coord, std::shared_ptr<RrtNode> parent, double cumulativeCost);
	bool operator<(std::shared_ptr<RrtNode> rhs);
	void addChild(std::shared_ptr<RrtNode> child, double cost);
	void rewire(std::shared_ptr<RrtNode> newParent, double cost);
};
