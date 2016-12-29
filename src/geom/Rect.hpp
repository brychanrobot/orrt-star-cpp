#pragma once
#include "Coord.hpp"

#include <vector>

class Rect {
   private:
   public:
	Coord topLeft;
	Coord bottomRight;
	Rect(Coord tl, Coord br);
	Rect(double tlx, double tly, double brx, double bry);
	bool contains(Coord point);
	bool intersects(Rect& rect);
	void inflate(double dx, double dy);
	double width();
	double height();
	void getPoints(std::vector<Coord>& points);
};
