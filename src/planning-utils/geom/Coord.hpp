#pragma once

/*
struct Coord {
  double x;
  double y;
  Coord(){}
  Coord(int x, int y) : x(x), y(y) {}
};
*/
#include <vector>

#include <boost/geometry/geometries/geometries.hpp>
typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> point;

class Coord {
   private:

   public:
	double x;
	double y;

	Coord() {}
	Coord(double x, double y) {
		this->x = x;
		this->y = y;
	}

	// double x() { return this->c[0]; }

	// double y() { return this->c[1]; }

	void change(double x, double y) {
		this->x = x;
		this->y = y;
	}

	point getBoostPoint() { return point(this->x, this->y); }
};
