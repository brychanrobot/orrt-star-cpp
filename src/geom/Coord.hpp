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
	std::vector<double> c = {0, 0};

   public:
	Coord() {}
	Coord(double x, double y) {
		c[0] = x;
		c[1] = y;
	}

	double x() { return this->c[0]; }

	double y() { return this->c[1]; }

	void change(double x, double y) {
		this->c[0] = x;
		this->c[1] = y;
	}

	point getBoostPoint() { return point(c[0], c[1]); }
};
