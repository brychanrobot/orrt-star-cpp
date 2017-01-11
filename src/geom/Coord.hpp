#pragma once

/*
struct Coord {
  double x;
  double y;
  Coord(){}
  Coord(int x, int y) : x(x), y(y) {}
};
*/

#include <boost/geometry/geometries/geometries.hpp>
typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> point;

class Coord : public point {
   public:
	Coord() : point(0, 0) {}
	Coord(double x, double y) : point(x, y) {}

	double x() { return this->get<0>(); }

	void x(double x) { this->set<0>(x); }

	double y() { return this->get<1>(); }

	void y(double y) { this->set<1>(y); }

	void change(double x, double y) {
		this->set<0>(x);
		this->set<1>(y);
	}
};
