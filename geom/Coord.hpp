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
typedef boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> point;

class Coord : public point {
   public:
	Coord() {}
	Coord(double x, double y, double z) : point(x, y, z) {}

	double x() { return this->get<0>(); }

	double y() { return this->get<1>(); }

	double z() { return this->get<2>(); }
};
