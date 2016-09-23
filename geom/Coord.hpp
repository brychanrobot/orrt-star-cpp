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

class Coord : public point
{
public:
  Coord(){}
  Coord(double x, double y) : point(x, y){}

  double x()
  {
    return this->get<0>();
  }

  double y()
  {
    return this->get<1>();
  }
};
