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
typedef boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> point3;

class Coord3 : public point3
{
public:
  Coord3(){}
  Coord3(double x, double y, double z) : point3(x, y, z){}

  double x()
  {
    return this->get<0>();
  }

  double y()
  {
    return this->get<1>();
  }

  double z()
  {
    return this->get<2>();
  }

  void setx(double x)
  {
    this->set<0>(x);
  }

  void sety(double y)
  {
    this->set<1>(y);
  }

  void setz(double z)
  {
    this->set<2>(z);
  }
};
