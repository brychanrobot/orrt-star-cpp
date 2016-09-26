#pragma once
#include "Coord.hpp"

class Rect {
private:
public:
  Coord topLeft;
  Coord bottomRight;
  Rect(Coord tl, Coord br);
  bool contains(Coord point);
  bool intersects(Rect& rect);
  void inflate(double dx, double dy);
  double width();
  double height();
};
