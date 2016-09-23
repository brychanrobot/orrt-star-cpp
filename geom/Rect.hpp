#pragma once
#include "Coord.hpp"

class Rect {
private:
  Coord topLeft;
  Coord bottomRight;
public:
  Rect(Coord tl, Coord br);
  bool contains(Coord point);
  bool intersects(Rect rect);
  void inflate(double dx, double dy);
};
