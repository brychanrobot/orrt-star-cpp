#pragma once

#include "Coord.hpp"
#include <cstdlib>
#include <cmath>

inline double angleBetweenCoords(Coord &c1, Coord &c2) {
  return atan2(c2.y()-c1.y(), c2.x()-c1.y());
}

inline double euclideanDistance(Coord &c1, Coord &c2) {
  double dx = c1.x() - c2.x();
  double dy = c1.y() - c2.y();

  return sqrt(dx*dx + dy*dy);
}

inline Coord randomPoint(int dx, int dy) {
  return Coord(rand()%dx, rand()%dy);
}
