#pragma once

#include <math>

double angleBetweenCoords(Coord &c1, Coord &c2) {
  atan2(c2.y-c1.y, c2.x-c1.y)
}

double euclideanDistance(Coord &c1, Coord &c2) {
  dx = c1.x - c2.x
  dy = c1.y - c2.y

  return sqrt(dx*dx + dy*dy)
}

Coord randomPoint(int dx, int dy) {
  return Coord(rand()%dx, rand()%dy);
}
