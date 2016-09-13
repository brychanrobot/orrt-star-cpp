#include "Rect.hpp"

Rect::Rect(Coord tl, Coord br) {
  this->topLeft = tl;
  this->bottomRight = br;
}

void Rect::inflate(dx, dy) {
  this->topLeft.x -= dx
  this->topLeft.y -= dy

  this->bottomRight.x += dx
  this->bottomRight.y += dy
}

bool Rect::contains(Coord point) {
  return point.x < this->bottomRight.x && point.x > this->topLeft.x &&
           point.y < this->bottomRight.y && point.y > this->topLeft.y
}

bool valueInRange(double value, double min, double max) {
  return (value >= min) && (value <= max);
}

bool Rect::intersects(Rect rect) {
  bool xOverlap = valueInRange(this->topLeft.x, rect.topLeft.x, rect.bottomRight.x) ||
                  valueInRange(rect.topLeft.x, this->topLeft.x, this->bottomRight.x);

  bool yOverlap = valueInRange(this->topLeft.y, rect.topLeft.y, rect.bottomRight.y) ||
                  valueInRange(rect.topLeft.y, this->topLeft.y, this->bottomRight.y);

  return xOverlap && yOverlap;
}
