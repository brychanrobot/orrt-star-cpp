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
  return point.x < this->br.x && point.x > this->tl.x &&
           point.y < this->br.y && point.y > this->tl.y
}
