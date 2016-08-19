#pragma once

class Rect {
private:
  Coord topLeft;
  Coord bottomRight;
public:
  Rect(Coord tl, Coord br);
  bool contains(Coord point);
};
