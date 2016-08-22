#pragma once

#include "Node.hpp"
#include "geom/Coord.hpp"
#include "geom/Rect.hpp"
#include <vector>

class OnlineFmtStar {
private:
  int width;
  int height;
  int mapArea;
  std::vector<std::vector<bool>>* obstacleHash;
  std::vector<Rect>* obstacleRects;
  int maxSegment;
  int rewireNeighborhood;
  Node root;
  Coord startPoint;
  Coord endPoint;
  Node endNode;
  int nodeAddThreshold;

  Coord randomOpenAreaPoint();

public:
  OnlineFmtStar(std::vector<std::vector<bool>>* obstacleHash, std::vector<Rect>* obstacleRects,
    double maxSegment, int width, int height);
};
