#pragma once

#include <stdlib.h>
#include "geom/Rect.hpp"

bool hasIntersection(Rect* rect, std::vector<Rect>* obstacles)
{
  for (auto obstacle:*obstacles)
  {
    if(obstacle.intersects(*rect))
    {
      return true;
    }
  }

  return false;
}

std::vector<Rect>* generateObstacles(int width, int height, int count)
{
  auto obstacles = new std::vector<Rect>();

  for(int x=0; x<count; x++)
  {
  }

  return obstacles;
}
