#pragma once

#include "Node.hpp"
#include "geom/Coord.hpp"
#include "geom/Rect.hpp"
#include <vector>
#include <queue>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/index/rtree.hpp>

//template<typename Params>;
typedef boost::geometry::model::box<point> box;
typedef std::pair<point, Node*> RtreeValue;
typedef boost::geometry::index::rtree<RtreeValue, boost::geometry::index::rstar<16> > Rtree;

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
  Node endNode;
  int nodeAddThreshold;
  Rtree rtree;
  std::priority_queue<Node*> open;

  Coord randomOpenAreaPoint();
  double getCost(Node* start, Node* end);
  void getNeighbors(Coord center, double radius, std::vector<RtreeValue>& results);
  void findBestOpenNeighbor(Node* node, Node*& bestNeighbor, double& bestCost);
  void sampleAndAdd();

public:
  Coord startPoint;
  Coord endPoint;
  OnlineFmtStar(std::vector<std::vector<bool>>* obstacleHash, std::vector<Rect>* obstacleRects,
    double maxSegment, int width, int height);
  void sample();
};
