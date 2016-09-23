#include "OnlineFmtStar.hpp"
#include "geom/utils.hpp"
#include "geom/Coord.hpp"
#include <cstdlib>
#include <stdio.h>
#include <ctime>
#include <limits>

using namespace std;


OnlineFmtStar::OnlineFmtStar(vector<vector<bool>>* obstacleHash, vector<Rect>* obstacleRects,
  double maxSegment, int width, int height)
{

    srand(time(NULL)); // initialize the random number generator so it happens

    this->width = width;
    this->height = height;
    this->mapArea = width * height;
    this->obstacleHash = obstacleHash;
    this->obstacleRects = obstacleRects;
    this->maxSegment = maxSegment;
    this->rewireNeighborhood = maxSegment*6;
    this->nodeAddThreshold = 0.015 * width*height;

    this->startPoint = this->randomOpenAreaPoint();

    do
    {
      this->endPoint = this->randomOpenAreaPoint();
    } while(euclideanDistance(this->startPoint, this->endPoint) < width/2.0);


    this->root = Node(this->startPoint, NULL, 0.0);
    this->root.status = Status::Open;
    this->rtree.insert(RtreeValue(this->startPoint, &this->root));
    this->open.push(&this->root);

    endNode = Node(this->endPoint, NULL, 0.0);
    this->rtree.insert(RtreeValue(this->endPoint, &endNode));

    for(int n = 0; n < this->nodeAddThreshold; n++)
    {
      auto point = this->randomOpenAreaPoint();

      auto node = Node(point, NULL, numeric_limits<double>::max());
      this->rtree.insert(RtreeValue(point, &node));
    }
}

void OnlineFmtStar::sample()
{
  if (!this->open.empty())
  {
    this->sampleAndAdd();
  }
}

void OnlineFmtStar::sampleAndAdd()
{
  auto bestOpenNode = this->open.top();
  this->open.pop();

  vector<RtreeValue> neighbors;
  this->getNeighbors(bestOpenNode->coord, this->rewireNeighborhood, neighbors);

  for(auto neighbor_tuple : neighbors)
  {
    auto neighbor = neighbor_tuple.second;
    if (neighbor->status == Status::Unvisited)
    {
      Node* bestParent = NULL;
      double bestCost;
      findBestOpenNeighbor(neighbor, bestParent, bestCost);
      if (bestParent != NULL)
      {
          bestParent->addChild(neighbor, bestCost);
          neighbor->status = Status::Open;
          this->open.push(neighbor);
      }
    }
  }

  bestOpenNode->status = Status::Closed;

}


Coord OnlineFmtStar::randomOpenAreaPoint()
{
  while(true)
  {
    Coord point = randomPoint(this->width, this->height);
    if(!this->obstacleHash->at((int)point.y()).at((int)point.x())) {

      return point;
    }
  }
}

double OnlineFmtStar::getCost(Node* start, Node* end)
{
  return euclideanDistance(start->coord, end->coord);
}

void OnlineFmtStar::getNeighbors(Coord center, double radius, vector<RtreeValue>& results)
{
  box query_box(point(center.x()-radius, center.y()-radius), point(center.x()+radius, center.y()+radius));
  this->rtree.query(boost::geometry::index::intersects(query_box), std::back_inserter(results));
}

void OnlineFmtStar::findBestOpenNeighbor(Node* node, Node*& bestNeighbor, double& bestCost)
{
  vector<RtreeValue> neighbors;
  this->getNeighbors(node->coord, this->rewireNeighborhood, neighbors);

  double bestCumulativeCost = std::numeric_limits<double>::max();
  for (auto neighbor_tuple : neighbors)
  {
    auto neighbor = neighbor_tuple.second;
    auto cost = this->getCost(neighbor, node);
    if (neighbor->status == Status::Open && cost + neighbor->cumulativeCost < bestCumulativeCost)
    {
      bestCost = cost;
      bestCumulativeCost = cost + neighbor->cumulativeCost;
      bestNeighbor = neighbor;
    }
  }
}
