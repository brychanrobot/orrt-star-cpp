#include "OnlineFmtStar.hpp"
#include "geom/utils.hpp"
#include <cstdlib>
#include <ctime>
#include <limits>

using namespace std;


OnlineFmtStar::OnlineFmtStar(vector<vector<bool>>* obstacleHash, vector<Rect>* obstacleRects,
  double maxSegment, int width, int height) {

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

    do {
      this->endPoint = this->randomOpenAreaPoint();
    } while(euclideanDistance(this->startPoint, this->endPoint) < width/2.0)

    this->root = Node(this->startPoint, NULL, 0.0);
    this->root.status = Status::Open;

    endNode = Node(this->endPoint, NULL, 0.0);

    for n = 0; n < this->nodeAddThreshold; n++ {
      point = this->randomOpenAreaPoint();

      node = Node(point, NULL, numeric_limits<double>::max());
    }
}

Coord OnlineFmtStar::randomOpenAreaPoint() {
  while(true) {
    point = randomPoint(this->width, this->height)
    if(!obstacleHash[point.y][point.x]) {
      return point;
    }
  }
}
