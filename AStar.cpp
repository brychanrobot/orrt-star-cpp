
using namespace std;

AStar::AStar(vector<vector<bool>> *obstacleHash, vector<Rect *> *obstacleRects, int width, int height, bool usePseudoRandom)
    : Planner(obstacleHash, obstacleRects, width, height, usePseudoRandom) {}

void AStar::replan(Coord &newEndpoint) {
	Planner::replan(newEndpoint);
	// stuff
}
