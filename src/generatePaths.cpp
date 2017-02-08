#include <cstdlib>
#include <fstream>
#include <iostream>
#include <set>
#include "planning-utils/libs/cxxopts.hpp"
#include "planning/AStar.hpp"
#include "planning-utils/utils.hpp"

using namespace std;

const double OBSTACLE_PADDING = 5;

void saveMap(string filename, vector<vector<bool>>& map) {
	ofstream f(filename, std::ofstream::out);
	f << "[";
	for (uint r = 0; r < map.size(); r++) {
		f << "[";
		for (uint c = 0; c < map[r].size(); c++) {
			f << ((map[r][c]) ? 1 : 0);
			if (c < map[r].size() - 1) {
				f << ", ";
			}
		}
		if (r < map.size() - 1) {
			f << "],";
		} else {
			f << "]";
		}
	}
	f << "]";
	f.close();
}

void saveMapRects(string filename, vector<shared_ptr<Rect>> rects) {
	ofstream f(filename, std::ofstream::out);
	f << "[";
	for (uint r = 0; r < rects.size(); r++) {
		f << "[[" << rects[r]->topLeft.x << "," << rects[r]->topLeft.y << "],[";
		f << rects[r]->bottomRight.x << "," << rects[r]->bottomRight.y << "]]";
		if (r != rects.size() - 1) {
			f << ", ";
		}
	}
	f << "]";
}

void appendPath(string filename, Coord& start, Coord& end, deque<Coord>& path) {
	ofstream f(filename, std::ofstream::out | std::ofstream::app);
	if (f.tellp() == 0) {
		f << "[";
	} else {
		f << ", ";
	}

	f << "[[" << start.x << "," << start.y << "], [" << end.x << "," << end.y << "], [";
	for (uint c = 0; c < path.size(); c++) {
		if (c == 0) {
			f << "[";
		} else {
			f << ",[";
		}
		f << path[c].x << "," << path[c].y << "]";
	}
	// f.seekp(2, ios_base::end);
	// f << "\b";
	// f.seekp(f.tellp() - 1);
	// f.write("],", 2);
	f << "]]";
	f.close();
}

int main(int argc, char* argv[]) {
	srand(time(0));

	int width = 50;
	int height = 50;
	bool usePseudoRandom = true;

	AStar* planner = NULL;
	shared_ptr<vector<shared_ptr<Rect>>> obstacleRects;
	vector<vector<bool>> obstacleHash;

	for (int mapNum = 0; mapNum < 1000; mapNum++) {
		delete planner;
		obstacleRects.clear();
		int num_obstacles = rand() % 4 + 1;
		printf("%d\n", num_obstacles);
		generateObstacleRects(width, height, num_obstacles, obstacleRects, OBSTACLE_PADDING);

		obstacleHash = vector<vector<bool>>(height, vector<bool>(width, false));
		generateObstacleHash(obstacleRects, obstacleHash);

		planner = new AStar(&obstacleHash, &obstacleRects, width, height, usePseudoRandom);
		appendPath("data/" + to_string(mapNum) + "paths.json", planner->root->coord, planner->endNode->coord, planner->bestPath);

		saveMap("data/" + to_string(mapNum) + "map.json", obstacleHash);
		saveMapRects("data/" + to_string(mapNum) + "maprects.json", obstacleRects);

		for (int replanCount = 0; replanCount < 500; replanCount++) {
			planner->randomStart();
			planner->randomReplan();

			appendPath("data/" + to_string(mapNum) + "paths.json", planner->root->coord, planner->endNode->coord, planner->bestPath);
		}

		ofstream f("data/" + to_string(mapNum) + "paths.json", std::ofstream::app);
		f << "]";
	}
}
