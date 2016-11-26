#pragma once

#include <stdlib.h>
#include <cmath>
// #include <math.h>
#include "../geom/Rect.hpp"
#include "../geom/utils.hpp"

bool hasIntersection(Rect& rect, std::vector<Rect*>& obstacles) {
	for (auto obstacle : obstacles) {
		if (obstacle->intersects(rect)) {
			return true;
		}
	}

	return false;
}

void generateObstacleRects(int width, int height, int count, std::vector<Rect*>& obstacles, double padding = 5) {
	for (int x = 0; x < count; x++) {
		Rect* rect = NULL;
		while (true) {
			auto topLeft = randomPoint(width, height);
			auto bottomRight = randomPoint(width, height);

			delete rect;
			rect = new Rect(topLeft, bottomRight);
			if (rect->width() > padding * 2 && rect->height() > padding * 2 && !hasIntersection(*rect, obstacles)) {
				break;
			}
		}

		obstacles.push_back(rect);
	}
}

void generateObstacleHash(std::vector<Rect*>& obstacleRects, std::vector<std::vector<bool>>& obstacleHash) {
	for (auto obstacle : obstacleRects) {
		for (int r = obstacle->topLeft.y(); r < obstacle->bottomRight.y(); r++) {
			for (int c = obstacle->topLeft.x(); c < obstacle->bottomRight.x(); c++) {
				obstacleHash[r][c] = true;
			}
		}
	}
}
