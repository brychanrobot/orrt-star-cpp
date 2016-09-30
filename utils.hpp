#pragma once

#include <stdlib.h>
#include <cmath>
#include "geom/Rect.hpp"
#include "geom/utils.hpp"

bool hasIntersection(Rect& rect, std::vector<Rect*>& obstacles) {
	for (auto obstacle : obstacles) {
		if (obstacle->intersects(rect)) {
			return true;
		}
	}

	return false;
}

void generateObstacleRects(double width, double height, int count, std::vector<Rect*>& obstacles) {
	for (int x = 0; x < count; x++) {
		Rect* rect;
		while (true) {
			auto topLeft = randomPoint(width, height);
			auto bottomRight = randomPoint(width, height);

			rect = new Rect(topLeft, bottomRight);
			// printf("%.2f, %.2f, %d\n", rect->width(), rect->height(), hasIntersection(*rect, obstacles));
			if (rect->width() > 0 && rect->height() > 0 && !hasIntersection(*rect, obstacles)) {
				break;
			}
		}

		obstacles.push_back(rect);
	}
}

void generateObstacleHash(std::vector<Rect*>& obstacleRects, std::vector<std::vector<bool>>& obstacleHash, int obstacleHashWidth,
                          int obstacleHashHeight) {
	for (auto obstacle : obstacleRects) {
		for (int r = obstacle->bottomRight.y() * obstacleHashHeight; r < obstacle->topLeft.y() * obstacleHashHeight; r++) {
			for (int c = obstacle->topLeft.x() * obstacleHashWidth; c < obstacle->bottomRight.x() * obstacleHashWidth; c++) {
				obstacleHash[r][c] = true;
			}
		}
	}
}
