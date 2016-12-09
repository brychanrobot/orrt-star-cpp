#pragma once

#include <stdlib.h>
#include <cmath>
// #include <math.h>
#include "../geom/Rect.hpp"
#include "../geom/utils.hpp"

bool hasIntersection(Rect& rect, std::vector<std::shared_ptr<Rect>>& obstacles) {
	for (const auto& obstacle : obstacles) {
		if (obstacle->intersects(rect)) {
			return true;
		}
	}

	return false;
}

void generateObstacleRects(int width, int height, int count, std::vector<std::shared_ptr<Rect>>& obstacles, double padding = 5) {
	for (int x = 0; x < count; x++) {
		std::shared_ptr<Rect> rect;
		while (true) {
			auto topLeft = randomPoint(width, height);
			auto bottomRight = randomPoint(width, height);

			// delete rect;
			rect = std::make_shared<Rect>(topLeft, bottomRight);
			if (rect->width() > padding * 2 && rect->height() > padding * 2 && rect->width() < 0.5 * width && rect->height() < 0.5 * height &&
			    !hasIntersection(*rect, obstacles)) {
				break;
			}
		}

		obstacles.push_back(rect);
	}
}

void generateObstacleHash(std::vector<std::shared_ptr<Rect>>& obstacleRects, std::vector<std::vector<bool>>& obstacleHash) {
	for (const auto& obstacle : obstacleRects) {
		for (int r = obstacle->topLeft.y(); r < obstacle->bottomRight.y(); r++) {
			for (int c = obstacle->topLeft.x(); c < obstacle->bottomRight.x(); c++) {
				obstacleHash[r][c] = true;
			}
		}
	}
}
