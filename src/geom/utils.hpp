#pragma once

#include <cmath>
#include <cstdlib>
#include "Coord.hpp"
#include "Coord3.hpp"

inline double angleBetweenCoords(Coord &c1, Coord &c2) { return atan2(c2.y() - c1.y(), c2.x() - c1.x()); }

inline double euclideanDistance(double x1, double y1, double x2, double y2) {
	double dx = x1 - x2;
	double dy = y1 - y2;

	return sqrt(dx * dx + dy * dy);
}

inline double euclideanDistance(Coord &c1, Coord &c2) { return euclideanDistance(c1.x(), c1.y(), c2.x(), c2.y()); }

inline Coord randomPoint(int dx, int dy) { return Coord(rand() % dx, rand() % dy); }

inline double clamp(double val, double lo, double hi) { return std::min(std::max(val, lo), hi); }
