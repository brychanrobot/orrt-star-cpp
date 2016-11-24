#pragma once

#include <cmath>
#include <cstdlib>
#include "Coord.hpp"
#include "Coord3.hpp"

inline double angleBetweenCoords(Coord &c1, Coord &c2) { return atan2(c2.y() - c1.y(), c2.x() - c1.x()); }

inline double euclideanDistance(Coord &c1, Coord &c2) {
	double dx = c1.x() - c2.x();
	double dy = c1.y() - c2.y();

	return sqrt(dx * dx + dy * dy);
}

inline Coord randomPoint(int dx, int dy) { return Coord(rand() % dx, rand() % dy); }

inline double clamp(double val, double lo, double hi) { return std::min(std::max(val, lo), hi); }
