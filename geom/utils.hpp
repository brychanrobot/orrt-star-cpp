#pragma once

#include <cmath>
#include <cstdlib>
#include "Coord.hpp"

inline double angleBetweenCoords(Coord &c1, Coord &c2) { return atan2(c2.y() - c1.y(), c2.x() - c1.y()); }

inline double euclideanDistance(Coord &c1, Coord &c2) {
	double dx = c1.x() - c2.x();
	double dy = c1.y() - c2.y();

	return sqrt(dx * dx + dy * dy);
}

inline Coord randomPoint(double dx, double dy) { return Coord(((double)rand() / RAND_MAX) * dx, ((double)rand() / RAND_MAX) * dy); }
