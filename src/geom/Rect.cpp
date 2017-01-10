#include "Rect.hpp"

using namespace std;

Rect::Rect(Coord tl, Coord br) : Rect(tl.x(), tl.y(), br.x(), br.y()) {}

Rect::Rect(double tlx, double tly, double brx, double bry) {
	double minx, maxx, miny, maxy;
	if (tlx < brx) {
		minx = tlx;
		maxx = brx;
	} else {
		minx = brx;
		maxx = tlx;
	}

	if (tly < bry) {
		miny = tly;
		maxy = bry;
	} else {
		miny = bry;
		maxy = tly;
	}

	this->topLeft.change(minx, miny);
	this->bottomRight.change(maxx, maxy);
}

void Rect::inflate(double dx, double dy) {
	this->topLeft.change(this->topLeft.x() - dx, this->topLeft.y() - dy);

	this->bottomRight.change(this->bottomRight.x() + dx, this->bottomRight.y() + dy);
}

bool Rect::contains(Coord point) {
	return point.x() < this->bottomRight.x() && point.x() > this->topLeft.x() && point.y() < this->bottomRight.y() && point.y() > this->topLeft.y();
}

bool valueInRange(double value, double min, double max) { return (value >= min) && (value <= max); }

bool Rect::intersects(Rect& rect) {
	bool xOverlap = valueInRange(this->topLeft.x(), rect.topLeft.x(), rect.bottomRight.x()) ||
	                valueInRange(rect.topLeft.x(), this->topLeft.x(), this->bottomRight.x());

	bool yOverlap = valueInRange(this->topLeft.y(), rect.topLeft.y(), rect.bottomRight.y()) ||
	                valueInRange(rect.topLeft.y(), this->topLeft.y(), this->bottomRight.y());

	return xOverlap && yOverlap;
}

double Rect::width() { return this->bottomRight.x() - this->topLeft.x(); }

double Rect::height() { return this->bottomRight.y() - this->topLeft.y(); }

void Rect::getPoints(vector<Coord>& points) {
	double pad = 1;

	points.push_back(Coord(this->topLeft.x() - pad, this->topLeft.y() - pad));
	points.push_back(Coord(this->bottomRight.x() + pad, this->bottomRight.y() + pad));

	points.push_back(Coord(this->topLeft.x() - pad, this->bottomRight.y() + pad));
	points.push_back(Coord(this->bottomRight.x() + pad, this->topLeft.y() - pad));
}
