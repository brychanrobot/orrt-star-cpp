#include "Rect.hpp"

Rect::Rect(Coord tl, Coord br) {
	double minx, maxx, miny, maxy;
	if (tl.x() < br.x()) {
		minx = tl.x();
		maxx = br.x();
	} else {
		minx = br.x();
		maxx = tl.x();
	}

	if (br.y() < tl.y()) {
		miny = tl.y();
		maxy = br.y();
	} else {
		miny = br.y();
		maxy = tl.y();
	}

	this->topLeft = Coord(minx, miny);
	this->bottomRight = Coord(maxx, maxy);
}

void Rect::inflate(double dx, double dy) {
	this->topLeft = Coord(this->topLeft.x() - dx, this->topLeft.y() - dy);

	this->bottomRight = Coord(this->bottomRight.x() + dx, this->bottomRight.y() + dy);
}

bool Rect::contains(Coord point) {
	return point.x() < this->bottomRight.x() && point.x() > this->topLeft.x() && point.y() < this->bottomRight.y() && point.y() > this->topLeft.y();
}

bool valueInRange(double value, double min, double max) { return (value >= min) && (value <= max); }

bool Rect::intersects(Rect& rect) {
	bool xOverlap = valueInRange(this->topLeft.x(), rect.topLeft.x(), rect.bottomRight.x()) ||
	                valueInRange(rect.topLeft.x(), this->topLeft.x(), this->bottomRight.x());

	bool yOverlap = valueInRange(this->topLeft.y(), rect.bottomRight.y(), rect.topLeft.y()) ||
	                valueInRange(rect.topLeft.y(), this->bottomRight.y(), this->topLeft.y());

	return xOverlap && yOverlap;
}

double Rect::width() { return this->bottomRight.x() - this->topLeft.x(); }

double Rect::height() { return this->topLeft.y() - this->bottomRight.y(); }
