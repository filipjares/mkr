#ifndef TYPES_H
#define TYPES_H

struct SPosition {
	double x;
	double y;
	double yaw;
	
	SPosition() {}
	SPosition(double x, double y) : x(x), y(y) {}
	SPosition(double x, double y, double yaw) : x(x), y(y), yaw(yaw) {}
	
	SPosition& operator=(const SPosition& pos) {
	   if (this != &pos) {
	      x = pos.x;
	      y = pos.y;
	      yaw = pos.yaw;
	   }
	   return *this;
	}
};

#endif

// vi:ai:sw=4 ts=4 sts=0
