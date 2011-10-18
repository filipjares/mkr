/*
* File name:	Map.h
* Date:		2011/15/10		
* Author	Tomas Juchelka
*/
#include <string>
#include <vector>
#include "clipper.hpp"

std::vector<clipper::IntPoint> map;

struct Position {
	long x;
	long y;
	long phi;
};
