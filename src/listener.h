#ifndef LISTENER_H
#define LISTENER_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include "project/GetOdometry.h"
#include "clipper.hpp"
#include <vector>
#include <fstream>
#include <string>
#include <cstring>
#include <cmath>
#include <iostream>
#include "consts.h"
#include "types.h"

#define CLIP(i) clip[0][i]
#define OUTER_IDX	0	//simplification because of we have only one starshape polygon with holes

#define TOLERANCE (1.0e-4)
#define NEAR_ZERO(val) (((val) > -TOLERANCE) && ((val) < TOLERANCE))
#define NEAR_EQUAL(a, b) NEAR_ZERO((a) - (b))

template<typename T>
std::string toString(T val)
{
    std::stringstream ss("");
    ss << val;
    return ss.str();
}

ClipperLib::Polygons subj(1), clip(1), clip_tmp(1);
ClipperLib::ExPolygons solution;


#endif

// vi:ai:sw=4 ts=4 sts=0
