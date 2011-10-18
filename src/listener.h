#ifndef LISTENER_H
#define LISTENER_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "Map.h"
#include "clipper.hpp"

#define PI 3.14159265

namespace project
{

class Robot
{
public:
  Robot();
private:
  void laserCallback(const sensor_msgs::LaserScan msg);
  void odomCallback(const nav_msgs::Odometry msg);

};

}

#endif
