#ifndef LISTENER_H
#define LISTENER_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "clipper.hpp"
#include <vector>
#include <fstream>
#include <string>
#include <cstring>
#include <cmath>

#define PI 3.14159265
#define ZERO 0.0
#define MIN_ANGLE -2.0
#define MAX_ANGLE 2.0

//struct State {
//  double posX;
//  double posY;
extern float orientation;
extern float posX;
extern float posY;
//};

extern struct State robot_state;

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
