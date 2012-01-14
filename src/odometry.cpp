/*
* File name: odometry.cpp
* Date:      2011/11/3
* Author:    Tomas Juchelka
*/

#include "odometry.h"
#include "project/GetOdometry.h"

float X = 0.0;
float Y = 0.0;
float orientation = 0.0;

void odom_callback(const nav_msgs::Odometry msg)
{
	float z = msg.pose.pose.orientation.z;
	float w = msg.pose.pose.orientation.w;
	X = msg.pose.pose.position.x*CM;
	Y = msg.pose.pose.position.y*CM;
	//atan2(2*(y*x+w*z),w*w+x*x-y*y-z*z);
	orientation = atan2(2*w*z,w*w-z*z);
}

bool odom_srv(project::GetOdometry::Request  &req, project::GetOdometry::Response &res)
{
  res.x = X;
  res.y = Y;
  res.angle = orientation;
  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometry");
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("get_robot_state", odom_srv);
  ros::Subscriber sub = n.subscribe("odom", 1, odom_callback);
  ROS_INFO("Ready to return odometry.");
  
  ros::Rate r(10); // hz
  while (ros::ok())
  {
	ros::spinOnce();
	r.sleep();
	}

  return 0;
}

// vi:ai:sw=4 ts=4 sts=0
