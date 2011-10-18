#include "listener.h"
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void laserCallback(const sensor_msgs::LaserScan msg)
{
ROS_INFO("laser");
  /*
  ROS_INFO("I heard: [%f]", msg.ranges[100]);
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<project::AddTwoInts>("add_two_ints");
  project::AddTwoInts srv;
  srv.request.a = 1;
  srv.request.b = 2;
  if (client.call(srv))
  {
    ROS_INFO("Sum: %ld", (long int)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
  }*/
  //clipper::Polygons subj(1), clip(1), solution;
  for(unsigned int i = 0; i<msg.ranges.size();i++){
  	//clip[0].push_back(clipper::IntPoint(100,100,true,true));
	//ROS_INFO(msg.ranges[i]);
  }
}

void odomCallback(const nav_msgs::Odometry msg)
{
ROS_INFO("odometry");
//ROS_INFO((int)msg.pose.pose.position.x);  
//pos.x = msg.pose.pose.position.x;
}


int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub_laser = n.subscribe("base_scan", 1, laserCallback);
  ros::Subscriber sub_odom = n.subscribe("odom", 1, odomCallback);
  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  project::Robot explorer();
  
  ros::Rate r(1); // 10 hz
  while (ros::ok())
  {
 	 ROS_INFO("GO");	
	 ros::spinOnce();
	 r.sleep();
	}
  
  //ros::spin();

  return 0;
}

