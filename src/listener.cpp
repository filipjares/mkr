/*
* File name: listener.cpp
* Date:      2011/10/24
* Author:    Tomas Juchelka
*/

#include "listener.h"

double perpendicularDistance(clipper::IntPoint & first, clipper::IntPoint & last, clipper::IntPoint & furthest){
	double k = (last.Y-first.Y)/(last.X-first.X);
	return fabs(furthest.X-k*furthest.Y+k*first.X-first.Y)/sqrt(1+k*k);
}

/**
 * Ramer–Douglas–Peucker algorithm for reducing the number of points in a curve (polygon).
 */
void RDP(std::vector<clipper::IntPoint> & scan, const double epsilon, const int startIdx, const int endIdx){
	double d;
	double dmax = 0.0;
	int idx = 0;
	for (int i=startIdx+1; i<endIdx-1; i++) {
		d = perpendicularDistance(scan[startIdx], scan[endIdx], scan[i]);
		if(d > dmax) {
			dmax = d;
			idx = i;
		}
	}
}

void reducePolyPoints(std::vector<clipper::IntPoint> & scan, std::vector<clipper::IntPoint> & res){
	double dx,dy,k1,k2,dist;
	int idx = 0;
	for (unsigned int i=scan.size()-2; i>0; i--) {
   	dx = scan[i+1].X-scan[i].X;	   	
   	dy = scan[i+1].Y-scan[i].Y;
		k1 = dx/dy;
   	dx = scan[i].X-scan[i-1].X;	   	
   	dy = scan[i].Y-scan[i-1].Y;
		if(dy < 0.01)
			k2 = 0.0;
		k2 = dx/dy;
		dist = sqrt((scan[i+1].X-scan[i].X)*(scan[i+1].X-scan[i].X)+(scan[i+1].Y-scan[i].Y)*(scan[i+1].Y-scan[i].Y));
		if((fabs(k1-k2)<0.2 && dist < 100) || dist < 10)
			idx++;
		else
			res.push_back(scan[i]);
		//	scan.erase(i);
	}
	ROS_INFO("ERASED %d points",idx);
}

void writeData(const std::string fileName, std::vector<clipper::IntPoint> & solution, const unsigned int id) {
	using namespace std;
	char str[50];
	sprintf(str, "scans/scan_%d.dat", 1);
	//ofstream out (fileName.c_str());
	ofstream out (str);
	if (out.is_open()) {
		for (unsigned int i=0; i<solution.size(); i++) {
      	out << solution.at(i).X << " "
        	<< solution.at(i).Y << "\n";
		}
	out.close();
	} else cout << "Soubor se nepodarilo nacist...";
//	cout << "Output written succesfully." << endl;
}

void tfScanToPoints(const sensor_msgs::LaserScan msg, std::vector<clipper::IntPoint> & scan, double angle_min, double angle_max, const double angle_offset)
{
	if(msg.angle_min > angle_min)
		angle_min = msg.angle_min;
	if(msg.angle_max < angle_max)
		angle_max = msg.angle_max;

	/* Conversion to global angle 0-2PI
	 */
	double angleG;
	if(angle_offset>=ZERO) {
		angleG = angle_offset;
	}
	else {
		angleG = PI + PI + angle_offset;
	}

	//ROS_INFO("Parameters: %f %f %f %f %d", angle_min,angle_max,angle_offset,angleG,msg.ranges.size());
	double angle = msg.angle_min;
	double angleA;
	for(unsigned int i = 0; i<msg.ranges.size();i++){
  		if(angle < angle_min || angle > angle_max) {
			angle += msg.angle_increment;
			continue;
		}
		// in centimeters
		double r = msg.ranges[i]*100;
		angleA = -angleG + angle;
		// angle correction
		while(angleA > 2*PI)
			angleA -= 2*PI;
		while(angleA < ZERO)
			angleA += 2*PI;
		double x,y;
		if(angleA >= ZERO && angleA < PI/2) {
			y = -r*sin(angleA);
			x = r*cos(angleA);
		}
		else if(angleA >= PI/2 && angleA < PI) {	
			x = -r*sin(angleA-PI/2);
			y = -r*cos(angleA-PI/2);
		}
		else if(angleA >= PI && angleA < 3*PI/2) {
			x = -r*cos(angleA-PI);
			y = r*sin(angleA-PI);	
		}
		else {
			x = r*sin(angleA-3*PI/2);
			y = r*cos(angleA-3*PI/2);
		}
		scan.push_back(clipper::IntPoint(posX+x,posY+y,false,false));
		angle += msg.angle_increment;
	}
}

void laserCallback(const sensor_msgs::LaserScan msg)
{
	//ROS_INFO("laser");
	//ROS_INFO("angle between measurements: %f", msg.angle_increment);
	//ROS_INFO("angle min: %f", msg.angle_min);
	//ROS_INFO("angle max: %f", msg.angle_max);
	
	std::vector<clipper::IntPoint> scan;
	//std::vector<clipper::IntPoint> res;
	tfScanToPoints(msg, scan, MIN_ANGLE, MAX_ANGLE, orientation);
	//reducePolyPoints(scan,res);
	ROS_INFO("Num points in polygon: %d", scan.size());
	unsigned int id = msg.header.seq;
	writeData("scan.txt",scan,id);
 	//ROS_INFO("%d %lld %lld", i, scan[i].X, scan[i].Y);
}

void odomCallback(const nav_msgs::Odometry msg)
{
	//ROS_INFO("odometry");
	float angle;
	float z = msg.pose.pose.orientation.z;
	float w = msg.pose.pose.orientation.w;
	posX = msg.pose.pose.position.x;
	posY = msg.pose.pose.position.y;
	//atan2(2*(y*x+w*z),w*w+x*x-y*y-z*z);
	angle = atan2(2*w*z,w*w-z*z);
	if(angle<=ZERO)
		angle = -angle;
	else
		angle = 2*PI-angle;
	ROS_INFO("angle: %f x: %f y: %f", angle,posX,posY);
	orientation = angle;
}

//struct State robot_state;
//robot_state.posX = 0.0;
//robot_state->Y = 0.0;
float orientation = 0.0;
float posX = 0.0;
float posY = 0.0;

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
  
  ros::Rate r(10); // hz
  while (ros::ok())
  {
 	 //ROS_INFO("GO");	
	 ros::spinOnce();
	 r.sleep();
	}
  
  //ros::spin();

  return 0;
}

