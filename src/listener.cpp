/*
* File name: listener.cpp
* Date:      2011/10/24
* Author:    Tomas Juchelka
*/

#include "listener.h"

class Listener
{
public:
ros::NodeHandle node_handle_;
ros::V_Subscriber subs_;
ros::ServiceClient odom_client;

Listener(const ros::NodeHandle& node_handle)
: node_handle_(node_handle)
{
init();
}

void checkMap(ClipperLib::Polygons & obj) {
	for (unsigned int i = 0; i < obj.size(); i++) {
		for (unsigned int j = 1; j < obj[i].size(); j++) {
			//mark frontiers with legth < that treshold
			if(euclideanDistance(obj[i][j].X,obj[i][j].Y,obj[i][j-1].X,obj[i][j-1].Y) < FRONTIER_RANGE*2/3 && obj[i][j-1].outputEdge && obj[i][j].inputEdge){
				obj[i][j-1].outputEdge = false;
				obj[i][j].inputEdge = false;
			}
		}
	}
}

void simplifyMap(ClipperLib::Polygons & obj) {
	for (unsigned int i = 0; i < obj.size(); i++) {
		for (unsigned int j = 1; j < obj[i].size()-1; j++) {
			//simplifying map
			if((obj[i][j-1].outputEdge && obj[i][j+1].inputEdge) || (!obj[i][j-1].outputEdge && !obj[i][j+1].inputEdge)) {
				if(perpendicularDistance(obj[i][j-1], obj[i][j+1], obj[i][j]) < EPSILON) {
					obj[i][j].intersectPt = true;
					j = j+1;
				}		
			}
		}
	}
}

bool lineIntersection(ClipperLib::IntPoint & p, const ClipperLib::IntPoint & p1, const ClipperLib::IntPoint & p2, const ClipperLib::IntPoint & p3, const ClipperLib::IntPoint & p4){
	double den = (p4.Y - p3.Y)*(p2.X - p1.X) - (p4.X - p3.X)*(p2.Y - p1.Y);
	// check if the lines are parallel
	if(NEAR_EQUAL(den,0.0))
		return false;
	double s = (p4.X - p3.X)*(p1.Y - p3.Y) - (p4.Y - p3.Y)*(p1.X - p3.X)/den;
	double t = (p2.X - p1.X)*(p1.Y - p3.Y) - (p2.Y - p1.Y)*(p1.X - p3.X)/den;
	// check if they are in the interval <0,1>
	// if so, the line segments intersect, otherwise lines intersect
	if(s >= 0 && s <= 1 && t >= 0 && t <= 1) {
		p.X = p1.X + s*(p2.X - p1.X);
		p.Y = p1.Y + s*(p2.Y - p1.Y);
		return true;
	} else
		return false;
}

void clipScan(ClipperLib::Polygons & scan_n, const ClipperLib::Polygons & scan, const ClipperLib::Polygons & map){
	for (unsigned int i = 0; i < scan[0].size(); i++) {
		
	}
}

/*
void getIntersectPoint(ClipperLib::IntPoint & p, const ClipperLib::IntPoint & p1, const ClipperLib::IntPoint & p2, const ClipperLib::IntPoint & p3, const ClipperLib::IntPoint & p4){
	p.X = ((p1.X*p2.Y - p1.Y*p2.X)*(p3.X - p4.X) - (p1.X - p2.X)*(p3.X*p4.Y - p3.Y*p4.X))/((p1.X - p2.X)*(p3.Y - p4.Y) - (p1.Y - p2.Y)*(p3.X - p4.X));
	p.Y = ((p1.X*p2.Y - p1.Y*p2.X)*(p3.Y - p4.Y) - (p1.Y - p2.Y)*(p3.X*p4.Y - p3.Y*p4.X))/((p1.X - p2.X)*(p3.Y - p4.Y) - (p1.Y - p2.Y)*(p3.X - p4.X));
}
*/
private:
float orientation;
float posX;
float posY;

void init() {
	subs_.push_back(node_handle_.subscribe("base_scan", 1, &Listener::laserCallback,this));
   odom_client = node_handle_.serviceClient<project::GetOdometry>("get_robot_state");
  // subs_.push_back(node_handle_.subscribe("odom", 1, &Listener::odomCallback,this));
}

double perpendicularDistance(ClipperLib::IntPoint & first, ClipperLib::IntPoint & last, ClipperLib::IntPoint & furthest){
	double dx = last.X-first.X;
	double dy = last.Y-first.Y;
	return fabs(-dy*furthest.X+dx*furthest.Y+dy*first.X-dx*first.Y)/sqrt(dy*dy+dx*dx);
}

double euclideanDistance(const double x1, const double y1, const double x2, const double y2){
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

/**
 * Ramer–Douglas–Peucker algorithm for reducing the number of points in a curve (polygon).
 */
void RDP(std::vector<ClipperLib::IntPoint> & scan, const double epsilon, const int startIdx, const int endIdx){
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
	if(dmax >= epsilon){
		// adding point to the output by using now unused variable
		scan[idx].intersectPt = true;
		RDP(scan, epsilon, startIdx, idx);
		RDP(scan, epsilon, idx, endIdx);
	}
}

void tfScanToPoints(const sensor_msgs::LaserScan msg, std::vector<ClipperLib::IntPoint> & scan, double angle_min, double angle_max, const double angle_offset)
{
	if(msg.angle_min > angle_min)
		angle_min = msg.angle_min;
	if(msg.angle_max < angle_max)
		angle_max = msg.angle_max;
	double angle = msg.angle_max;
	double x,y;
	for(int i = msg.ranges.size()-1; i>=0;i--){
  		if(angle < angle_min || angle > angle_max) {
			angle -= msg.angle_increment;
			continue;
		}
		// in centimeters	
		double r = msg.ranges[i]*CM;
		if(msg.ranges[i] == LASER_RANGE)
			r-= MAX_RANGE_CORR;
		y = r*sin(angle+angle_offset);
		x = r*cos(angle+angle_offset);
		bool frontierIn = false;
		bool frontierOut = false;
		if(scan.size() > 0) {
			if(msg.ranges[i] == LASER_RANGE && msg.ranges[i+1] == LASER_RANGE) {
				frontierIn = true;
				scan[scan.size()-1].outputEdge = true;
			}
		}
		scan.push_back(ClipperLib::IntPoint(posX+x,posY+y,frontierIn,frontierOut));
		angle -= msg.angle_increment;
	}
}

void processScan(std::vector<ClipperLib::IntPoint> & scan)
{
	scan[0].intersectPt = true;
	scan[0].inputEdge = true;
	scan[scan.size()-1].intersectPt = true;
	scan[scan.size()-1].outputEdge = true;
	unsigned int idx = 0; 
	double x = posX;
	double y = posY;
	for(unsigned int i = 0; i<scan.size()-1;i++) {
		if(abs(euclideanDistance(scan[i].X,scan[i].Y,x,y) - euclideanDistance(scan[i+1].X,scan[i+1].Y,x,y)) > FRONTIER_RANGE) {
			RDP(scan, EPSILON, idx, i);
			scan[i].outputEdge = true;
			scan[i+1].inputEdge = true;
			scan[i].intersectPt = true;
			scan[i+1].intersectPt = true;
			idx = i+1;
			i++;
		}
		if(i == scan.size()-2)
			RDP(scan, EPSILON, idx, scan.size()-1);
	}
	//adding position of robot to polygon
	scan.push_back(ClipperLib::IntPoint(posX,posY,true,true));
	scan[scan.size()-1].intersectPt = true;
}

void selectFrontiers(std::vector<ClipperLib::IntPoint> & scan)
{
	for(unsigned int i = 0; i < scan.size(); i++) {
		if(scan[i].intersectPt) {
//			ROS_INFO("%lld %lld", scan[i].X,scan[i].Y);
			scan[i].intersectPt = false;
			clip[0].push_back(scan[i]);
		}
	}
	//CHECK scan for correct frontier pairs after deleting points
	for(unsigned int i = 1; i<clip[0].size()-1;i++) {
		if(CLIP(i-1).outputEdge){
			if(!CLIP(i).inputEdge)
				CLIP(i-1).outputEdge = false;
		}
	}	
/*	ROS_INFO("new scan");
	ROS_INFO("robot position %f %f",posX,posY);
	for(unsigned int i = 0; i<clip[0].size();i++) {
		ROS_INFO("%lld %lld %d %d", CLIP(i).X,CLIP(i).Y,CLIP(i).inputEdge,CLIP(i).outputEdge);
	}*/	
}

void laserCallback(const sensor_msgs::LaserScan msg)
{
	project::GetOdometry srv;
	if(odom_client.call(srv)){
		posX = srv.response.x;
		posY = srv.response.y;
		orientation = srv.response.angle;
	}else
		ROS_ERROR("Failed to call service get_robot_state. Position is not actual!");
	std::vector<ClipperLib::IntPoint> scan;
	tfScanToPoints(msg, scan, MIN_ANGLE, MAX_ANGLE, orientation);
	processScan(scan);
	selectFrontiers(scan);
}
};

void visualizeScan(ros::Publisher & marker_pub, ClipperLib::Polygons & obj) {
   visualization_msgs::Marker line_strip;
	line_strip.header.frame_id = "/odom";
	line_strip.ns = "scan";
	line_strip.action = visualization_msgs::Marker::ADD;
	line_strip.pose.orientation.w = 1.0;
	line_strip.id = 0;
	line_strip.type = visualization_msgs::Marker::LINE_STRIP;
	line_strip.scale.x = 0.1;
	line_strip.color.g = 1.0f;
	line_strip.color.a = 1.0;
	//adding point only for vizualization
	unsigned int size = obj[0].size();
	geometry_msgs::Point p;
	p.x = obj[0][size-1].X/CM;
	p.y = obj[0][size-1].Y/CM;
	line_strip.points.push_back(p);
	for(unsigned int i = 0; i<size;i++){
		p.x = obj[0][i].X/CM;
		p.y = obj[0][i].Y/CM;
		line_strip.points.push_back(p);
	}
	marker_pub.publish(line_strip);
}

void visualizeMap(ros::Publisher & marker_pub, ClipperLib::Polygons & obj, const int id) {
   visualization_msgs::Marker line_strip, line_list, text;
	line_strip.header.frame_id = "/odom";
//	line_strip.header.stamp = ros::Time::now();
	line_strip.ns = "lines";
	line_strip.lifetime = ros::Duration(5);
	line_strip.action = visualization_msgs::Marker::ADD;
	line_strip.pose.orientation.w = 1.0;
	line_strip.id = id;
	line_strip.type = visualization_msgs::Marker::LINE_LIST;
	line_strip.scale.x = 0.5;
	line_strip.color.r = 1.0;
	line_strip.color.a = 1.0;
	geometry_msgs::Point p1;
	geometry_msgs::Point p2;
	for(unsigned int i = 0; i<obj[id].size()-1;i++) {
		p1.x = obj[id][i].X/CM;
		p1.y = obj[id][i].Y/CM;
		p2.x = obj[id][i+1].X/CM;
		p2.y = obj[id][i+1].Y/CM;
		if(obj[id][i].outputEdge && obj[id][i+1].inputEdge) {
			line_list.points.push_back(p1);
			line_list.points.push_back(p2);
		}
		else {
			line_strip.points.push_back(p1);
			line_strip.points.push_back(p2);
		}
	}
	//adding point to join last point with robot pose
		p1.x = obj[id][obj[id].size()-1].X/CM;
		p1.y = obj[id][obj[id].size()-1].Y/CM;
		p2.x = obj[id][0].X/CM;
		p2.y = obj[id][0].Y/CM;
		if(obj[id][obj[id].size()-1].outputEdge && obj[id][0].inputEdge) {
			line_list.points.push_back(p1);
			line_list.points.push_back(p2);
		}
		else {
			line_strip.points.push_back(p1);
			line_strip.points.push_back(p2);
		}
	line_list.header.frame_id = "/odom";
	line_list.ns = "frontiers";
	line_list.lifetime = ros::Duration(5);
	line_list.action = visualization_msgs::Marker::ADD;
	line_list.pose.orientation.w = 1.0;
	line_list.id = id;
	line_list.type = visualization_msgs::Marker::LINE_LIST;
	line_list.scale.x = 0.5;
	line_list.color.b = 1.0f;
	line_list.color.a = 1.0;
	
	marker_pub.publish(line_strip);
	marker_pub.publish(line_list);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  Listener l(n);

  ros::Rate r(10); // hz
  /*subj[0].push_back(ClipperLib::IntPoint(-10,-3,true,true));
  subj[0].push_back(ClipperLib::IntPoint(10,-3,true,true));
  subj[0].push_back(ClipperLib::IntPoint(10,3,true,true));
  subj[0].push_back(ClipperLib::IntPoint(-10,3,true,true));
  */
/*
	ClipperLib::IntPoint p1;
	p1.X = 0;
	p1.Y = 0;
	ClipperLib::IntPoint p2;
	p2.X = 0;
	p2.Y = 100000;
	ClipperLib::IntPoint p3;
	p3.X = 10;
	p3.Y = 0;
	ClipperLib::IntPoint p4;
	p4.X = -10;
	p4.Y = 150000;
	ClipperLib::IntPoint p;
	
	bool intersect = l.lineIntersection(p,p1,p2,p3,p4);
	ROS_INFO("intersect: %d point: %lld %lld", intersect, p.X, p.Y);

	return 0;
*/
  //int size = 0;
  while (ros::ok())
  {
//		l.clipScan(clip_tmp, clip, subj);
//		clip = clip_tmp;
//		clip_tmp[0].clear();
		ClipperLib::Clipper c;
		c.AddPolygons(subj, ClipperLib::ptSubject);
		c.AddPolygons(clip, ClipperLib::ptClip);
		c.Execute(ClipperLib::ctUnion, solution, ClipperLib::pftNonZero);
		for(unsigned int i = 0;i<solution.size();i++){
			visualizeMap(marker_pub,solution,i);
		}
	//	if(clip[0].size()>0)
	//	visualizeMap(marker_pub,clip,0);
	//	size = 0;
	//	for(unsigned int i = 0;i<solution.size();i++){
	//		size += solution[i].size();
	//	}
	
	//	ROS_INFO("size: %d",size);
		clip[0].clear();
		l.checkMap(solution);
	// cant be done so
	//	l.simplifyMap(solution);
		subj = solution;
	ros::spinOnce();
	r.sleep();
	}

  return 0;
}

