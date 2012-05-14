/*
* File name: listener.cpp
* Date:      2011/10/24
* Author:    Tomas Juchelka
*/

#include "poly2vd.hpp" // has to be included first because of the ZERO macro
#include "listener.h"
#include "VdPublisher.hpp"	// because of VdPublisher::RVIZ_EDGES_WIDTH

#include <errno.h>
#include <signal.h>
#include <termios.h>
#include <fcntl.h>

class Listener
{
public:
	ros::NodeHandle node_handle_;
	ros::V_Subscriber subs_;
	ros::ServiceClient odom_client;
	
	Listener(const ros::NodeHandle& node_handle) : node_handle_(node_handle)
	{
		init();
	}

	SPosition getPosition(void) const {
		return pos;
	}
		
	void eliminateCoincidentFrontiers(ClipperLib::Polygon & p) {
		//inline frontier
		int s = p.size();
		double maxDist = sqrt(2.0);
		for (int i = 1; i < s-2; i++) {
			if(!p[i].inputEdge && p[i].outputEdge && p[i+1].inputEdge && !p[i+1].outputEdge)	{
				//is isolated
				if(perpendicularDistance(p[i],p[i+1],p[i-1]) <= maxDist)	{
					p[i].outputEdge = false;
					p[i+1].inputEdge = false;
				}
				else if(perpendicularDistance(p[i],p[i+1],p[i+2]) <= maxDist)	{
					p[i].outputEdge = false;
					p[i+1].inputEdge = false;	
				}
			}
		}
	}
	
	void eliminateFalseFrontiers(ClipperLib::Polygon & p) {
		//mark frontiers with legth < that treshold
		double minSize = FRONTIER_RANGE;
	
		//first - find first non frontier point
		int k = 1;
		while(p[k-1].inputEdge){
			k++;
		}
				
		//second - find the frontier group
		int s = p.size();
		for (int i = k; i < s; i++) {
			if(!p[i].inputEdge)
				continue;
			double len = 0.0;
			int start = i-1;
			int end = start;
			while(p[end+1].inputEdge)	{
				end++;
				len += euclideanDistance(p[end].X,p[end].Y,p[end-1].X,p[end-1].Y); 
				if(end == s-1)
					break;
			}
	
		//third - if it spreads over the end, continue searching fron the beginning
			
			bool spreaded = (end == s-1 && p[0].inputEdge) ? true : false;
			if(spreaded) {
				end = 0;
				len += euclideanDistance(p[s-1].X,p[s-1].Y,p[0].X,p[0].Y); 
				while(p[end+1].inputEdge)	{
					end++;
					len += euclideanDistance(p[end].X,p[end].Y,p[end-1].X,p[end-1].Y); 
				}
				if(len < minSize) {
					p[start].outputEdge = false;
					p[end].inputEdge = false;
					for (int n = start+1; n < s; n++) {
						p[n].inputEdge = false;
						p[n].outputEdge = false;
					}
					for (int n = 0; n < end; n++) {
						p[n].inputEdge = false;
						p[n].outputEdge = false;
					}
				}	
				break;
			}
			else	{	
				if(len < minSize) {
					p[start].outputEdge = false;
					p[end].inputEdge = false;
					for (int n = start+1; n < end; n++) {
						p[n].inputEdge = false;
						p[n].outputEdge = false;
					}
				}	
				i = end + 1;
			}
		}
	}
	
	void checkMap(ClipperLib::ExPolygons & obj) {		
		for (unsigned int i = 0; i < obj.size(); i++) {
			eliminateFalseFrontiers(obj[i].outer);
			eliminateCoincidentFrontiers(obj[i].outer);
			for(unsigned j = 0; j < obj[i].holes.size(); j++) {
				eliminateFalseFrontiers(obj[i].holes[j]);
				eliminateCoincidentFrontiers(obj[i].holes[j]);
			}
		}
	}
	
	void simplifyMap(ClipperLib::ExPolygons & obj) {
		int count = 0;
		for (unsigned int i = 0; i < obj.size(); i++) {
		//outer polygon
		double lenA,lenB,lenAB;
		for (unsigned int j = 1; j < obj[i].outer.size()-1; j++) {
				if(obj[i].outer[j].inputEdge == obj[i].outer[j].outputEdge){
					lenA = euclideanDistance(obj[i].outer[j].X,obj[i].outer[j].Y,obj[i].outer[j-1].X,obj[i].outer[j-1].Y);
					lenB = euclideanDistance(obj[i].outer[j].X,obj[i].outer[j].Y,obj[i].outer[j+1].X,obj[i].outer[j+1].Y);
					lenAB = euclideanDistance(obj[i].outer[j+1].X,obj[i].outer[j+1].Y,obj[i].outer[j-1].X,obj[i].outer[j-1].Y);
					/* reasonable range of the divisor on the right side is between 10 (coarse polygons) and 10000 (fine polygons) */
					if(lenA + lenB - lenAB < 0.3) {
						obj[i].outer[j].intersectPt = true;	//mark for remove
						count++;	
					}
				}	
			}
		}
	}

private:
	SPosition pos;
	
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
	/*  		if(angle < angle_min || angle > angle_max) {
				angle -= msg.angle_increment;
				continue;
			}*/
			// in centimeters	
			double r = msg.ranges[i]*CM;
			if(abs(msg.ranges[i] - LASER_RANGE) < 0.1)
				r -= MAX_RANGE_CORR;
			y = r*sin(angle+angle_offset);
			x = r*cos(angle+angle_offset);
			bool frontierIn = false;
			bool frontierOut = false;
			if(scan.size() > 0) {
				if(abs(msg.ranges[i] - LASER_RANGE) < 0.1 && abs(msg.ranges[i+1] - LASER_RANGE) < 0.1) {
					frontierIn = true;
					scan[scan.size()-1].outputEdge = true;
				}
			}
			scan.push_back(ClipperLib::IntPoint(pos.x+x,pos.y+y,frontierIn,frontierOut));
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
		double x = pos.x;
		double y = pos.y;
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
		scan.push_back(ClipperLib::IntPoint(pos.x,pos.y,true,true));
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
		ROS_INFO("robot position %f %f",pos.x,pos.y);
		for(unsigned int i = 0; i<clip[0].size();i++) {
			ROS_INFO("%lld %lld %d %d", CLIP(i).X,CLIP(i).Y,CLIP(i).inputEdge,CLIP(i).outputEdge);
		}*/	
	}
	
	void laserCallback(const sensor_msgs::LaserScan msg)
	{
		project::GetOdometry srv;
		if(odom_client.call(srv)){
			pos.x = srv.response.x;
			pos.y = srv.response.y;
			pos.yaw = srv.response.angle;
		}else
			ROS_ERROR("Failed to call service get_robot_state. Position is not actual!");
		std::vector<ClipperLib::IntPoint> scan;
		tfScanToPoints(msg, scan, MIN_ANGLE, MAX_ANGLE, pos.yaw);
		processScan(scan);
		selectFrontiers(scan);
	}
};

void visualizeMap(ros::Publisher & marker_pub, ClipperLib::Polygon & obj, const int id) {
	visualization_msgs::Marker line_strip, line_list, text;
	line_strip.header.frame_id = "/odom";
//	line_strip.header.stamp = ros::Time::now();
	line_strip.ns = "lines";
	line_strip.lifetime = ros::Duration(5);
	line_strip.action = visualization_msgs::Marker::ADD;
	line_strip.pose.orientation.w = 1.0;
	line_strip.id = id;
	line_strip.type = visualization_msgs::Marker::LINE_LIST;
	line_strip.scale.x = poly2vd::VdPublisher::RVIZ_EDGES_WIDTH;
	line_strip.color.r = 1.0;
	line_strip.color.a = 1.0;
	geometry_msgs::Point p1;
	geometry_msgs::Point p2;
	for(unsigned int i = 0; i<obj.size()-1;i++) {
		p1.x = obj[i].X/CM;
		p1.y = obj[i].Y/CM;
		p2.x = obj[i+1].X/CM;
		p2.y = obj[i+1].Y/CM;
		if(obj[i].outputEdge && obj[i+1].inputEdge) {
			line_list.points.push_back(p1);
			line_list.points.push_back(p2);
		}
		else {
			line_strip.points.push_back(p1);
			line_strip.points.push_back(p2);
		}
	}
	//adding point to join last point with robot pose
	p1.x = obj[obj.size()-1].X/CM;
	p1.y = obj[obj.size()-1].Y/CM;
	p2.x = obj[0].X/CM;
	p2.y = obj[0].Y/CM;
	if(obj[obj.size()-1].outputEdge && obj[0].inputEdge) {
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
	line_list.scale.x = poly2vd::VdPublisher::RVIZ_EDGES_WIDTH;
	line_list.color.b = 1.0f;
	line_list.color.a = 1.0;
	
	marker_pub.publish(line_strip);
	marker_pub.publish(line_list);
}

void convertPolyInCentimeters2SegsInMeters(ClipperLib::ExPolygons & poly, in_segs * s, unsigned int size) {
	in_segs init;
	init.x1 = 0.0;
	init.x2 = 0.0;
	init.y1 = 0.0;
	init.y2 = 0.0;
	for (unsigned int i = 0; i < size; i++)
		s[i] = init;
	unsigned int k = 0;
	for(unsigned int n = 0; n < poly.size(); n++)	{
		unsigned int sz = poly[n].outer.size();
		for(unsigned int i = 0; i < sz - 1; i++)	{
			// endpoint coords
			s[k].x1 = (double)poly[n].outer[i].X/CM;
			s[k].y1 = (double)poly[n].outer[i].Y/CM;
			s[k].x2 = (double)poly[n].outer[i+1].X/CM;
			s[k].y2 = (double)poly[n].outer[i+1].Y/CM;
			// is it a frontier
			s[k].ext_appl.isFrontier = (poly[n].outer[i].outputEdge && poly[n].outer[i+1].inputEdge);
			s[k].ext_appl.isOuter = true;
			k++;
		}
		// endpoint coords
		s[k].x1 = (double)poly[n].outer[sz-1].X/CM;
		s[k].y1 = (double)poly[n].outer[sz-1].Y/CM;
		s[k].x2 = (double)poly[n].outer[0].X/CM;
		s[k].y2 = (double)poly[n].outer[0].Y/CM;
		// is it a frontier
		s[k].ext_appl.isFrontier = (poly[n].outer[sz-1].outputEdge && poly[n].outer[0].inputEdge);
		s[k].ext_appl.isOuter = true;
		k++;

		for(unsigned int m = 0; m < poly[n].holes.size(); m++)	{	
			unsigned int sz = poly[n].holes[m].size();
			for(unsigned int i = 0; i < sz - 1; i++)	{
				// endpoint coords
				s[k].x1 = (double)poly[n].holes[m][i].X/CM;
				s[k].y1 = (double)poly[n].holes[m][i].Y/CM;
				s[k].x2 = (double)poly[n].holes[m][i+1].X/CM;
				s[k].y2 = (double)poly[n].holes[m][i+1].Y/CM;
				// is it a frontier
				s[k].ext_appl.isFrontier = (poly[n].holes[m][i].outputEdge && poly[n].holes[m][i+1].inputEdge);
				s[k].ext_appl.isOuter = false;
				k++;
			}
			// endpoint coords
			s[k].x1 = (double)poly[n].holes[m][sz-1].X/CM;
			s[k].y1 = (double)poly[n].holes[m][sz-1].Y/CM;
			s[k].x2 = (double)poly[n].holes[m][0].X/CM;
			s[k].y2 = (double)poly[n].holes[m][0].Y/CM;
			// is it a frontier
			s[k].ext_appl.isFrontier = (poly[n].holes[m][sz-1].outputEdge && poly[n].holes[m][0].inputEdge);
			s[k].ext_appl.isOuter = false;
			k++;
		}
	}	
}

/* ************* Terminal handling and DOT export ******************** */

int kfd = 0; // stdin
struct termios cooked, raw;

void terminal_setup()
{
	// get the console in raw mode                                                              
	tcgetattr(kfd, &cooked);
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &=~ (ICANON | ECHO);
	// Setting a new line, then end of file                         
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	tcsetattr(kfd, TCSANOW, &raw);
	// want non-blocking input; FIXME: check efficiency of this solution
	int flags = fcntl(kfd, F_GETFL);
	flags |= O_NONBLOCK;
	fcntl(kfd, F_SETFL, flags);
}

void terminal_cleanup(int sig)
{
	tcsetattr(kfd, TCSANOW, &cooked);
	ros::shutdown();
}

void quit(int status)
{
	terminal_cleanup(0);
	exit(status);
}

#define EXPORT_KEY_LOWERCASE		'e'
#define EXPORT_KEY_UPPERCASE		'E'
#define DOT_FILE_PATH_AND_PREFIX	"/tmp/vd-"
#define DOT_FILE_SUFFIX				".dot"

/* If EXPORT_KEY_* was pressed, this function performs the export
 * of resulting Voronoi diagram into DOT file */
void handleDotExport(poly2vd::Poly2VdConverter &poly2vd)
{
	char ch;
	// FIXME: check efficiency of this solution
	// (the call to read and checking for error)
	if (read(kfd, &ch, 1) < 0) {
		if (errno == EAGAIN) {
			ch = 0;
		} else {
			perror("read():");
			quit(-1);
		}
	}

	static int exportCounter = 0;
	if (ch == EXPORT_KEY_LOWERCASE || ch == EXPORT_KEY_UPPERCASE) {
		std::stringstream fileName;
		fileName << DOT_FILE_PATH_AND_PREFIX << std::setfill('0') << std::setw(2) << std::right
			<< exportCounter++ << DOT_FILE_SUFFIX;
		poly2vd.exportVdToDot(fileName.str(), false, false);
		std::cout << "DOT description of the Voronoi diagram saved to"
			<< fileName.str() << std::endl;
	}
}

/* **************************** main() ******************************* */

int main(int argc, char **argv) {
	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	Listener l(n);

	// Setup the terminal in order to be able to handle keypresses
	terminal_setup();
	signal(SIGINT,terminal_cleanup);
	// Let the user know of the exportToDot feature
	std::cout << "Pressing '" << EXPORT_KEY_LOWERCASE << "' produces DOT files "
		<< DOT_FILE_PATH_AND_PREFIX << "XX" << DOT_FILE_SUFFIX << " for graphviz.  "
		<< "This is however " << std::endl << "an experimental feature and resulting files are "
		<< "too big to be processed by graphviz." << std::endl << std::endl;

	poly2vd::Poly2VdConverter poly2vd;	// Vroni based polygons -> VD converter
	poly2vd.usePublisher(&marker_pub, "/odom", 2.5);
	
	ros::Rate r(0.5); // hz
	while (ros::ok()) {
//		l.clipScan(clip_tmp, clip, subj);
//		clip = clip_tmp;
//		clip_tmp[0].clear();
		ClipperLib::Clipper c;
		c.AddPolygons(subj, ClipperLib::ptSubject);
		c.AddPolygons(clip, ClipperLib::ptClip);
		c.Execute(ClipperLib::ctUnion, solution, ClipperLib::pftNonZero, ClipperLib::pftNonZero, ClipperLib::pftOn);
		clip[0].clear();
		unsigned int idx = 0;
		for(unsigned int i = 0;i<solution.size();i++){
			visualizeMap(marker_pub,solution[i].outer,++idx);
			for (unsigned int j = 0; j < solution[i].holes.size(); j++) {
				visualizeMap(marker_pub,solution[i].holes[j],++idx);
			}	
		}

		// Compute and publish VD / WMAT
		unsigned int size = 0; // polygonal segments count
		for(unsigned int i = 0; i < solution.size(); i++) {
			size += solution[i].outer.size();
			for (unsigned int j = 0; j < solution[i].holes.size(); j++) {
				size += solution[i].holes[j].size();
			}	
		}

		if (size > 3) {
			// convert polygons to segments
			in_segs segs[size];
			convertPolyInCentimeters2SegsInMeters(solution, segs, size);
			// pass segments to Vroni
			poly2vd.prepareNewInput(segs, size);
			// let the Vroni compute the VD
			poly2vd.convert();
			// publish results
			SPosition p = l.getPosition();
			coord start;
			start.x = p.x / CM;
			start.y = p.y / CM;
			// poly2vd.publish_root(marker_pub, start, "/odom", 2.0);
			poly2vd.findCriticalNodes(start);
			poly2vd.publishResults();
		}

		// if key pressed, export Voronoi diagram to DOT file in /tmp/
		handleDotExport(poly2vd);

		l.checkMap(solution);
		l.simplifyMap(solution);

		subj.clear();
		for (unsigned int i = 0; i < solution.size(); i++) {
			subj.push_back(solution[i].outer);
			//just for test
			subj[i].clear();
			for (unsigned int j = 0; j < solution[i].outer.size(); j++) {
				if(!solution[i].outer[j].intersectPt)
				subj[i].push_back(solution[i].outer[j]);
			}	
			for (unsigned int j = 0; j < solution[i].holes.size(); j++) {
				subj.push_back(solution[i].holes[j]);
			}	
		}
		
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}

// vi:ai:sw=4 ts=4 sts=0
