/*
 * ===========================================================================
 *
 *       Filename:  poly2vd.cpp
 *
 *    Description:  Convenient interface to VRONI
 *
 *        Version:  1.0
 *        Created:  01/05/2012 07:12:55 PM
 *       Compiler:  gcc
 *
 *        Authors:  Filip Jares (fj), filipjares@gmail.com
 *                  Tomas Juchelka (tj), tomas.juchelka@gmail.com
 *
 * ===========================================================================
 */
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <string>
#include <list>
#include <algorithm>
#include <assert.h>
#include "poly2vd.hpp"

/* ************ ROS includes (other than in poly2vd.hpp) ************* */

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// These macros are defined in Vroni's basic.h; but I had problems including it

#define ScaleX(xc)  (scale_factor * ((xc) - shift.x))

#define ScaleY(yc)  (scale_factor * ((yc) - shift.y))

#define ScaleV(value)  ((value) * scale_factor)

#define UnscaleX(xc)  (assert(scale_factor > 0.0), (xc) / scale_factor + shift.x)

#define UnscaleY(yc)  (assert(scale_factor > 0.0), (yc) / scale_factor + shift.y)

#define UnscaleV(value)  (assert(scale_factor > 0.0), (value) / scale_factor)

/* ********************** Constructor and destructor ***************** */

Poly2VdConverter::Poly2VdConverter()
{
	API_InitializeProgram();
	input_prepared = false;
}

Poly2VdConverter::~Poly2VdConverter()
{
	API_TerminateProgram();
}

/* ********************** Public methods ***************************** */

void Poly2VdConverter::prepareNewInput(in_segs * segs, unsigned int size)
{
	// without this call to API_ResetAll, it should be possible to to
	// call this method several times to add several sets of segments
	API_ResetAll();				// reset Vronis' internal data
	
	boolean new_input;
	API_ArrayInput(
		0,NULL,					/*  no points */
		size, segs,
		0,NULL, &new_input);	/*  no arcs */

	// new_input == false means nothing but no input data

	// now, client code can call convert();
	input_prepared = true;
}

void Poly2VdConverter::prepareNewInput(char * const inputFileName)
{
	API_ResetAll();

	boolean input_ok;
	API_FileInput(inputFileName, &input_ok);

	// Vroni calls exit() on error, nothing to check here :-(
	input_prepared = true;
}

void Poly2VdConverter::convert()
{
	assert(input_prepared);

	char emptyString[] = "";
	API_ComputeVD(
		false,    /* save input data to file?     */
		true,     /* first call for this data?    */
		false,    /* don't measure time           */
		3,        /* scale factor for bounding box; default: 1 */
		0,        /* sampling factor              */
		0,        /* approximation factor for circular arcs */
		emptyString,       /* name of the output file;     */
		false,    /* check for duplicate segs prior to the computation?     */
		false,    /* compute an approximate VD for circular arcs and        */
		          /*  use it for subsequent operations (such as offsetting) */
		0.0,      /* approximation threshold for  */
		          /* circular arcs; see           */
		          /* see ApproxArcsBounded() in   */
		          /* in approx.cc; default = 0.0  */
		0.0,      /* approximation threshold for  */
		          /* circular arcs; see           */
		          /* see ApproxArcsBounded() in   */
		          /* in approx.cc; default = 0.0  */
		false,    /* shall we use my heuristic    */
		          /* approximation threshold?     */
		false,    /* compute VD/DT of points only */
		false,    /* output point VD/DT           */
		false,    /* output file for point VD/DT  */
		false);   /* FIXME: use true here? shall we clean up
		             the data prior to the VD computation?
		             I suppose it will be needed, poly2vd() will
		             be called repeatedly; do some examination */

	API_ComputeWMAT(
		false,    /* shall we use my heuristic    */
		          /* for finding nice WMAT        */
		          /* thresholds?                  */
		0.0,      /* angle threshold for WMAT     */
		          /* computation;in radians, out  */
		          /* of the interval [0, pi]      */
		0.0,      /* distance threshold for WMAT  */
		          /* computation                  */
		false,    /* do you want to time the      */
		          /* computation?                 */
		false,    /* true if WMAT is to be        */
		          /* computed only on the left    */
		          /* side of input segments       */
		false);   /* true if WMAT is to be        */
		          /* computed only on the right   */
		          /* side of input segments       */

	// This would output WMAT data as produced by Vroni's "--ma" parameter
	// char o_file[] = "/tmp/my_ma_output.txt";
	// API_OutputMA(o_file);
	
	input_prepared = false;
}

/* ********************** Private methods - experimental ************* */

int getWmatEdgeCount(void)
{
	int k = 0;
	for (int e = 0;  e < GetNumberOfEdges(); e++) {
		if (IsWmatEdge(e)) k++;
	}
	return k;
}

std::string coordToString(const coord & coord)
{
	using namespace std;

	stringstream ss;
	ss << "(" << UnscaleX(coord.x) << ", " << UnscaleY(coord.y) << ")";
	return ss.str();
}

static void publish_input_data(ros::Publisher & marker_pub, std::string frame_id, double duration)
{
	// prepare the Marker
	visualization_msgs::Marker input_marker;
	input_marker.header.frame_id = frame_id;
	input_marker.header.stamp = ros::Time::now();
	input_marker.ns = "input";
	input_marker.action = visualization_msgs::Marker::ADD;
	input_marker.pose.orientation.w = 1.0;
	input_marker.id = 1;
	input_marker.lifetime = ros::Duration(duration);
	input_marker.type = visualization_msgs::Marker::LINE_LIST;
	input_marker.scale.x = 2;
	input_marker.color.g = 1.0f;
	input_marker.color.a = 1.0;

	using namespace std;

//	cout << "Number of input segments: " << num_segs << endl;

	geometry_msgs::Point p;
	for(int i = 0; i < num_segs; i++) { // num_segs is internal Vroni variable
		p.x = UnscaleX(GetSegStartCoord(i).x);
		p.y = UnscaleY(GetSegStartCoord(i).y);
		input_marker.points.push_back(p);
		p.x = UnscaleX(GetSegEndCoord(i).x);
		p.y = UnscaleY(GetSegEndCoord(i).y);
		input_marker.points.push_back(p);
	}

	marker_pub.publish(input_marker);
}

/* None of defining sites is dummy point; e */
static bool isEdgeDefinedByDummyPoint(int e)
{
	// num_pts is Vroni's internal variable
	int last_valid_pnt_ix = num_pnts - 3;

	int s1, s2; t_site t1, t2;
	GetLftSiteData(e, &s1, &t1);
	GetRgtSiteData(e, &s2, &t2);

	return s1 == 0 || s1 == 1 || s1 > last_valid_pnt_ix
		|| s2 == 0 || s2 == 1 || s2 > last_valid_pnt_ix;
}

static std::string site_type_names[] = {"SEG", "ARC", "PNT", "VDN", "VDE", "DTE", "CCW", "CW", "UNKNOWN" };

static std::string edgeDefiningSitesToString(int e)
{
	int s1, s2; t_site t1, t2;
	GetLftSiteData(e, &s1, &t1);
	GetRgtSiteData(e, &s2, &t2);

	using namespace std;

	stringstream ss;

	ss << "s1 = " << s1
		<< "\t[" << site_type_names[t1] << "],\ts2 = " << s2
		<< "\t[" << site_type_names[t2] << "]";
	return ss.str();
}

visualization_msgs::Marker prepareSphereMarker(int id, double radius, std::string frame_id, double duration) {
	// prepare the Marker
	visualization_msgs::Marker new_marker;
	new_marker.header.frame_id = frame_id;
	new_marker.header.stamp = ros::Time::now();
	new_marker.ns = "deg2Nodes";
	new_marker.action = visualization_msgs::Marker::ADD;
	new_marker.pose.orientation.w = 1.0;
	new_marker.id = id;
	new_marker.lifetime = ros::Duration(duration);
	new_marker.type = visualization_msgs::Marker::SPHERE;
	new_marker.scale.x = new_marker.scale.y = new_marker.scale.z = radius;
	new_marker.color.b = 1.0f;
	new_marker.color.a = 0.5;

	return new_marker;
}

//tempate <class T>
//boolean contains(list<T> & lst, const T &element)
//{
//	list<T>::iterator it =  find(lst.begin(), lst.end(), element);
//	return it != lst.end();
//}
bool contains(std::list<int> & lst, const int & element)
{
	std::list<int>::iterator it = find(lst.begin(), lst.end(), element);
	return it != lst.end();
}

/* n - node_id */
static void publishCriticalNodeCandidateIfAppropriate(int e, std::list<int> & usedNodes, ros::Publisher & marker_pub, std::string frame_id, double duration)
{
		int n1 = GetStartNode(e);
		int n2 = GetEndNode(e);

		bool isN1Deg2 = IsDeg2Node(n1);
		bool isN2Deg2 = IsDeg2Node(n2);

		// only degree 2 nodes are critical point candidates
		if (!isN1Deg2 && !isN2Deg2) {
			return;
		}

		coord c1, c2; double r1, r2;
		GetNodeData(n1, &c1, &r1);
		GetNodeData(n2, &c2, &r2);

		// FIXME: there are only points of degree 3, more incident point
		// representing single point ... don't forget

		// critical point candidate has to be local minimum
		if (r1 == r2) {
			return;
		}

		bool hasNeighbourOfDeg3 = false; // TODO
		int candidate;
		coord c_candidate;
		double r_candidate;

		// the one with smaller clearance radius is the candidate
		if (r1 < r2) {
			// the candidate is degree 3 -> not a critical point
			if (!isN1Deg2) {
				return;
			}
			// clearance radius has to be positive
			if (r1 == 0) {
				return;
			}
			if (!isN2Deg2) {
				hasNeighbourOfDeg3 = true;
			}
			candidate = n1;
			r_candidate = r1;
			c_candidate = c1;
		} else {
			// the candidate is degree 3 -> not a critical point
			if (!isN2Deg2) {
				return;
			}
			// clearance radius has to be positive
			if (r2 == 0) {
				return;
			}
			if (!isN1Deg2) {
				hasNeighbourOfDeg3 = true;
			}
			candidate = n2;
			r_candidate = r2;
			c_candidate = c2;
		}

		coord c_ccw, c_cw; double r_ccw, r_cw;

		int e_ccw = GetCCWEdge(e,candidate);
		int n_ccw = GetOtherNode(e_ccw, candidate);
		GetNodeData(n_ccw, &c_ccw, &r_ccw);
		// clearance radii of all neighbour have to be greater than ours
		if (r_candidate >= r_ccw) {
			return;
		}
		if (!IsDeg2Node(n_ccw)) {
			hasNeighbourOfDeg3 = true;
		}
		int e_cw = GetCWEdge(e,candidate);
		int n_cw = GetOtherNode(e_cw, candidate);
		GetNodeData(n_cw, &c_cw, &r_cw);
		// clearance radii of all neighbour have to be greater than ours
		if (r_candidate >= r_cw) {
			return;
		}
		if (!IsDeg2Node(n_cw)) {
			hasNeighbourOfDeg3 = true;
		}

		// candidate node has to have neighbour of degree 3
		if (!hasNeighbourOfDeg3) {
			return;
		}

		// All right. Our candidate node is a true critical node!

		if (!contains(usedNodes, candidate)) {
			visualization_msgs::Marker marker = prepareSphereMarker(candidate, UnscaleV(r_candidate), frame_id, duration);
			marker.pose.position.x = UnscaleX(c_candidate.x);
			marker.pose.position.y = UnscaleY(c_candidate.y);
			usedNodes.push_back(candidate);
			marker_pub.publish(marker);
		}
}

static bool printed = false;

void Poly2VdConverter::publish_wmat_deg2_nodes(ros::Publisher & marker_pub, std::string frame_id, double duration)
{
	using namespace std;
	using namespace visualization_msgs;

	list<int> usedNodes;

	for (int e = 0;  e < GetNumberOfEdges(); e++) {
		if (!IsWmatEdge(e)) {
			continue;
		}

		publishCriticalNodeCandidateIfAppropriate(e, usedNodes, marker_pub, frame_id, duration);
	}

	if (!printed) {
		cout << "have count " << usedNodes.size() << endl;
		cout << "publishing ids: ";
		list<int>::iterator it;
		for (it = usedNodes.begin(); it != usedNodes.end(); it++) {
			cout << *it << ", ";
		}
		cout << endl;
		printed = true;
	}
}

void Poly2VdConverter::publish_wmat(ros::Publisher & marker_pub, std::string frame_id, double duration)
{
	// prepare the Marker
	visualization_msgs::Marker wmat_marker;
	wmat_marker.header.frame_id = frame_id;
	wmat_marker.header.stamp = ros::Time::now();
	wmat_marker.ns = "wmat";
	wmat_marker.action = visualization_msgs::Marker::ADD;
	wmat_marker.pose.orientation.w = 1.0;
	wmat_marker.id = 0;
	wmat_marker.lifetime = ros::Duration(duration);
	wmat_marker.type = visualization_msgs::Marker::LINE_LIST;
	wmat_marker.scale.x = 0.25;
	wmat_marker.color.g = 1.0f;
	wmat_marker.color.a = 1.0;

	using namespace std;

//	if (!printed) cout << "count: " << num_pnts << endl;

//	stringstream ossX;
//	ossX << "W D\nm u\na m\nt m\n  y" << endl << endl;

	geometry_msgs::Point p;
	for (int e = 0;  e < GetNumberOfEdges(); e++) {
//		ossX << (IsWmatEdge(e)?'w':'-') << " ";

//		either
//		if (isEdgeDefinedByDummyPoint)
//			continue;
//		}
//		or
		if (!IsWmatEdge(e)) {
			continue;
		}

//		ossX << (isEdgeDefinedByDummyPoint(e)?'-':'x') << "\t";
//		ossX << edgeDefiningSitesToString(e) << "\t";

		coord c; double r;
		GetNodeData(GetStartNode(e), &c, &r);
//		ossX << coordToString(c) << ",\t";
		p.x = UnscaleX(c.x);
		p.y = UnscaleY(c.y);
		wmat_marker.points.push_back(p);
		GetNodeData(GetEndNode(e), &c, &r);
//		ossX << coordToString(c) << endl;
		p.x = UnscaleX(c.x);
		p.y = UnscaleY(c.y);
		wmat_marker.points.push_back(p);
	}

//	if (!printed) {
//		cout << ossX.str();
//		printed = true;
//	}

	marker_pub.publish(wmat_marker);
}

/* Sends single message with input and output data */
void publish_result( int argc, char *argv[], Poly2VdConverter & p2vd )
{
	// init ros
	ros::init(argc, argv, "poly2vd");
	ros::NodeHandle n;
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

	ros::Rate r(1);

	while (ros::ok())
	{
		// publish both input segments and output wmat data
		publish_input_data(marker_pub, "/odom", 5.0);
		p2vd.publish_wmat(marker_pub, "/odom", 5.0);

		r.sleep();
 	}
}

#ifdef POLY2VD_STANDALONE

/**
 * The main() function serves my experimental purposes
 */
int main ( int argc, char *argv[] )
{
	using namespace std;

	if (argc != 2) {
		cout << "Usage: " << argv[0] << " filename" << endl <<
			endl << "file has to be of type known to Vroni" << endl;
		return EXIT_FAILURE;
	}

	Poly2VdConverter p2vd;
	// p2vd.prepareNewInput(segs, size);
	p2vd.prepareNewInput(argv[1]);
	p2vd.convert();

	cout << "total edges size: " << edges.size() << ", wmat edges: " << getWmatEdgeCount() << endl;
	cout << "WMAT edges count: " << getWmatEdgeCount() << endl;
	cout << "node count: " << GetNumberOfNodes() << endl;
	cout << "pnts count: " << num_pnts << endl;
	cout << "segs count: " << num_segs << endl;

	publish_result(argc, argv, p2vd);

	return EXIT_SUCCESS;
}				/* ----------  end of function main  ---------- */

#endif 			/* ----- #ifndef POLY2VD_STANDALONE ----------- */

// vi:ai:sw=4 ts=4 sts=0
