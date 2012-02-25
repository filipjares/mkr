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
#include <fstream>
#include <iomanip>
#include <string>
#include <limits>
#include <list>
#include <map>
#include <algorithm>
#include <assert.h>
#include <cmath>
#include "poly2vd.hpp"

/* ************ ROS includes (other than in poly2vd.hpp) ************* */

#ifndef POLY2VD_STANDALONE

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#endif

// These macros are defined in Vroni's basic.h; but I had problems including it

#define ScaleX(xc)  (scale_factor * ((xc) - shift.x))

#define ScaleY(yc)  (scale_factor * ((yc) - shift.y))

#define ScaleV(value)  ((value) * scale_factor)

#define UnscaleX(xc)  (assert(scale_factor > 0.0), (xc) / scale_factor + shift.x)

#define UnscaleY(yc)  (assert(scale_factor > 0.0), (yc) / scale_factor + shift.y)

#define UnscaleV(value)  (assert(scale_factor > 0.0), (value) / scale_factor)

/* ******************************************************************* */
/* ********************** Constructor and destructor ***************** */
/* ******************************************************************* */

Poly2VdConverter::Poly2VdConverter()
{
	API_InitializeProgram();
	input_prepared = false;
}

Poly2VdConverter::~Poly2VdConverter()
{
	API_TerminateProgram();
}

/* ******************************************************************* */
/* ********************** Public methods ***************************** */
/* ******************************************************************* */

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
		emptyString,    /* output file for point VD/DT  */
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

/* ******************************************************************* */
/* ********************** Private methods follow ********************* */
/* ******************************************************************* */

/* ******************* Structures and constants ********************** */

struct Color {
	float r, g, b, a;
	Color();
	Color(float _r, float _g, float _b): r(_r), g(_g), b(_b), a(1.0f) {};
	Color(float _r, float _g, float _b, float _a): r(_r), g(_g), b(_b), a(_a) {};

	static const Color BLUE;
	static const Color RED;
	static const Color YELLOW;
};

const Color Color::BLUE(0.0f, 0.0f, 1.0f, 0.5f);
const Color Color::RED (1.0f, 0.0f, 0.0f, 0.75f);
const Color Color::YELLOW (1.0f, 1.0f, 0.0f, 0.75f);

static std::string site_type_names[] = {"SEG", "ARC", "PNT", "VDN", "VDE", "DTE", "CCW", "CW", "UNKNOWN" };

const bool SHRINK = false;
const bool SHUFFLE = true;
const bool EXPORT2DOT = true;
const bool PUBLISH_ROS = false;

const double COMPARISON_RATIO = 0.96;

/* *************** Utility functions (VRONI-related) ***************** */

static int getWmatEdgeCount(void)
{
	int k = 0;
	for (int e = 0;  e < GetNumberOfEdges(); e++) {
		if (IsWmatEdge(e)) k++;
	}
	return k;
}

static coord roundCoord(const coord &c)
{
	double N = 10.0; coord r;	// FIXME: use constant

	r.x = round(N*c.x)/N;
	r.y = round(N*c.y)/N;
	
	return r;
}

static bool coordCompare(const coord & c1, const coord & c2)
{
	coord r1 = roundCoord(c1);
	coord r2 = roundCoord(c2);

	if (r1.x < r2.x) {
		return true;
	} else {
		return r1.y < r2.y;
	}
}

static double coordDistance(const coord &c1, const coord &c2)
{
	return sqrt((c2.x - c1.x)*(c2.x - c1.x) + (c2.y - c1.y)*(c2.y - c1.y));
}

static double nodeDistance(int n1, int n2)
{
	return coordDistance(GetNodeCoord(n1), GetNodeCoord(n2));
}

static bool areCoordsEqual(const coord &c1, const coord &c2)
{
	return c1.x == c2.x && c1.y == c2.y;
}

static bool areNodesEqual(int n1, int n2)
{
	coord c1, c2; double r1, r2;
	GetNodeData(n1, &c1, &r1);
	GetNodeData(n2, &c2, &r2);

	return areCoordsEqual(c1, c2);
}

static bool areNodesInSingleBin(int n1, int n2)
{
	// are rounded coords equal?
	coord r1 = roundCoord(GetNodeCoord(n1));
	coord r2 = roundCoord(GetNodeCoord(n2));

	return areCoordsEqual(r1, r2);
}

static bool areNodesNear(int n1, int n2)
{
	double l = nodeDistance(n1, n2);

	return l < 0.1;		// FIXME use constant
}

/* Performs "contrast comparison" of non-negative arguments, it treats "close
 * enough" numbers as being equal.  Checks for significant ratio between both
 * arguments.  The COMPARISON_RATIO constant is used as a "threshold".
 * COMPARISON_RATIO is lower than 1 and positive. We define x1 being
 * "contrasty lower" than x2 (i.e. x1 << x2) if and only if it is
 * x1 < COMPARISON_RATIO * x2.
 *
 * Returns:
 * 			-1		iff		x1  <<  x2
 * 			 0		iff		x1 "==" x2
 * 			 1		iff		x1  >>  x2
 *
 * Suggested usage: contrastCompare(x1, x2) OP 0, where OP is one of usual
 * comparison operators ('<', '==', '!=', '>').
 */
static char contrastCompare(double x1, double x2)
{
	assert(x1 >= 0 && x2 >= 0); // implemented for non-negative inputs only

	if (x1 < x2) {
		if (x1 < COMPARISON_RATIO * x2) {
			return -1;
		} else {
			return 0;
		}
	} else {
		if (x2 < COMPARISON_RATIO * x1) {
			return 1;
		} else {
			return 0;
		}
	}
}

static coord determineNodeDisplacement(int n)
{
	// Compute centre of gravity of neighbours of n
	int e1   = GetIncidentEdge(n);
	int n1   = GetOtherNode(e1, n);
	coord c1 = GetNodeCoord(n1);

	int e2   = GetCCWEdge(e1, n);
	int e3   = GetCWEdge(e1, n);

	int n2   = GetOtherNode(e2, n);
	coord c2 = GetNodeCoord(n2);

	coord centre;					// neighbours' centre of gravity
	if (e2 == e3) {
		centre.x = (c1.x + c2.x)/2;
		centre.y = (c1.y + c2.y)/2;
	} else {
		int n3    = GetOtherNode(e3, n);
		coord c3  = GetNodeCoord(n3);
		centre.x = (c1.x + c2.x + c3.x)/3;
		centre.y = (c1.y + c2.y + c3.y)/3;
	}

	coord c = GetNodeCoord(n);		// original node's coords

	coord d;						// the displacement
	double l = coordDistance(c, centre);
	if (l == 0.0) {
		d.x = 0.0;
		d.y = 0.0;
	} else {
		d.x = centre.x - c.x;
		d.y = centre.y - c.y;
		// FIXME: use reasonable constants
		d.x *= 0.05 / l;				// normalize
		d.y *= 0.05 / l;
	}

	return d;
}

static bool hasIncidentNeighbour(int n)
{
	int e1 = GetIncidentEdge(n);
	int n1 = GetOtherNode(e1, n);

	if (areNodesEqual(n, n1)) {
		return true;
	}

	int e_ccw = GetCCWEdge(e1, n);
	int n_ccw = GetOtherNode(e_ccw, n);
	if (areNodesEqual(n, n_ccw)) {
		return true;
	}

	int e_cw = GetCCWEdge(e1, n);
	int n_cw = GetOtherNode(e_cw, n);
	if (areNodesEqual(n, n_cw)) {
		return true;
	}

	return false;
}

static bool hasCloseNeighbour(int n)
{
	int e1 = GetIncidentEdge(n);
	int n1 = GetOtherNode(e1, n);

	bool result = false;
//	if (areNodesInSingleBin(n, n1)) {
	if (areNodesNear(n, n1)) {
		result = true;
	}

	int e_ccw = GetCCWEdge(e1, n);
	int n_ccw = GetOtherNode(e_ccw, n);
//	if (areNodesInSingleBin(n, n_ccw)) {
	if (areNodesNear(n, n_ccw)) {
		result = true;
	}

	int e_cw = GetCCWEdge(e1, n);
	int n_cw = GetOtherNode(e_cw, n);
//	if (areNodesInSingleBin(n, n_cw)) {
	if (areNodesNear(n, n_cw)) {
		result = true;
	}

	return result;
}

/* Vroni adds four dummy points to defining sites of the input; dummy points are located at the corners
 * of the bounding box and their indices in the pnts vector are 0, 1, num_pnts-2 and num_pnts-1. */
static bool isDummyPoint(const int & site_id, const t_site & site_type)
{
	// num_pts is Vroni's internal variable
	int last_valid_pnt_ix = num_pnts - 3;
	
	return site_type == PNT && (site_id == 0 || site_id == 1 || site_id > last_valid_pnt_ix);
}

/** Returns true iff none of the defining sites of the edge e is dummy point. */
static bool isEdgeDefinedByDummyPoint(int e)
{
	int s1, s2; t_site t1, t2;
	GetLftSiteData(e, &s1, &t1);
	GetRgtSiteData(e, &s2, &t2);

	return isDummyPoint(s1, t1) || isDummyPoint(s2, t2);
}

static bool isDeg2WmatNode(int e, int n)
{
	assert(IsWmatEdge(e));

	int e_ccw = GetCCWEdge(e, n);
	int e_cw  = GetCWEdge(e, n);

	if (e_ccw == e_cw) {
		return IsWmatEdge(e_ccw);
	} else {
		return ( IsWmatEdge(e_ccw) && !IsWmatEdge(e_cw) )
		    || (!IsWmatEdge(e_ccw) &&  IsWmatEdge(e_cw) );
	}
}

static bool isDeg2Node(int n)
{
	int e = GetIncidentEdge(n);
	int e_ccw = GetCCWEdge(e, n);
	int e_cw  = GetCWEdge(e, n);
		
	if (e_ccw == e_cw) {
		return IsWmatEdge(e_ccw);
	} else {
		return ( IsWmatEdge(e_ccw) && !IsWmatEdge(e_cw) )
		    || (!IsWmatEdge(e_ccw) &&  IsWmatEdge(e_cw) );
	}
}

static bool isDeg3Node(int n)
{
	int e = GetIncidentEdge(n);
	int e_ccw = GetCCWEdge(e, n);
	int e_cw  = GetCWEdge(e, n);
		
	if (e_ccw != e_cw) {
		return IsWmatEdge(e_ccw) && IsWmatEdge(e_cw) && IsWmatEdge(e);
	} else {
		return false;
	}
}

static bool hasDeg3Neighbour(int n)
{
	int e = GetIncidentEdge(n);
	int m = GetOtherNode(e, n);

	return isDeg3Node(m);
}

static bool isFrontierBasedEdge(int e)
{
	int s1, s2; t_site t1, t2;
	GetLftSiteData(e, &s1, &t1);
	GetRgtSiteData(e, &s2, &t2);

	return (t1 == SEG && segs[s1].ext_appl.isFrontier) || (t2 == SEG && segs[s2].ext_appl.isFrontier);
}

static bool isSegmentBasedEdge(int e)
{
	int s1, s2; t_site t1, t2;
	GetLftSiteData(e, &s1, &t1);
	GetRgtSiteData(e, &s2, &t2);

	return (t1 == SEG) || (t2 == SEG);
}

static bool isOuterBasedEdge(int e)
{
	int s1, s2; t_site t1, t2;
	GetLftSiteData(e, &s1, &t1);
	GetRgtSiteData(e, &s2, &t2);

	return (t1 == SEG && segs[s1].ext_appl.isOuter) || (t2 == SEG && segs[s2].ext_appl.isOuter);
}

	/* *************** Utility functions (other) ************************* */

	//tempate <class T>
	//boolean contains(list<T> & lst, const T &element)
	//{
	//	list<T>::iterator it =  find(lst.begin(), lst.end(), element);
	//	return it != lst.end();
	//}
	bool contains(std::list<int> & lst, int element)
	{
		std::list<int>::iterator it = find(lst.begin(), lst.end(), element);
		return it != lst.end();
	}

	/** random double in range [-1, 1] */
	double random_double()
	{
		return 2*((double)rand()/(double)RAND_MAX) - 1;
	}

	template <class T>
	inline std::string to_string (const T& t)
	{
		std::stringstream ss;
		ss << t;
		return ss.str();
	}

	/* ********************** toString() functions *********************** */

	std::string coordToString(const coord & coord, bool addParentheses)
	{
		using namespace std;

		string lp, rp;
		if (addParentheses) {
			lp = "(";
			rp = ")";
		} else {
			lp = rp = "";
		}

		stringstream ss;
		ss << fixed << setprecision(2) << right;
		ss << lp << setw(6) << right << UnscaleX(coord.x) << ", " // FIXME
				 << setw(6) << right << UnscaleY(coord.y) << rp;
		return ss.str();
	}

	std::string coordToString(const coord & coord)
	{
		return coordToString(coord, true);
	}

	std::string nodeToString(int n)
	{
		coord c; double r;
		GetNodeData(n, &c, &r);

		using namespace std;
		stringstream ss;
		ss << fixed << setprecision(2) << right;
		ss << "[n=" << n << ", r =" << setw(6) << UnscaleV(r) << ", coord = " << coordToString(c) << "]";

		return ss.str();
	}

	std::string edgeToString(int e)
	{
		int n1 = GetStartNode(e);
		int n2 = GetEndNode(e);

		using namespace std;
		stringstream ss;
		ss << fixed << setprecision(2) << right;
		ss << "{e = " << e << ", " << (IsWmatEdge(e)?"WMAT":"nowm") << ": "
			<< nodeToString(n1) << ", " << nodeToString(n2) << "}";

		return ss.str();
	}

	static std::string edgeDefiningSitesToString(int e)
	{
		int s1, s2; t_site t1, t2;
		GetLftSiteData(e, &s1, &t1);
		GetRgtSiteData(e, &s2, &t2);

		using namespace std;

		stringstream ss;

		ss << "s1 = " << setw(2) << right << s1
			<< " [" << site_type_names[t1] << "],  s2 = "
			<< setw(2) << right << s2
			<< " [" << site_type_names[t2] << "]";
		return ss.str();
	}

	/* ********************** "Publisher" functions ********************** */

	#ifndef POLY2VD_STANDALONE

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

	void publishSphere(ros::Publisher & marker_pub, int id, coord location, double diameter, Color color, std::string frame_id, double duration)
	{
		// prepare the Marker
		visualization_msgs::Marker marker;
		marker.header.frame_id = frame_id;
		marker.header.stamp = ros::Time::now();
		marker.ns = "deg2Nodes";
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.orientation.w = 1.0;
		marker.id = id;
		marker.lifetime = ros::Duration(duration);
		marker.type = visualization_msgs::Marker::SPHERE;

		// set its position, diameter and color
		marker.pose.position.x = UnscaleX(location.x);
		marker.pose.position.y = UnscaleY(location.y);
		marker.scale.x = marker.scale.y = marker.scale.z = UnscaleV(diameter);
		marker.color.r = color.r; marker.color.g = color.g; marker.color.b = color.b;
		marker.color.a = color.a;

		// publish it using given publisher
		marker_pub.publish(marker);
	}

	/* e - edge id */
	static void publishCriticalNodeCandidateIfAppropriate(int e, std::list<int> & usedNodes, ros::Publisher & marker_pub, std::string frame_id, double duration, bool printIt)
	{
			int n1 = GetStartNode(e);
			int n2 = GetEndNode(e);

			// are the end-nodes connecting this WMAT Edge (e) with just one other WMAT Edge?
			bool isN1Deg2 = isDeg2WmatNode(e, n1); // IsDeg2Node(n1);
			bool isN2Deg2 = isDeg2WmatNode(e, n2); // IsDeg2Node(n2);

			// only degree 2 nodes are critical point candidates
			if (!isN1Deg2 && !isN2Deg2) {
				return;
			}

			coord c1, c2; double r1, r2;
			GetNodeData(n1, &c1, &r1);
			GetNodeData(n2, &c2, &r2);

			// FIXME: VRONI does not have VD nodes with degree higher than 3. Nodes
			// of higher degree are represented by multiple nodes of maximum
			// degree 3 located at the same position and iterconnected together.
			// Have to cope with that... For now, I am pretending there are no such
			// connected nodes.

			// critical point candidate has to be local minimum
			if (contrastCompare(r1, r2) == 0) {
				return;
			}

			bool hasNeighbourOfDeg3 = false;
			int candidate;
			coord c_candidate;
			double r_candidate;

			// the one with smaller clearance radius is the candidate
			if (contrastCompare(r1, r2) < 0) {
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

			// Note: e_ccw and e_cw are not neccessarily equal now.  Even though the
			// candidate node is has WMAT degree of 2, its VD degree may be higher.

			int e_ccw = GetCCWEdge(e,candidate);
			int e_cw = GetCWEdge(e,candidate);

			if (IsWmatEdge(e_ccw)) {
				assert(e_ccw == e_cw || !IsWmatEdge(e_cw));

				int n_ccw = GetOtherNode(e_ccw, candidate);
				GetNodeData(n_ccw, &c_ccw, &r_ccw);
				// clearance radii of all the neighbours have to be greater than ours
				if (r_candidate >= r_ccw) {
					return;
				}
				if (!IsDeg2Node(n_ccw)) {
					hasNeighbourOfDeg3 = true;
				}
				if (areCoordsEqual(c_candidate, c_ccw)) {
					using namespace std;
					cout << "The candidate (" << candidate << ") and node related to it through "
						<< "e_ccw have the same coords: " << coordToString(c_candidate) << endl;
				}
			}

			if (IsWmatEdge(e_cw)) {
				assert(e_ccw == e_cw || !IsWmatEdge(e_ccw));

				int n_cw = GetOtherNode(e_cw, candidate);
				GetNodeData(n_cw, &c_cw, &r_cw);
				// clearance radii of all neighbours have to be greater than ours
				if (r_candidate >= r_cw) {
					return;
				}
				if (!IsDeg2Node(n_cw)) {
					hasNeighbourOfDeg3 = true;
				}
				if (areCoordsEqual(c_candidate, c_cw)) {
					using namespace std;
					cout << "The candidate (" << candidate << ") and node related to it through "
						<< "e_cw have the same coords: " << coordToString(c_candidate) << endl;
				}
			}

			// candidate node has to have neighbour of degree 3
			if (!hasNeighbourOfDeg3) {
				return;
			}

			// All right. Our candidate node is a true critical node!

			if (!contains(usedNodes, candidate)) {
				publishSphere(marker_pub, candidate, c_candidate, r_candidate, Color::BLUE, frame_id, duration);
				usedNodes.push_back(candidate);

				using namespace std;

				if (printIt) {
					cout << setw(4) << right << candidate << ": " << coordToString(c_candidate)
						<< " (" << fixed << setprecision(3) << r_candidate << ")"
						<< ":\t"
						<< endl;
					// TODO
				}
			}
	}

	// FIXME: use VRONI's definition
	#define  ZERO      1.0e-13   /* small number, greater than machine precision */

	int getMaNodeNotOnBoundary()
	{
		bool at_boundary = true;
		int n = 0, e;
		double r;

		// find a MA node that is not on the boundary
		e = 0;
		while (at_boundary  &&  (e < GetNumberOfEdges())) {
			if (IsWmatEdge(e)) {
				n = GetStartNode(e);
				r = GetNodeParam(n);
				if (r > ZERO) {
					at_boundary = false;
				}
				else {
				n = GetOtherNode(e, n);
				r = GetNodeParam(n);
				if (r > ZERO) {
					at_boundary = false;
				}
				else {
					++e;
				}
			}
		}
		else {
			++e;
		}
	}

	if (at_boundary) {
		return -1;
	} else {
		return n;
	}
}


void addTheOtherNodeIfAppropriate(int edge, int sourceNode, std::list<int> & open, bool *closed)
{
	if (IsWmatEdge(edge)) {
		int otherNode = GetOtherNode(edge, sourceNode);
		if (closed[otherNode] == false) {
			closed[otherNode] = true;
			open.push_back(otherNode);
		}
	} else {
		// TODO: check incident nodes
	}
}

void BFS(int root, bool * nodes)
{
	std::list<int> open;
	bool closed[GetNumberOfNodes()];	
	for (int i = 0; i < GetNumberOfNodes(); i++) closed[i] = false;
	
	open.push_back(root);

	while (!open.empty()) {
		int n = open.front();
		open.pop_front();
		
		if(closed[n] == true)
			continue;

		closed[n] = true;
		
		// each Vroni's node has at most three incident edges
		int e1 = GetIncidentEdge(n);				// get the first one
		nodes[GetStartNode(e1)] = true;
		nodes[GetEndNode(e1)] = true;
		if(GetNodeParam(GetStartNode(e1)) != 0){
			open.push_back(GetStartNode(e1));
		}
		if(GetNodeParam(GetEndNode(e1)) != 0){
			open.push_back(GetEndNode(e1));
		}

		int e_ccw = GetCCWEdge(e1, n);				// get the second one
		nodes[GetStartNode(e_ccw)] = true;
		nodes[GetEndNode(e_ccw)] = true;
		if(GetNodeParam(GetStartNode(e_ccw)) != 0){
			open.push_back(GetStartNode(e_ccw));
		}
		if(GetNodeParam(GetEndNode(e_ccw)) != 0){
			open.push_back(GetEndNode(e_ccw));
		}

		int e_cw = GetCWEdge(e1, n);
		if (e_cw != e_ccw){
			nodes[GetStartNode(e_cw)] = true;
			nodes[GetEndNode(e_cw)] = true;
			if(GetNodeParam(GetStartNode(e_cw)) != 0){
				open.push_back(GetStartNode(e_cw));
			}
			if(GetNodeParam(GetEndNode(e_cw)) != 0){
				open.push_back(GetEndNode(e_cw));
			}
		}
		
	}

}

void markOutNodes(bool * nodes)
{	
	coord c; double r;
	for (int e = 0;  e < GetNumberOfEdges(); e++) {	
		if (!IsWmatEdge(e)) {
			continue;
		}

		GetNodeData(GetStartNode(e), &c, &r);
		if(abs(c.x) >= 1 || abs(c.y) >= 1)
			BFS(GetStartNode(e),nodes);

		GetNodeData(GetEndNode(e), &c, &r);
		if(abs(c.x) >= 1 || abs(c.y) >= 1)
			BFS(GetEndNode(e),nodes);
	}
}

bool testRootNode(int root)
{
	std::list<int> open;
	bool closed[GetNumberOfNodes()];	
	for (int i = 0; i < GetNumberOfNodes(); i++) closed[i] = false;
	
	open.push_back(root);

	while (!open.empty()) {
		int n = open.front();
		open.pop_front();
		
		if(closed[n] == true)
			continue;

		closed[n] = true;
		
		int e1 = GetIncidentEdge(n);				// get the first one
		if(isOuterBasedEdge(e1))
			return true;		
		if(GetNodeParam(GetStartNode(e1)) != 0){
			open.push_back(GetStartNode(e1));
		}
		if(GetNodeParam(GetEndNode(e1)) != 0){
			open.push_back(GetEndNode(e1));
		}

		int e_ccw = GetCCWEdge(e1, n);				// get the second one
		if(isOuterBasedEdge(e_ccw))
			return true;		
		if(GetNodeParam(GetStartNode(e_ccw)) != 0){
			open.push_back(GetStartNode(e_ccw));
		}
		if(GetNodeParam(GetEndNode(e_ccw)) != 0){
			open.push_back(GetEndNode(e_ccw));
		}

		int e_cw = GetCWEdge(e1, n);
		if (e_cw != e_ccw){
			if(isOuterBasedEdge(e_cw))
				return true;		
			if(GetNodeParam(GetStartNode(e_cw)) != 0){
				open.push_back(GetStartNode(e_cw));
			}
			if(GetNodeParam(GetEndNode(e_cw)) != 0){
				open.push_back(GetEndNode(e_cw));
			}
		}
		
	}
	return false;
}

int getRootNode(SPosition & pos){
// find node which is not on the boundary and is inside polygon

	bool outNodes[GetNumberOfNodes()];
	for (int i = 0; i < GetNumberOfNodes(); i++) outNodes[i] = false;
	markOutNodes(outNodes);

	coord p;
	p.x = pos.x;
	p.y = pos.y;
	coord c,cu;
	double r, dist;
	int root;
	do{
	root = -1;
	double min_dist = std::numeric_limits<double>::max();
		for (int e = 0;  e < GetNumberOfEdges(); e++) {
			
			if (!IsWmatEdge(e)) {
				continue;
			}
	
			if(outNodes[GetStartNode(e)] || outNodes[GetEndNode(e)])
				continue;
	
			// With this condition included, I (fj) found initial robot position
			// which resulted in unsatisfied assertion after the for loop
			// if (isSegmentBasedEdge(e)) {
			// 	continue;
			// }
	
			GetNodeData(GetStartNode(e), &c, &r);
			cu.x = UnscaleX(c.x);
			cu.y = UnscaleY(c.y);
			dist = coordDistance(p,cu);
			if(dist < min_dist){
				root = GetStartNode(e);
				min_dist = dist;
			}
	
			GetNodeData(GetEndNode(e), &c, &r);
			cu.x = UnscaleX(c.x);
			cu.y = UnscaleY(c.y);
			dist = coordDistance(p,cu);
			if(dist < min_dist){
				root = GetEndNode(e);
				min_dist = dist;
			}
		}
		assert(root != -1);
		outNodes[root] = true;
	}
	while(!testRootNode(root));

	return root;
}

void doTheSearch(ros::Publisher & marker_pub, const std::string & frame_id, double duration)
{
	using namespace std;

	int root = getMaNodeNotOnBoundary();
	
	if (root == -1) {
		std::cerr << "Medial axis not known or data is corrupted. Could not find any node not on the boundary";
		return;
	}

	// publish the root node as red sphere
	coord c; double r;
	GetNodeData(root, &c, &r);
	publishSphere(marker_pub, root, c, 2*r, Color::RED, frame_id, duration);

	// prepare open and close "lists":
	int nodeCount = GetNumberOfNodes();
	std::list<int> open;
	bool closed[nodeCount];
	for (int i = 0; i < nodeCount; i++) closed[i] = false;

	open.push_back(root);
	closed[root] = true;

	while (!open.empty()) {
		int n = open.front();
		open.pop_front();

		// each Vroni's node has at most three incident edges
		int e1 = GetIncidentEdge(n);				// get the first one
		addTheOtherNodeIfAppropriate(e1, n, open, closed);

		int e_ccw = GetCCWEdge(e1, n);				// get the second one
		addTheOtherNodeIfAppropriate(e_ccw, n, open, closed);

		int e_cw = GetCWEdge(e1, n);
		if (e_cw != e_ccw) addTheOtherNodeIfAppropriate(e_cw, n, open, closed);
	}
}

void findCriticalNodes(bool * cNodes, bool * nodes)
{
	//TODO - here the nodes inside should be analyzed and searched for criticle ones and then published
	//now just vizualize them
	//later the functions will be modified to return usable data and suppress vizualization

	for (int n = 0;  n < GetNumberOfNodes(); n++) {
		cNodes[n] = false;	
		
		if (nodes[n]) {
			if(isDeg2Node(n) && GetNodeParam(n) > 0.01) {
				if(hasDeg3Neighbour(n))
					cNodes[n] = true;
			}
		}
	}
}

void Poly2VdConverter::publish_wmat_deg2_nodes(ros::Publisher & marker_pub, const std::string & frame_id, double duration)
{
	static int printed = 0;
	bool printIt = (printed % 20 == 0);

	using namespace std;
	using namespace visualization_msgs;

	// doTheSearch(marker_pub, frame_id, duration);

	list<int> usedNodes;

	for (int e = 0;  e < GetNumberOfEdges(); e++) {
		if (!IsWmatEdge(e)) {
			continue;
		}

		publishCriticalNodeCandidateIfAppropriate(e, usedNodes, marker_pub, frame_id, duration, printIt);
	}

	if (printIt) {
		cout << "publishing " << usedNodes.size() << " critical nodes: ";
		list<int>::iterator it;
		for (it = usedNodes.begin(); it != usedNodes.end(); it++) {
			cout << *it << ", ";
		}
		cout << endl;
	}
	printed++;
}

void Poly2VdConverter::publish_root(ros::Publisher & marker_pub, SPosition & p, const std::string & frame_id, double duration)
{
	int root = getRootNode(p);
	rootNode = root;
//	ROS_INFO("root: %d", root);
	// publish the root node as red sphere
	coord c; double r;
	GetNodeData(root, &c, &r);
	publishSphere(marker_pub, root, c, 0.05, Color::RED, frame_id, duration);
}

void Poly2VdConverter::publish_wmat(ros::Publisher & marker_pub, const std::string & frame_id, double duration)
{
	// prepare Markers for both "ordinary" (non-frontier based)...
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
	// ... and frontier-based edges
	visualization_msgs::Marker wmat_f_marker;
	wmat_f_marker.header.frame_id = frame_id;
	wmat_f_marker.header.stamp = ros::Time::now();
	wmat_f_marker.ns = "wmatF";
	wmat_f_marker.action = visualization_msgs::Marker::ADD;
	wmat_f_marker.pose.orientation.w = 1.0;
	wmat_f_marker.id = 0;
	wmat_f_marker.lifetime = ros::Duration(duration);
	wmat_f_marker.type = visualization_msgs::Marker::LINE_LIST;
	wmat_f_marker.scale.x = 0.25;
	wmat_f_marker.color.g = 0.5f;
	wmat_f_marker.color.b = 1.0f;
	wmat_f_marker.color.a = 1.0;

	using namespace std;

	bool inNodes[GetNumberOfNodes()];
	for (int i = 0; i < GetNumberOfNodes(); i++) inNodes[i] = false;
	BFS(rootNode,inNodes);
	
	geometry_msgs::Point p;
	for (int e = 0;  e < GetNumberOfEdges(); e++) {

		coord c; double r;
		
		if (!IsWmatEdge(e)) {
			continue;
		}

		if(!inNodes[GetStartNode(e)] || !inNodes[GetEndNode(e)])
			continue;

		GetNodeData(GetStartNode(e), &c, &r);
		p.x = UnscaleX(c.x);
		p.y = UnscaleY(c.y);
		if (isFrontierBasedEdge(e)) {
			wmat_f_marker.points.push_back(p);
		} else {
			wmat_marker.points.push_back(p);
		}
		GetNodeData(GetEndNode(e), &c, &r);
		p.x = UnscaleX(c.x);
		p.y = UnscaleY(c.y);
		if (isFrontierBasedEdge(e)) {
			wmat_f_marker.points.push_back(p);
		} else {
			wmat_marker.points.push_back(p);
		}
	}

	bool cNodes[GetNumberOfNodes()];
	findCriticalNodes(cNodes,inNodes);

	coord c; double r;

	for (int n = 0;  n < GetNumberOfNodes(); n++) {
		if(cNodes[n]){
			GetNodeData(n, &c, &r);
			publishSphere(marker_pub, n, c, 0.05, Color::BLUE, frame_id, duration);
		}
	}
	
	marker_pub.publish(wmat_marker);
	marker_pub.publish(wmat_f_marker);
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
		p2vd.publish_wmat_deg2_nodes(marker_pub, "/odom", 5.0);

		r.sleep();
 	}
}

#endif

/* ************ Export of VD to Dot file for Graphviz **************** */

/*  shuffle: true means "shuffle (almost) incident nodes" */
void outputNodeForDot(std::ofstream &fout, int n, bool shuffle, bool shrink)
{
	// shuffle the node position if needed
	coord c; double r;
	GetNodeData(n, &c, &r);
	if (shuffle && hasCloseNeighbour(n)) {
		coord d = determineNodeDisplacement(n);
		c.x += d.x;
		c.y += d.y;
	} else {
	}

	// output
	using namespace std;
	string otherparams = "fontsize=3,fixedsize=true,width=0.1,height=0.075,penwidth=0.5";
	if (shrink) {
		fout << "\t" << setw(2) << right << n << " [ pos=\"" << setw(18) << left << (coordToString(c, false) + "\"")
			<< ("," + otherparams) << " ];" << endl;
	} else {
		fout << "\t" << setw(2) << right << n << " [ pos=\"" << setw(18) << left << (coordToString(c, false) + "\"")
			<< ",label=\"" << setw(2) << n << " r=" << setprecision(3) << UnscaleV(r) << "\""
			<< " ];" << endl;
	}
}

void outputEdgeForDot(std::ofstream &fout, int e, bool shrink)
{
	using namespace std;

	// color
	string color = "black";
	if (IsWmatEdge(e)) color = "blue";
	if (isEdgeDefinedByDummyPoint(e)) color = "gold";

	// output
	fout << "\t" << setw(2) << right << GetStartNode(e) << " -- " << setw(2) << right<< GetEndNode(e)
		<< "\t[ color=" << setw(6) << left << (color + ",") << (shrink?"penwidth=0.5,fontsize=2,":"")
		<< "label=\"" << setw(4) << left << (to_string(e) + "\"") << " ]" << ";";
	if (isEdgeDefinedByDummyPoint(e)) {
		fout << "   /* " << edgeDefiningSitesToString(e) << " */";
	}
	fout << endl;
}

void Poly2VdConverter::exportVdToDot(const std::string &fileName, bool shuffle, bool shrink)
{
	using namespace std;

	ofstream fout(fileName.c_str());
	fout << "graph vd {" << endl << endl;

	if (!shrink) fout << "\toverlap=\"false\"" << endl << endl;

	// first four nodes are dummy-point-related
	for (int n = 4;  n < GetNumberOfNodes(); n++) {
		if (GetNodeStatus(n) != DELETED && GetNodeStatus(n) != MISC) {
			outputNodeForDot(fout, n, shuffle, shrink);
		}
	}
	fout << endl;
	for (int e = 0;  e < GetNumberOfEdges(); e++) {
		// first four nodes are dummy-point-related
		if (GetStartNode(e) > 3 && GetEndNode(e) > 3) {
			outputEdgeForDot(fout, e, shrink);
		}
	}

	fout << endl << "}" << endl;
	fout.close();
}

/* ********************** performTests() ***************************** */

void testContrastCompare()
{
	std::cout << "\tTesting contrastCompare()" << std::endl;
	// contrastCompare supports only positive number copmarison for now
	double x = 1;

	assert(contrastCompare(0.999*COMPARISON_RATIO*x, x) < 0);
	assert(contrastCompare(1.111*COMPARISON_RATIO*x, x) == 0);
	assert(contrastCompare(x, 0.999*COMPARISON_RATIO*x) > 0);
	assert(contrastCompare(x, 1.111*COMPARISON_RATIO*x) == 0);
	assert(contrastCompare(x, x) == 0);

	assert(contrastCompare(0.0, 0.00000000001) < 0);
	assert(contrastCompare(0.00000000001, 0.0) > 0);
}

void performTests()
{
	std::cout << "Performing tests:" << std::endl;

	testContrastCompare();

	std::cout << "Tests passed." << std::endl;
}


/* **************************** main() ******************************* */

#ifdef POLY2VD_STANDALONE

/**
 * The main() function serves my experimental purposes
 */
int main ( int argc, char *argv[] )
{
	using namespace std;

	// performTests();

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

	if (EXPORT2DOT) p2vd.exportVdToDot("/tmp/vd.dot", SHUFFLE, SHRINK);
#ifndef POLY2VD_STANDALONE
	if (PUBLISH_ROS) publish_result(argc, argv, p2vd);
#endif

	return EXIT_SUCCESS;
}				/* ----------  end of function main  ---------- */

#endif 			/* ----- #ifndef POLY2VD_STANDALONE ----------- */

// vi:ai:sw=4 ts=4 sts=0 tw=120
