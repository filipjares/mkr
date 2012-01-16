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
#include <vector>
#include <assert.h>
#include "poly2vd.hpp"


// These macros are defined in Vroni's basic.h; but I had problems including it

#define ScaleX(xc)  (scale_factor * ((xc) - shift.x))

#define ScaleY(yc)  (scale_factor * ((yc) - shift.y))

#define ScaleV(value)  ((value) * scale_factor)

#define UnscaleX(xc)  (assert(scale_factor > 0.0), (xc) / scale_factor + shift.x)

#define UnscaleY(yc)  (assert(scale_factor > 0.0), (yc) / scale_factor + shift.y)

#define UnscaleV(value)  (assert(scale_factor > 0.0), (value) / scale_factor)

/* ********************** Experimental includes ********************** */

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

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

void printCoord(const coord & coord)
{
	using namespace std;

	std::cout << UnscaleX(coord.x) << ", " << UnscaleY(coord.y) << std::endl;
}

void publish_input_data(ros::Publisher & marker_pub, visualization_msgs::Marker & prepared_marker)
{
	using namespace std;

//	cout << "Number of input segments: " << num_segs << endl;

	geometry_msgs::Point p;
	for(int i = 0; i < num_segs; i++) { // num_segs is internal Vroni variable
		p.x = UnscaleX(GetSegStartCoord(i).x);
		p.y = UnscaleY(GetSegStartCoord(i).y);
		prepared_marker.points.push_back(p);
		p.x = UnscaleX(GetSegEndCoord(i).x);
		p.y = UnscaleY(GetSegEndCoord(i).y);
		prepared_marker.points.push_back(p);
	}

	marker_pub.publish(prepared_marker);
}

static bool printed = false;

static std::string site_type_names[] = {"SEG", "ARC", "PNT", "VDN", "VDE", "DTE", "CCW", "CW", "UNKNOWN" };

void publish_wmat(ros::Publisher & marker_pub, visualization_msgs::Marker & prepared_marker)
{
	using namespace std;

	int last_valid_pnt_ix = num_pnts - 3; // num_pts is Vroni's internal variable
	if (!printed) cout << "count: " << num_pnts << endl;

	geometry_msgs::Point p;
	for (int e = 0;  e < GetNumberOfEdges(); e++) {
		int s1, s2;
		t_site t1, t2;
		GetLftSiteData(e, &s1, &t1);
		GetRgtSiteData(e, &s2, &t2);
		if (s1 == 0 || s1 == 1 || s1 > last_valid_pnt_ix
				|| s2 == 0 || s2 == 1 || s2 > last_valid_pnt_ix) {
			if (!printed) cout << "skipping this edge: s1 = " << s1
				<< " [" << site_type_names[t1] << "], s2 = " << s2
				<< " [" << site_type_names[t2] << "]" << endl;
			continue;
		}

		coord c; double r;
		GetNodeData(GetStartNode(e), &c, &r);
		if (!printed) {
			cout << s1 << ", " << t1 << "\t";
			printCoord(c);
		}
		p.x = UnscaleX(c.x);
		p.y = UnscaleY(c.y);
		prepared_marker.points.push_back(p);
		GetNodeData(GetEndNode(e), &c, &r);
		if (!printed) {
			cout << s2 << ", " << t2 << "\t";
			printCoord(c);
			cout << endl;
		}
		p.x = UnscaleX(c.x);
		p.y = UnscaleY(c.y);
		prepared_marker.points.push_back(p);
	}

	printed = true;

	marker_pub.publish(prepared_marker);
}

/* Sends single message with input and output data */
void publish_result( int argc, char *argv[] )
{
	// init ros
	ros::init(argc, argv, "poly2vd");
	ros::NodeHandle n;
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

	ros::Rate r(1);

	while (ros::ok())
	{
		// prepare markers
		visualization_msgs::Marker input_segments, wmat;
		wmat.header.frame_id = input_segments.header.frame_id = "/odom";
		wmat.header.stamp = input_segments.header.stamp = ros::Time::now();
		wmat.ns = "wmat";
		input_segments.ns = "input";
		wmat.action = input_segments.action = visualization_msgs::Marker::ADD;
		wmat.pose.orientation.w = input_segments.pose.orientation.w = 1.0;
		wmat.id = 0;
		input_segments.id = 1;
		wmat.lifetime = input_segments.lifetime = ros::Duration(5);
		wmat.type = input_segments.type = visualization_msgs::Marker::LINE_LIST;
		wmat.scale.x = input_segments.scale.x = 2;
		wmat.color.g = 1.0f;
		input_segments.color.g = input_segments.color.b = 1.0f;
		wmat.color.a = input_segments.color.a = 1.0;

		// publish both input segments and output wmat data
		publish_input_data(marker_pub, input_segments);
		publish_wmat(marker_pub, wmat);

		r.sleep();
 	}
}

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

	publish_result(argc, argv);

	return EXIT_SUCCESS;
}				/* ----------  end of function main  ---------- */

// vi:ai:sw=4 ts=4 sts=0
