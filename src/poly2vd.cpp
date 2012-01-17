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


static std::string site_type_names[] = {"SEG", "ARC", "PNT", "VDN", "VDE", "DTE", "CCW", "CW", "UNKNOWN" };

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

	visualization_msgs::Marker node_marker;
	node_marker.header.frame_id = frame_id;
	node_marker.header.stamp = ros::Time::now();
	node_marker.ns = "nodes";
	node_marker.action = visualization_msgs::Marker::ADD;
	node_marker.pose.orientation.w = 1.0;
	node_marker.id = 2;
	node_marker.lifetime = ros::Duration(duration);
	node_marker.type = visualization_msgs::Marker::SPHERE_LIST;
	node_marker.scale.x = 0.5;
	node_marker.scale.y = 0.5;
	node_marker.scale.z = 0.5;
	node_marker.color.b = 1.0;
	node_marker.color.a = 1.0;
	
	using namespace std;
	using namespace imr::dijkstra;
	unsigned int size = GetNumberOfEdges(); 
	for (unsigned int e = 0;  e < size;  ++e) {
	      SetEdgeFlagNew(e, true);
	}
	int e1;
	int start;
	int end;
	double t;
	coord c; double r;
	geometry_msgs::Point p;
	CHeap<double> heap(size);
	int start_edge = 0;
	for (unsigned int e = size/2;  e < size;  ++e) {
		if(!IsWmatEdge(e))
			continue;
		start = GetStartNode(e);
		end = GetEndNode(e);
		t = GetNodeParam(end);
		if(t < 1.0 && t > 0.0) {
			start_edge = e;
			break;
		}	
	}

	//method for finding edge inside polygon
	//must ensure that the first edge doesnt lie on the boundary -> otherwise the algorithm will stop
	//can be handled by inserting some initial loop testing it
	heap.add(start_edge,start_edge);	
	long u = heap.getFirst();
	SetEdgeFlagNew(u, false);
	cout << "NEW RUN -----------------" << endl;
	while(u != -1)	{	//while HEAP is not EMPTY 
		start = GetStartNode(u);
		end = GetEndNode(u);
		t = GetNodeParam(end);
		cout << "Processing edge: " << u << "radius: " << t << endl;
	
		if(IsWmatEdge(u)) {
			GetNodeData(start, &c, &r);
			p.x = UnscaleX(c.x);
			p.y = UnscaleY(c.y);
			wmat_marker.points.push_back(p);
			if(IsDeg2Node(start)) {
				node_marker.points.push_back(p);
			}
			
			GetNodeData(end, &c, &r);
			p.x = UnscaleX(c.x);
			p.y = UnscaleY(c.y);
			wmat_marker.points.push_back(p);
			if(IsDeg2Node(end)) {
				node_marker.points.push_back(p);
			}
		}
	
		if(t == 0.0) {	//terminate on the boundary
			u = heap.getFirst();
			continue;
		}	

		e1 = GetCWEdge(u,start);
		cout << "CWedge: " << e1 << " new: " << GetEdgeFlagNew(e1) << " isWmat: " << IsWmatEdge(e1) << endl;
		if(GetEdgeFlagNew(e1) && IsWmatEdge(e1)) {
			heap.add(e1,e1);	
			SetEdgeFlagNew(e1, false);
		}
		e1 = GetCCWEdge(u,start);
		cout << "CCWedge: " << e1 << " new: " << GetEdgeFlagNew(e1) << " isWmat: " << IsWmatEdge(e1) << endl;
		if(GetEdgeFlagNew(e1) && IsWmatEdge(e1)) {
			heap.add(e1,e1);	
			SetEdgeFlagNew(e1, false);
		}
		e1 = GetCWEdge(u,end);
		cout << "CWedge: " << e1 << " new: " << GetEdgeFlagNew(e1) << " isWmat: " << IsWmatEdge(e1) << endl;
		if(GetEdgeFlagNew(e1) && IsWmatEdge(e1)) {
			heap.add(e1,e1);	
			SetEdgeFlagNew(e1, false);
		}
		e1 = GetCCWEdge(u,end);
		cout << "CCWedge: " << e1 << " new: " << GetEdgeFlagNew(e1) << " isWmat: " << IsWmatEdge(e1) << endl;
		if(GetEdgeFlagNew(e1) && IsWmatEdge(e1)) {
			heap.add(e1,e1);	
			SetEdgeFlagNew(e1, false);
		}
		u = heap.getFirst();
	}
	marker_pub.publish(wmat_marker);
	marker_pub.publish(node_marker);
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

		ros::spinOnce();
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
