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
#include "GraphMeta.hpp"
#include "Color.hpp"
#include "VroniUtils.hpp"

/* ************ ROS includes (other than in poly2vd.hpp) ************* */

#ifndef POLY2VD_WITHOUT_ROS

// ROS Messages
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Our own helper class
#include "VdPublisher.hpp"

#endif

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

/* ******************* Constants ************************************* */

const bool SHRINK = false;
const bool SHUFFLE = true;
const bool EXPORT2DOT = false;
const bool PUBLISH_ROS = true;

/* *************** Utility functions (non-Vroni related) ************* */

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

/* ********************** "Publisher" functions ********************** */

#ifndef POLY2VD_WITHOUT_ROS

static void publish_input_data(ros::Publisher & marker_pub, std::string frame_id, double duration)
{
	// prepare the Marker - non-frontier edge
	visualization_msgs::Marker input_marker;
	input_marker.header.frame_id = frame_id;
	input_marker.header.stamp = ros::Time::now();
	input_marker.ns = "non-frontier-input";
	input_marker.action = visualization_msgs::Marker::ADD;
	input_marker.pose.orientation.w = 1.0;
	input_marker.id = 1;
	input_marker.lifetime = ros::Duration(duration);
	input_marker.type = visualization_msgs::Marker::LINE_LIST;
	input_marker.scale.x = 2;
	input_marker.color.r = 1.0;
	input_marker.color.a = 1.0;

	// prepare the Marker - frontier edge
	visualization_msgs::Marker frontier_marker;
	frontier_marker.header.frame_id = frame_id;
	frontier_marker.header.stamp = ros::Time::now();
	frontier_marker.ns = "frontier-input";
	frontier_marker.action = visualization_msgs::Marker::ADD;
	frontier_marker.pose.orientation.w = 1.0;
	frontier_marker.id = 1;
	frontier_marker.lifetime = ros::Duration(duration);
	frontier_marker.type = visualization_msgs::Marker::LINE_LIST;
	frontier_marker.scale.x = 2;
	frontier_marker.color.b = 1.0;
	frontier_marker.color.a = 1.0;

	using namespace std;

	//cout << "Number of input segments: " << num_segs << endl;

	geometry_msgs::Point p1, p2;
	for(int i = 0; i < num_segs; i++) { // num_segs is internal Vroni variable
		p1.x = UnscaleX(GetSegStartCoord(i).x);
		p1.y = UnscaleY(GetSegStartCoord(i).y);
		p2.x = UnscaleX(GetSegEndCoord(i).x);
		p2.y = UnscaleY(GetSegEndCoord(i).y);
		if (GetExtApplSeg(i).isFrontier) {
			frontier_marker.points.push_back(p1);
			frontier_marker.points.push_back(p2);
		} else {
			input_marker.points.push_back(p1);
			input_marker.points.push_back(p2);
		}
	}

	marker_pub.publish(input_marker);
	marker_pub.publish(frontier_marker);
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
		VdPublisher vdPub(marker_pub, frame_id, duration);
		vdPub.publishSphere(candidate, c_candidate, r_candidate, Color::BLUE);
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
			} else {
				n = GetOtherNode(e, n);
				r = GetNodeParam(n);
				if (r > ZERO) {
					at_boundary = false;
				}
				else {
					++e;
				}
			}
		} else {
			++e;
		}
	}

	if (at_boundary) {
		return -1;
	} else {
		return n;
	}
}

void addTheOtherNodeIfAppropriate(int edge, int sourceNode, GraphMeta & graph, VdPublisher & vdPub)
{
	int otherNode = GetOtherNode(edge, sourceNode);
	if (IsWmatEdge(edge)) {
		if (!graph.isEdgeUsed(edge)) {
			graph.setEdgeUsed(edge);
			vdPub.appendEdge(edge);
		}
		if (!graph.isNodeClosed(otherNode)) {
			graph.setNodeClosed(otherNode);
			graph.setPrevious(otherNode, sourceNode, edge); // FIXME: use euclidean length to determine the shortest path
			if (GetNodeParam(otherNode) > ZERO) {
				graph.addToOpenList(otherNode);
			}
		}
	} else {
		// check for incident nodes and report such a situation
		if (areNodesEqual(sourceNode, otherNode)) {
			std::cerr << "Nodes " << sourceNode << " and " << otherNode << " are equal"
				<< std::endl;
		}
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
			// FIXME: filling the open list even with the closed nodes
			open.push_back(GetStartNode(e1));
		}
		if(GetNodeParam(GetEndNode(e1)) != 0){
			// FIXME: filling the open list even with the closed nodes
			open.push_back(GetEndNode(e1));
		}

		int e_ccw = GetCCWEdge(e1, n);				// get the second one
		nodes[GetStartNode(e_ccw)] = true;
		nodes[GetEndNode(e_ccw)] = true;
		if(GetNodeParam(GetStartNode(e_ccw)) != 0){
			// FIXME: filling the open list even with the closed nodes
			open.push_back(GetStartNode(e_ccw));
		}
		if(GetNodeParam(GetEndNode(e_ccw)) != 0){
			// FIXME: filling the open list even with the closed nodes
			open.push_back(GetEndNode(e_ccw));
		}

		int e_cw = GetCWEdge(e1, n);
		if (e_cw != e_ccw){
			nodes[GetStartNode(e_cw)] = true;
			nodes[GetEndNode(e_cw)] = true;
			if(GetNodeParam(GetStartNode(e_cw)) != 0){
			// FIXME: filling the open list even with the closed nodes
				open.push_back(GetStartNode(e_cw));
			}
			if(GetNodeParam(GetEndNode(e_cw)) != 0){
			// FIXME: filling the open list even with the closed nodes
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
		// FIXME: why that? add comment
		if(abs(c.x) >= 1 || abs(c.y) >= 1)
			BFS(GetStartNode(e),nodes);

		// FIXME: Is it possible to avoid doing the BFS twice?
		GetNodeData(GetEndNode(e), &c, &r);
		// FIXME: why that? add comment
		if(abs(c.x) >= 1 || abs(c.y) >= 1)
			BFS(GetEndNode(e),nodes);
	}
}

bool rootNodeNotInsideHole(int root)
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
			// FIXME: filling the open list even with the closed nodes
			open.push_back(GetStartNode(e1));
		}
		if(GetNodeParam(GetEndNode(e1)) != 0){
			// FIXME: filling the open list even with the closed nodes
			open.push_back(GetEndNode(e1));
		}

		int e_ccw = GetCCWEdge(e1, n);				// get the second one
		if(isOuterBasedEdge(e_ccw))
			return true;		
		if(GetNodeParam(GetStartNode(e_ccw)) != 0){
			// FIXME: filling the open list even with the closed nodes
			open.push_back(GetStartNode(e_ccw));
		}
		if(GetNodeParam(GetEndNode(e_ccw)) != 0){
			// FIXME: filling the open list even with the closed nodes
			open.push_back(GetEndNode(e_ccw));
		}

		int e_cw = GetCWEdge(e1, n);
		if (e_cw != e_ccw){
			if(isOuterBasedEdge(e_cw))
				return true;		
			if(GetNodeParam(GetStartNode(e_cw)) != 0){
				// FIXME: filling the open list even with the closed nodes
				open.push_back(GetStartNode(e_cw));
			}
			if(GetNodeParam(GetEndNode(e_cw)) != 0){
				// FIXME: filling the open list even with the closed nodes
				open.push_back(GetEndNode(e_cw));
			}
		}
		
	}
	return false;
}

int getNearestRootNode(const coord &p)
{
	int root = -1;
	
	double min_dist = std::numeric_limits<double>::max();

	for (int e = 0;  e < GetNumberOfEdges(); e++) {
		if (IsWmatEdge(e)) {
			coord c, cu;
			double r;
			GetNodeData(GetStartNode(e), &c, &r);
			cu.x = UnscaleX(c.x);
			cu.y = UnscaleY(c.y);
			double dist = coordDistance(p,cu);
			if (dist < min_dist && r > ZERO) {
				root = GetStartNode(e);
				min_dist = dist;
			}

			GetNodeData(GetEndNode(e), &c, &r);
			cu.x = UnscaleX(c.x);
			cu.y = UnscaleY(c.y);
			dist = coordDistance(p,cu);
			if (dist < min_dist && r > ZERO) {
				root = GetEndNode(e);
				min_dist = dist;
			}
		}
	}

	assert(root != -1);
	return root;
}

/** Find a node which is not on the boundary and is inside polygon */
int getRootNode(const coord & p)
{
	bool outNodes[GetNumberOfNodes()];
	for (int i = 0; i < GetNumberOfNodes(); i++) outNodes[i] = false;
	markOutNodes(outNodes);

	coord c,cu;
	double r, dist;
	int root;
	do {
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
		// FIXME: add comment? marking the root node as outer node
		// just in order not to check it again?
		outNodes[root] = true;
	} while(!rootNodeNotInsideHole(root));

	return root;
}

void Poly2VdConverter::doTheSearch(const coord & start, ros::Publisher & marker_pub, const std::string & frame_id, double duration)
{
	VdPublisher vdPub(marker_pub, frame_id, duration);

	using namespace std;

	// get the rootNode
	// int root = getRootNode(start);
	int root = getNearestRootNode(start);
	assert(root >= 0 && root < GetNumberOfNodes());

	// publish the root node as red sphere
	coord c; double r;
	GetNodeData(root, &c, &r);
	vdPub.publishSphere(root, c, r, Color::RED);

	// prepare open and closed "lists" and other graph metadata:
	GraphMeta graph(GetNumberOfNodes(), GetNumberOfEdges());

	graph.addToOpenList(root);
	graph.setNodeClosed(root);

	while (!graph.isOpenListEmpty()) {
		int n = graph.getFirstNodeFromOpenList();

		// each Vroni's node has at most three incident edges
		int e1 = GetIncidentEdge(n);				// get the first one
		addTheOtherNodeIfAppropriate(e1, n, graph, vdPub);

		int e_ccw = GetCCWEdge(e1, n);				// get the second one
		addTheOtherNodeIfAppropriate(e_ccw, n, graph, vdPub);

		int e_cw = GetCWEdge(e1, n);
		if (e_cw != e_ccw) addTheOtherNodeIfAppropriate(e_cw, n, graph, vdPub);
	}

	vdPub.publishEdges();
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

void Poly2VdConverter::publish_root(ros::Publisher & marker_pub, const coord & start, const std::string & frame_id, double duration)
{
	int root = getRootNode(start);
	rootNode = root;
//	ROS_INFO("root: %d", root);
	// publish the root node as red sphere
	coord c; double r;
	GetNodeData(root, &c, &r);
	VdPublisher vdPub(marker_pub, frame_id, duration);
	vdPub.publishSphere(root, c, 0.05, Color::RED);
}

/* Sends single message with input and output data */
void publish_result( int argc, char *argv[], Poly2VdConverter & p2vd )
{
	// FIXME: retrieve start coords from the command line arguments
	coord start;
	start.x = 105.0;
	start.y = 163.0;

	// init ros
	ros::init(argc, argv, "poly2vd");
	ros::NodeHandle n;
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

	ros::Rate r(1);

	while (ros::ok())
	{
		// publish both input segments and output wmat data
		publish_input_data(marker_pub, "/odom", 5.0);
		p2vd.publish_wmat_deg2_nodes(marker_pub, "/odom", 5.0);
		p2vd.doTheSearch(start, marker_pub, "/odom", 5.0);

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
#ifndef POLY2VD_WITHOUT_ROS
	if (PUBLISH_ROS) publish_result(argc, argv, p2vd);
#endif

	return EXIT_SUCCESS;
}				/* ----------  end of function main  ---------- */

#endif 			/* ----- #ifndef POLY2VD_STANDALONE ----------- */

// vi:ai:sw=4 ts=4 sts=0 tw=120
