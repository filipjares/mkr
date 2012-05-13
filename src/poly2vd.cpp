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
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <list>
#include <algorithm>
#include <assert.h>
#include "poly2vd.hpp"

/* ************ ROS includes (other than in poly2vd.hpp) ************* */

#ifndef POLY2VD_WITHOUT_ROS

// ROS Messages
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Our own helper class
#include "Color.hpp"
#include "GraphMeta.hpp"
#include "VroniUtils.hpp"

#endif

/* ******************* Constants ************************************* */

const bool SHRINK = false;
const bool SHUFFLE = true;
const bool EXPORT2DOT = false;
const bool PUBLISH_ROS = true;

namespace poly2vd {

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

static void removeNodeFromListIfPresent(std::list<int> & lst, int element) {
	std::list<int>::iterator it = find(lst.begin(), lst.end(), element);
	if (it != lst.end()) {
		lst.erase(it);
	}
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
	input_marker.scale.x = VdPublisher::RVIZ_EDGES_WIDTH;
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
	frontier_marker.scale.x = VdPublisher::RVIZ_EDGES_WIDTH;
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

/**
 * Returns true if the clearance radius of the node n is "significantly smaller"
 * than the clearance radius of the nearest neighbour with "significantly
 * different" clearance radius occuring in the direction of the neighbour node
 * n1. In case of the clearance radius of neighbour n1 being "almost equal" to
 * the clearance radius of the node n, "further" neighbours are examined only if
 * n1 (and the eventual further ones) are of WMAT degree 2. If they are of
 * higher degree, false is returned.
 */
bool testNForBeingFuzzyMinimum(int n, int e1, int n1)
{
	double r = GetNodeParam(n);
	double r1 = GetNodeParam(n1);

	char comparison = VroniUtils::contrastCompare(r, r1);
	if (comparison < 0) {
		return true;
	}
	if (comparison > 0) {
		return false;
	}

	// we see that r and r1 are (almost) equal
	
	// is such a situation likely to occur? However the response will be
	// nagative in such a case
	if (!VroniUtils::isDeg2WmatNode(n1)) {
		return false;
	}
	
	int e2 = GetCCWEdge(e1, n1);
	if (!IsWmatEdge(e2)) {
		e2 = GetCCWEdge(e2, n1);
		if (!IsWmatEdge(e2)) {
			std::cerr << "ERROR: node n = " << n <<
				"does not have two incident WMAT edges even though it should"
				<< std::endl;
			exit(1);
		}
	}

	int n2 = GetOtherNode(e2, n1);
	return testNForBeingFuzzyMinimum(n1, e2, n2);
}

/**
 * Determines whether the node n is a critical node. Critical node has to meet
 * following four criteria (mentioning neighbours and degrees, only WMAT edges
 * are considered):
 *
 * <ul>
 * 	<li>it has to be a degree 2 node,</li>
 * 	<li>its clearance radius has to be a local minimum compared to the
 * 		clearance radius of its neighbours (however see the function
 * 		testNForBeingFuzzyMinimum() for details on how neighbours are
 * 		examined),</li>
 * 	<li>it has to have neighbour of degree 3</li>
 * 	<li>it has to divide explored and unexplored subcomponents of the map</li>
 * </ul>
 *
 * Another implicit criterion for the critical node is that its clearance radius
 * it has to have positive (non-zero) clearance radius.
 */
bool examineNode(int n, GraphMeta & graph, VdPublisher & vdPub)
{
	using visualization_msgs::Marker;

	// FIXME: examine only neighbours connected to n by EXAMINED WMAT edges?

	// only degree 2 nodes are critical point candidates
	if (!VroniUtils::isDeg2WmatNode(n)) {
		// std::cout << "It is not a Deg2 node." << std::endl;
		// vdPub.publishPoint(n, Marker::CUBE, GetNodeCoord(n), 0.01, Color::PINK);
		return false;
	}

	// critical node can not have zero clearance radius
	double r = GetNodeParam(n);
	if (r == 0) {
		// std::cout << "It has zero radius." << std::endl;
		// vdPub.publishPoint(n, Marker::CUBE, GetNodeCoord(n), 0.01, Color::VIOLET);
		return false;
	}

	// the node n has (I hope!) exactly two incident WMAT edges, find them
	int e1 = GetIncidentEdge(n);
	if (!IsWmatEdge(e1)) {
		e1 = GetCWEdge(e1, n);
		// if now e1 is not WMAT edge then n does not have
		// two incident WMAT edges
		if (!IsWmatEdge(e1)) {
			std::cerr << "ERROR: node n = " << n <<
				"does not have two incident WMAT edges even though it should"
				<< std::endl;
			exit(1);
		}
	}

	int e2 = GetCWEdge(e1, n);
	if (!IsWmatEdge(e2)) {
		e2 = GetCWEdge(e2, n);
		// if now e1 is not WMAT edge then n does not have
		// two incident WMAT edges
		if (!IsWmatEdge(e2)) {
			std::cerr << "ERROR: node n = " << n <<
				"does not have two incident WMAT edges even though it should"
				<< std::endl;
			exit(1);
		}
	}

	assert(e1 != e2);
	assert(IsWmatEdge(e1) && IsWmatEdge(e2));

	int n1 = GetOtherNode(e1, n);
	int n2 = GetOtherNode(e2, n);

	assert(n1 != n2);

	double r1 = GetNodeParam(n1);
	double r2 = GetNodeParam(n2);

	// Is the clearance radius of the node n a local minimum?
	// At least a fuzzy one? Analyze left and right side separatedly.
	bool isLocalMinimum1 = false;
	bool isLocalMinimum2 = false;

	if (VroniUtils::contrastCompare(r, r1) == 0) {
		isLocalMinimum1 = testNForBeingFuzzyMinimum(n, e1, n1);
	} else {
		isLocalMinimum1 = (VroniUtils::contrastCompare(r, r1) < 0);
	}
	if (VroniUtils::contrastCompare(r, r2) == 0) {
		isLocalMinimum2 = testNForBeingFuzzyMinimum(n, e2, n2);
	} else {
		isLocalMinimum2 = (VroniUtils::contrastCompare(r, r2) < 0);
	}

	// critical node has to be local minimum
	// add recursive search of nodes with "equal" radius
	if ( !isLocalMinimum1 || !isLocalMinimum2 ) {
		// std::cout << "Its clearance radius is not a local minimum (r1, r, r2) =="
		//				<< "(" << r1 << ", " << r << ", " << r2 << ")." << std::endl;
		// vdPub.publishPoint(n, Marker::CUBE, GetNodeCoord(n), 0.01, Color::ORANGE);
		return false;
	}

	// FIXME rename the isDeg3Node() function
	bool hasWmatDeg3Neighbour = VroniUtils::isDeg3Node(n1) || VroniUtils::isDeg3Node(n2);

	if (!hasWmatDeg3Neighbour) {
		// std::cout << "It does not have a deg3 neighbour." << std::endl;
		// vdPub.publishPoint(n, Marker::CUBE, GetNodeCoord(n), 0.01, Color::GREEN);
		return false;
	}

	// all the critical node criteria are satisfied
	return true;
}

void Poly2VdConverter::exploreCriticalNodesOnPath(int goalNode, GraphMeta & graph, VdPublisher & vdPub)
{
	int prevEdge;
	int n = goalNode;
	bool criticalNodeFound = false;

	while ((prevEdge = graph.getPreviousEdge(n)) != -1 && !graph.isFrontierBoundaryNodeSet(prevEdge)) {
		assert(graph.getEdgeStatus(prevEdge) == EXPLORED);

		// mark the explored prevEdge as edge leading from
		// the root to the frontierBoundaryNode edge goalNode
		graph.setFrontierBoundaryNode(prevEdge, goalNode);

		// examine node n for being a critical node
		criticalNodeFound = examineNode(n, graph, vdPub);
		// std::cout << "Node " << n << " is " << (criticalNodeFound?"":"NOT ")
		// 	<< "a critical node." << std::endl;
		if (criticalNodeFound) {
			criticalNodes.push_back(n);
		}
		n = graph.getPreviousNode(n);
		if (criticalNodeFound) {
			break; // only have to remove the rest of critical nodes on this path
		}
	}
	// if a new critical point was found on this path, reove the old ones
	if (criticalNodeFound) {
		while ((prevEdge = graph.getPreviousEdge(n)) != -1) {
			assert(graph.getEdgeStatus(prevEdge) == EXPLORED);

			// is the node n a critical point? if so, remove it
			removeNodeFromListIfPresent(criticalNodes, n);

			n = graph.getPreviousNode(n);
		}
	}
}


/**
 * Using the vdPub VdPublisher, this function publishes a Marker for the
 * specified node n, colour and shape of the marker are determined by the node
 * properties. Useful for testing.
 *
 * \param <n> node id of the node to be marked (published)
 * \param <vdPub> publisher to be used to publish the marker
 */
void experimentallyMarkTheNode(int n, VdPublisher & vdPub)
{
	using visualization_msgs::Marker;

	bool published = true;

	if (GetNodeParam(n) == 0) {
		vdPub.publishPoint(n, Marker::CUBE, GetNodeCoord(n), 0.01, Color::RED);
	} else {
		if (VroniUtils::isDeg3Node(n)) { // FIXME: rename (WMAT)
			if (VroniUtils::isDeg2WmatNode(n)) {
				std::cerr << "ERROR, inconsistence" << std::endl;
				exit(2);
			}
			vdPub.publishPoint(n, Marker::CYLINDER, GetNodeCoord(n), 0.01, Color::YELLOW);
		} else {
			if (VroniUtils::isDeg2WmatNode(n)) {
				vdPub.publishPoint(n, Marker::CYLINDER, GetNodeCoord(n), 0.01, Color::VIOLET);
			} else {
				published = false;
			}
		}
	}

	if (!published) {
		vdPub.publishPoint(n, Marker::CUBE, GetNodeCoord(n), 0.03, Color::ORANGE);
	}
}

void Poly2VdConverter::addTheOtherNodeIfAppropriate(int edge, int sourceNode, GraphMeta & graph, VdPublisher & vdPub)
{
	int otherNode = GetOtherNode(edge, sourceNode);
	if (IsWmatEdge(edge)) {
		if (!graph.isEdgeUsed(edge)) {
			graph.setEdgeUsed(edge);
			vdPub.appendEdge(edge);

			EdgeStatus status;	// edge status to be set

			int prevEdge = graph.getPreviousEdge(sourceNode);
			if (prevEdge != -1 && graph.getEdgeStatus(prevEdge) == FRONTIER) {
				status = FRONTIER;
			} else {
				if (VroniUtils::isFrontierBasedEdge(edge)) {
					status = FRONTIER;
					exploreCriticalNodesOnPath(sourceNode, graph, vdPub);
					vdPub.publishSphere(sourceNode, GetNodeCoord(sourceNode), GetNodeParam(sourceNode)/2.0, Color::BLUE);
					// vdPub.appendPath(sourceNode, graph);
				} else {
					status = EXPLORED;
				}
			}
			graph.setEdgeStatus(edge, status);
		}
		if (!graph.isNodeClosed(otherNode)) {
			graph.setNodeClosed(otherNode);
			// experimentallyMarkTheNode(otherNode, vdPub);
			graph.setPrevious(otherNode, sourceNode, edge); // FIXME: use euclidean length to determine the shortest path
			if (GetNodeParam(otherNode) > ZERO) {
				graph.addToOpenList(otherNode);
			}
		}
	} else {
		// check for incident nodes and report such a situation
		if (VroniUtils::areNodesEqual(sourceNode, otherNode)) {
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
		if(VroniUtils::isOuterBasedEdge(e1))
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
		if(VroniUtils::isOuterBasedEdge(e_ccw))
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
			if(VroniUtils::isOuterBasedEdge(e_cw))
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
			double dist = VroniUtils::coordDistance(p,cu);
			if (dist < min_dist && r > ZERO) {
				root = GetStartNode(e);
				min_dist = dist;
			}

			GetNodeData(GetEndNode(e), &c, &r);
			cu.x = UnscaleX(c.x);
			cu.y = UnscaleY(c.y);
			dist = VroniUtils::coordDistance(p,cu);
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
			dist = VroniUtils::coordDistance(p,cu);
			if(dist < min_dist){
				root = GetStartNode(e);
				min_dist = dist;
			}
	
			GetNodeData(GetEndNode(e), &c, &r);
			cu.x = UnscaleX(c.x);
			cu.y = UnscaleY(c.y);
			dist = VroniUtils::coordDistance(p,cu);
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
	int root = getNearestRootNode(start);	// FIXME: this is faster but is not correct
	assert(root >= 0 && root < GetNumberOfNodes());

	// publish the root node as red sphere
	coord c; double r;
	GetNodeData(root, &c, &r);
	vdPub.publishSphere(root, c, 0.01, Color::RED);

	// prepare open and closed "lists" and other graph metadata:
	GraphMeta graph(GetNumberOfNodes(), GetNumberOfEdges());

	graph.addToOpenList(root);
	graph.setNodeClosed(root);

	// prepare the list of critical nodes (make it empty)
	criticalNodes.clear();

	// cout << "-------- root " << root << endl;

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
	vdPub.publishCriticalNodes(criticalNodes);
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
	if (shuffle && VroniUtils::hasCloseNeighbour(n)) {
		coord d = VroniUtils::determineNodeDisplacement(n);
		c.x += d.x;
		c.y += d.y;
	} else {
	}

	// output
	using namespace std;
	string otherparams = "fontsize=3,fixedsize=true,width=0.1,height=0.075,penwidth=0.5";
	if (shrink) {
		fout << "\t" << setw(2) << right << n << " [ pos=\"" << setw(18) << left << (VroniUtils::coordToString(c, false) + "\"")
			<< ("," + otherparams) << " ];" << endl;
	} else {
		fout << "\t" << setw(2) << right << n << " [ pos=\"" << setw(18) << left << (VroniUtils::coordToString(c, false) + "\"")
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
	if (VroniUtils::isEdgeDefinedByDummyPoint(e)) color = "gold";

	// output
	fout << "\t" << setw(2) << right << GetStartNode(e) << " -- " << setw(2) << right<< GetEndNode(e)
		<< "\t[ color=" << setw(6) << left << (color + ",") << (shrink?"penwidth=0.5,fontsize=2,":"")
		<< "label=\"" << setw(4) << left << (to_string(e) + "\"") << " ]" << ";";
	if (VroniUtils::isEdgeDefinedByDummyPoint(e)) {
		fout << "   /* " << VroniUtils::edgeDefiningSitesToString(e) << " */";
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

	assert(VroniUtils::contrastCompare(0.999*VroniUtils::COMPARISON_RATIO*x, x) < 0);
	assert(VroniUtils::contrastCompare(1.111*VroniUtils::COMPARISON_RATIO*x, x) == 0);
	assert(VroniUtils::contrastCompare(x, 0.999*VroniUtils::COMPARISON_RATIO*x) > 0);
	assert(VroniUtils::contrastCompare(x, 1.111*VroniUtils::COMPARISON_RATIO*x) == 0);
	assert(VroniUtils::contrastCompare(x, x) == 0);

	assert(VroniUtils::contrastCompare(0.0, 0.00000000001) < 0);
	assert(VroniUtils::contrastCompare(0.00000000001, 0.0) > 0);
}

void performTests()
{
	std::cout << "Performing tests:" << std::endl;

	testContrastCompare();

	std::cout << "Tests passed." << std::endl;
}

}				/* ----- namespace poly2vd ----- */

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

	poly2vd::Poly2VdConverter p2vd;
	// p2vd.prepareNewInput(segs, size);
	p2vd.prepareNewInput(argv[1]);
	p2vd.convert();

	cout << "total edges size: " << edges.size() << ", wmat edges: " << poly2vd::VroniUtils::getWmatEdgeCount() << endl;
	cout << "WMAT edges count: " << poly2vd::VroniUtils::getWmatEdgeCount() << endl;
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
