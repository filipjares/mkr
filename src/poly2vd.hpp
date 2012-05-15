/*
 * ===========================================================================
 *
 *       Filename:  poly2vd.hpp
 *
 *    Description:  Convenient interface to VRONI - Header file
 *
 *        Version:  1.0
 *        Created:  01/15/2012 06:18:56 PM
 *       Compiler:  gcc
 *
 *        Authors:  Filip Jares (fj), filipjares@gmail.com
 *                  Tomas Juchelka (tj), tomas.juchelka@gmail.com
 *
 * ===========================================================================
 */
#ifndef  POLY2VD_H_INC
#define  POLY2VD_H_INC

/* ********************** ROS includes ******************************* */

#ifndef POLY2VD_WITHOUT_ROS

#include <ros/ros.h>

#endif

/* ********************** std includes ******************************* */

#include <list>
#include <vector>

/* ********************** Poly2VdConverter includes ****************** */

#include "GraphMeta.hpp"
#include "VdPublisher.hpp"

/* ********************** Poly2VdConverter class ********************* */

namespace poly2vd {

struct Node
{
	coord location;
	double radius;
	Node(const coord & c, double r): location(c.x, c.y), radius(r) {}
};

/**
 * Poly2VdConverter object processes a polygonal map (input) and using the
 * Vroni library it acquires a Voronoi diagram containing information about
 * the medial axis; on that Voronoi data it can perform search for
 * "critical nodes".
 *
 * Input polygonal map is made of line segments. Each of these segments
 * contains a flag designating whether this segment is a frontier or not.
 * Information about frontiers is used in the search for critical nodes.
 *
 * Lifecycle of the Poly2VdConverter object is described below. Instance
 * may be used repeatedly to perform a search on different data sets.
 * Described methods have to be called in the specified order.
 *
 * - Data is fed to the object using the
 *   prepareNewInput(in_segs *, unsigned int)
 *   method.  (It is overloaded and has two variants, one
 *   accepting array of in_segs type and the other, experimental one,
 *   accepting name of input file.)
 * - The input is processed when you call the convert() method. Vroni is
 *   used to obtain a Voronoi diagram for the provided input segments.
 * - Call findCriticalNodes(const coord &) in order to perform the search
 *   for critical nodes.  The provided coordinate represents position that
 *   is used to find a voronoi diagram node that is used as a "root" for
 *   the search.
 * - Call the getCriticalNodes() method which creates a new vector
 *   containing found critical nodes and returns a pointer to the vector.
 *
 * @todo choose a better name for the class
 *
 */
class Poly2VdConverter
{
private:

	enum {
		NO_INPUT,
		INPUT_PREPARED,
		VD_READY,
		CRITICAL_POINTS_READY
	} state;

	int rootNode;

	std::list<int> criticalNodes;

	VdPublisher vdPub;

public:
	Poly2VdConverter();

	~Poly2VdConverter();
#ifndef POLY2VD_WITHOUT_ROS
	void usePublisher(const ros::Publisher * marker_pub, const std::string & frame_id, double duration);
#endif
	/**
	 * Feeds the input segments to Vroni.
	 *
	 * @param segs pointer to an array of segments
	 * @param size length of the array
	 */
	void prepareNewInput(in_segs * segs, unsigned int size);

	void prepareNewInput(char * const fileName);

	/**
	 * Exploits the Vroni library in order to obtain Voronoi diagram
	 * of the input line segments.
	 */
	void convert();

	/**
	 * Performs the search for critical nodes. You have to call
	 * convert() first.
	 *
	 * @param start position that is used to find the "nearest"
	 *				node of the medial axis; that node will be used
	 *				as a "root" during the search
	 */
	void findCriticalNodes(const coord & start);

	/**
	 * Creates new instance of a vector of coords and returns pointer to
	 * that vector. You can call this method only after you called
	 * findCriticalNodes() for the current data set.
	 */
	std::vector<Node> * getCriticalNodes();
#ifndef POLY2VD_WITHOUT_ROS
	void publishResults();
#endif
	void exportVdToDot(const std::string &fileName, bool shuffle, bool shrink);

private:
	void addTheOtherNodeIfAppropriate(int edge, int sourceNode, GraphMeta & graph);
	void exploreCriticalNodesOnPath(int goalNode, GraphMeta & graph);
};

}        /* ----- namespace poly2vd ----- */

#endif   /* ----- #ifndef POLY2VD_H_INC  ----- */
