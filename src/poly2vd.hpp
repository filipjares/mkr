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

/* ********************** Poly2VdConverter includes ****************** */

#include "GraphMeta.hpp"
#include "VdPublisher.hpp"

/* ********************** Poly2VdConverter class ********************* */

namespace poly2vd {

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
	void prepareNewInput(in_segs * segs, unsigned int size);

	void prepareNewInput(char * const fileName);

	void convert();

	void doTheSearch(const coord & start);
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
