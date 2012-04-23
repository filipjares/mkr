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
	bool input_prepared;
	
	int rootNode;
	// TODO:
	bool result_ready;
public:
	Poly2VdConverter();

	~Poly2VdConverter();

	void prepareNewInput(in_segs * segs, unsigned int size);

	void prepareNewInput(char * const fileName);

	void convert();

#ifndef POLY2VD_WITHOUT_ROS
	// FIXME: remove?
	void publish_root(ros::Publisher & marker_pub, const coord & start, const std::string & frame_id, double duration);
	
	// FIXME: remove?
	void publish_wmat_deg2_nodes(ros::Publisher & marker_pub, const std::string & frame_id, double duration);

	void doTheSearch(const coord & start, ros::Publisher & marker_pub, const std::string & frame_id, double duration);
#endif
	void exportVdToDot(const std::string &fileName, bool shuffle, bool shrink);
};

}        /* ----- namespace poly2vd ----- */

#endif   /* ----- #ifndef POLY2VD_H_INC  ----- */
