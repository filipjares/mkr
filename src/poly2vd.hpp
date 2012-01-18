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

/* ********************** VRONI includes ***************************** */

#define HAVE_BOOL	// boolean defined in types.h, included by dvi_graphics_header.h
#define MAT		// API_ComputeWMAT in ext_appl_inout.h

#include "dvi_graphics_header.h"	// beacause of API_InitializeProgram()

// access to VRONI's internal data
#include "fpkernel.h"			// because of the double_arg macro
#include "vronivector.h"
#include "defs.h"

// ext_appl_inout.h has to be included after "defs.h" (which includes coord.h)
#include "ext_appl_inout.h"

#undef ZERO // both src/consts.h and VRONI_6.0/src/consts.H define ZERO (differently)

/* ********************** ROS includes ******************************* */

#include <ros/ros.h>

/* ********************** Poly2VdConverter class ********************* */

class Poly2VdConverter
{
private:
	bool input_prepared;
public:
	Poly2VdConverter();

	~Poly2VdConverter();

	void prepareNewInput(in_segs * segs, unsigned int size);

	void prepareNewInput(char * const fileName);

	void convert();

	void publish_wmat(ros::Publisher & marker_pub, std::string frame_id, double duration);

	void publish_wmat_deg2_nodes(ros::Publisher & marker_pub, std::string frame_id, double duration);
};

#endif   /* ----- #ifndef POLY2VD_H_INC  ----- */
