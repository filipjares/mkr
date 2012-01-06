/*
 * ===========================================================================
 *
 *       Filename:  main.cpp
 *
 *    Description:  An attempt to use VRONI as a library
 *
 *        Version:  1.0
 *        Created:  01/05/2012 07:12:55 PM
 *       Compiler:  gcc
 *
 *         Author:  Filip Jares (fj), filipjares@post.cz
 *
 * ===========================================================================
 */
#include <stdio.h>
#include <stdlib.h>
#include "clipper.hpp"

#define HAVE_BOOL	// boolean defined in types.h, included by dvi_graphics_header.h
#define MAT		// API_ComputeWMAT in ext_appl_inout.h

#include "dvi_graphics_header.h"	// beacause of API_InitializeProgram()
#include "ext_appl_inout.h"

boolean poly2VD(void)
{
	boolean input_ok = false;

	API_InitializeProgram();
	API_FileInput("/home/filip/fel/MKR-cviceni/util/" 
			"fiddling_with_vroni/data/map_from_the_paper.dxf",
			&input_ok);
	if (!input_ok) {
		printf("Could not load data file\n");
		return EXIT_FAILURE;
	}

	API_ComputeVD(
		false,    /* save input data to file?     */
		true,     /* first call for this data?    */
		false,    /* don't measure time           */
		3,        /* scale factor for bounding box; default: 1 */
		0,        /* sampling factor              */
		0,        /* approximation factor for circular arcs */
		"",       /* name of the output file;     */
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
		false);   /* shall we clean up the data prior to the VD computation? */

	// TODO clean YES

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

	char o_file[] = "/tmp/my_ma_output.txt";
	API_OutputMA(o_file);
	// API_ResetAll();

	API_TerminateProgram();
	return EXIT_SUCCESS;
}				/* ----------  end of function poly2VD -------- */

int main ( int argc, char *argv[] )
{
	return poly2VD();
}				/* ----------  end of function main  ---------- */
