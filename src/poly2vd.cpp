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
#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <cstring>
#include "clipper.hpp"

#define HAVE_BOOL	// boolean defined in types.h, included by dvi_graphics_header.h
#define MAT		// API_ComputeWMAT in ext_appl_inout.h

#include "dvi_graphics_header.h"	// beacause of API_InitializeProgram()
#include "ext_appl_inout.h"

boolean poly2VD(in_segs * segs, unsigned int size)
{
//	boolean input_ok = false;
	boolean new_input = true;
	API_InitializeProgram();
	/*
	char fileName[]="./util/fiddling_with_vroni/data/map_from_the_paper.dxf";
	API_FileInput(fileName,&input_ok);
	if (!input_ok) {
		printf("Could not load data file\n");
		return EXIT_FAILURE;
	}*/
	API_ArrayInput(0,NULL, size, segs,0,NULL, &new_input);

	char ofile[]="";
	boolean _false = false;
	boolean _true = true;

	API_ComputeVD(
		_false,    /* save input data to file?     */
		_true,     /* first call for this data?    */
		_false,    /* don't measure time           */
		3,        /* scale factor for bounding box; default: 1 */
		0,        /* sampling factor              */
		0,        /* approximation factor for circular arcs */
		ofile,    /* name of the output file;     */
		_false,    /* check for duplicate segs prior to the computation?     */
		_false,    /* compute an approximate VD for circular arcs and        */
					 /*  use it for subsequent operations (such as offsetting) */
		0.0,      /* approximation threshold for  */
					 /* circular arcs; see           */
			  		 /* see ApproxArcsBounded() in   */
			  		 /* in approx.cc; default = 0.0  */
		0.0,      /* approximation threshold for  */
			  		 /* circular arcs; see           */
			  		 /* see ApproxArcsBounded() in   */
			  		 /* in approx.cc; default = 0.0  */
		_false,    /* shall we use my heuristic    */
			  		 /* approximation threshold?     */
		_false,    /* compute VD/DT of points only */
		_false,    /* output point VD/DT           */
		ofile,    /* output file for point VD/DT  */
		_false);   /* shall we clean up the data prior to the VD computation? */
//
//	// TODO clean YES
//
	API_ComputeWMAT(
                _false,    /* shall we use my heuristic    */
		          /* for finding nice WMAT        */
		          /* thresholds?                  */
		0.0,      /* angle threshold for WMAT     */
		          /* computation;in radians, out  */
		          /* of the interval [0, pi]      */
		0.0,      /* distance threshold for WMAT  */
		          /* computation                  */
                _false,    /* do you want to time the      */
		          /* computation?                 */
                _false,    /* true if WMAT is to be        */
		          /* computed only on the left    */
		          /* side of input segments       */
                _false);   /* true if WMAT is to be        */
		          /* computed only on the right   */
		          /* side of input segments       */

	std::cout << "PNTS size: " << API_getVD() << std::endl;
	char o_file[] = "./tmp/my_ma_output.txt";
	API_OutputMA(o_file);
	API_ResetAll();
//
	API_TerminateProgram();
	std::cout << "Program finished succesfully." << std::endl;
	std::cout << "Output written to: " << o_file << std::endl;
	return EXIT_SUCCESS;
}				/* ----------  end of function poly2VD -------- */

void convertPoly2Segs(ClipperLib::Polygons & poly, in_segs * s, unsigned int size)
{
	in_segs init;
	init.x1 = 0.0;
	init.x2 = 0.0;
	init.y1 = 0.0;
	init.y2 = 0.0;
	for (unsigned int i = 0; i < size; i++)
		s[i] = init;
	unsigned int k = 0;
	for(unsigned int n = 0; n < poly.size(); n++)	{
		unsigned int sz = poly[n].size();
		for(unsigned int i = 0; i < sz - 1; i++)	{
			s[k].x1 = (double)poly[n][i].X;
			s[k].y1 = (double)poly[n][i].Y;
			s[k].x2 = (double)poly[n][i+1].X;
			s[k].y2 = (double)poly[n][i+1].Y;
			k++;
		}
			s[k].x1 = (double)poly[n][sz-1].X;
			s[k].y1 = (double)poly[n][sz-1].Y;
			s[k].x2 = (double)poly[n][0].X;
			s[k].y2 = (double)poly[n][0].Y;
			k++;
	}	
}

int main ( int argc, char *argv[] )
{
	ClipperLib::Polygons poly(1);
	poly[0].push_back(ClipperLib::IntPoint(180,200));
	poly[0].push_back(ClipperLib::IntPoint(260,200));
	poly[0].push_back(ClipperLib::IntPoint(260,150));
	poly[0].push_back(ClipperLib::IntPoint(180,150));
	
	unsigned int size = 0;
	for(unsigned int i = 0; i < poly.size(); i++)
		size += poly[i].size();

	in_segs segs[size];
	convertPoly2Segs(poly, segs, size);
	std::cout << "Polygon structure after conversion: " << std::endl;
	for(unsigned int i = 0; i < size; i++) {
		std::cout << "x1: " << segs[i].x1 << std::endl; 
		std::cout << "y1: " << segs[i].y1 << std::endl; 
		std::cout << "x2: " << segs[i].x2 << std::endl; 
		std::cout << "y2: " << segs[i].y2 << std::endl; 
		std::cout << "--------------" << std::endl; 
	}
	return poly2VD(segs, size);
	//return 0;
}				/* ----------  end of function main  ---------- */
