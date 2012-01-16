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
#include "clipper.hpp"
#include "poly2vd.hpp"

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

/* ********************** Private methods **************************** */

int getWmatEdgeCount(void)
{
	int k = 0;
	for (int e = 0;  e < GetNumberOfEdges(); e++) {
		if (IsWmatEdge(e)) k++;
	}
	return k;
}

/**
 * The main() function serves my experimental purposes
 */
int main ( int argc, char *argv[] )
{
//	// Prepare the polygon
//	ClipperLib::Polygons poly(1);
//	poly[0].push_back(ClipperLib::IntPoint(180,200));
//	poly[0].push_back(ClipperLib::IntPoint(260,200));
//	poly[0].push_back(ClipperLib::IntPoint(260,150));
//	poly[0].push_back(ClipperLib::IntPoint(180,150));
//	
//	// Count its size
//	unsigned int size = 0;
//	for(unsigned int i = 0; i < poly.size(); i++)
//		size += poly[i].size();
//
//	// Convert the polygon to array of segments
//	in_segs segs[size];
//	convertPoly2Segs(poly, segs, size);


	/* ************************************ */

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

	cout << "edges size: " << edges.size() << endl;
	cout << "WMAT edges count: " << getWmatEdgeCount() << endl;

	return EXIT_SUCCESS;
}				/* ----------  end of function main  ---------- */

// vi:ai:sw=4 ts=4 sts=0
