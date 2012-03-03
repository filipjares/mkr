/*
 * ===========================================================================
 *
 *       Filename:  VroniUtils.hpp
 *
 *    Description:  Utility functions helping to cope with Vroni's internal
 *                  data
 *
 *        Version:  1.0
 *        Created:  02/26/2012 09:39:36 PM
 *       Compiler:  gcc
 *
 *         Author:  Filip Jares (fj), filipjares@post.cz
 *                  Tomas Juchelka (tj), tomas.juchelka@gmail.com
 *
 * ===========================================================================
 */

#ifndef  VRONI_UTILS_H_INC
#define  VRONI_UTILS_H_INC

#include "poly2vd.hpp"

/* ******************* Scale / Unscale macros ************************ */
// These macros are defined in Vroni's basic.h; but I had problems including it

#define ScaleX(xc)  (scale_factor * ((xc) - shift.x))

#define ScaleY(yc)  (scale_factor * ((yc) - shift.y))

#define ScaleV(value)  ((value) * scale_factor)

#define UnscaleX(xc)  (assert(scale_factor > 0.0), (xc) / scale_factor + shift.x)

#define UnscaleY(yc)  (assert(scale_factor > 0.0), (yc) / scale_factor + shift.y)

#define UnscaleV(value)  (assert(scale_factor > 0.0), (value) / scale_factor)

/* ******************* Constants ************************************* */

static std::string site_type_names[] = {"SEG", "ARC", "PNT", "VDN", "VDE", "DTE", "CCW", "CW", "UNKNOWN" };

const double COMPARISON_RATIO = 0.96;

/* *************** Utility functions (VRONI-related) ***************** */

static int getWmatEdgeCount(void)
{
	int k = 0;
	for (int e = 0;  e < GetNumberOfEdges(); e++) {
		if (IsWmatEdge(e)) k++;
	}
	return k;
}

static coord roundCoord(const coord &c)
{
	double N = 10.0; coord r;	// FIXME: use constant

	r.x = round(N*c.x)/N;
	r.y = round(N*c.y)/N;
	
	return r;
}

static bool coordCompare(const coord & c1, const coord & c2)
{
	coord r1 = roundCoord(c1);
	coord r2 = roundCoord(c2);

	if (r1.x < r2.x) {
		return true;
	} else {
		return r1.y < r2.y;
	}
}

static double coordDistance(const coord &c1, const coord &c2)
{
	return sqrt((c2.x - c1.x)*(c2.x - c1.x) + (c2.y - c1.y)*(c2.y - c1.y));
}

static double nodeDistance(int n1, int n2)
{
	return coordDistance(GetNodeCoord(n1), GetNodeCoord(n2));
}

static bool areCoordsEqual(const coord &c1, const coord &c2)
{
	return c1.x == c2.x && c1.y == c2.y;
}

static bool areNodesEqual(int n1, int n2)
{
	coord c1, c2; double r1, r2;
	GetNodeData(n1, &c1, &r1);
	GetNodeData(n2, &c2, &r2);

	return areCoordsEqual(c1, c2);
}

static bool areNodesInSingleBin(int n1, int n2)
{
	// are rounded coords equal?
	coord r1 = roundCoord(GetNodeCoord(n1));
	coord r2 = roundCoord(GetNodeCoord(n2));

	return areCoordsEqual(r1, r2);
}

static bool areNodesNear(int n1, int n2)
{
	double l = nodeDistance(n1, n2);

	return l < 0.1;		// FIXME use constant
}

/* Performs "contrast comparison" of non-negative arguments, it treats "close
 * enough" numbers as being equal.  Checks for significant ratio between both
 * arguments.  The COMPARISON_RATIO constant is used as a "threshold".
 * COMPARISON_RATIO is lower than 1 and positive. We define x1 being
 * "contrasty lower" than x2 (i.e. x1 << x2) if and only if it is
 * x1 < COMPARISON_RATIO * x2.
 *
 * Returns:
 * 			-1		iff		x1  <<  x2
 * 			 0		iff		x1 "==" x2
 * 			 1		iff		x1  >>  x2
 *
 * Suggested usage: contrastCompare(x1, x2) OP 0, where OP is one of usual
 * comparison operators ('<', '==', '!=', '>').
 */
static char contrastCompare(double x1, double x2)
{
	assert(x1 >= 0 && x2 >= 0); // implemented for non-negative inputs only

	if (x1 < x2) {
		if (x1 < COMPARISON_RATIO * x2) {
			return -1;
		} else {
			return 0;
		}
	} else {
		if (x2 < COMPARISON_RATIO * x1) {
			return 1;
		} else {
			return 0;
		}
	}
}

static coord determineNodeDisplacement(int n)
{
	// Compute centre of gravity of neighbours of n
	int e1   = GetIncidentEdge(n);
	int n1   = GetOtherNode(e1, n);
	coord c1 = GetNodeCoord(n1);

	int e2   = GetCCWEdge(e1, n);
	int e3   = GetCWEdge(e1, n);

	int n2   = GetOtherNode(e2, n);
	coord c2 = GetNodeCoord(n2);

	coord centre;					// neighbours' centre of gravity
	if (e2 == e3) {
		centre.x = (c1.x + c2.x)/2;
		centre.y = (c1.y + c2.y)/2;
	} else {
		int n3    = GetOtherNode(e3, n);
		coord c3  = GetNodeCoord(n3);
		centre.x = (c1.x + c2.x + c3.x)/3;
		centre.y = (c1.y + c2.y + c3.y)/3;
	}

	coord c = GetNodeCoord(n);		// original node's coords

	coord d;						// the displacement
	double l = coordDistance(c, centre);
	if (l == 0.0) {
		d.x = 0.0;
		d.y = 0.0;
	} else {
		d.x = centre.x - c.x;
		d.y = centre.y - c.y;
		// FIXME: use reasonable constants
		d.x *= 0.05 / l;				// normalize
		d.y *= 0.05 / l;
	}

	return d;
}

static bool hasIncidentNeighbour(int n)
{
	int e1 = GetIncidentEdge(n);
	int n1 = GetOtherNode(e1, n);

	if (areNodesEqual(n, n1)) {
		return true;
	}

	int e_ccw = GetCCWEdge(e1, n);
	int n_ccw = GetOtherNode(e_ccw, n);
	if (areNodesEqual(n, n_ccw)) {
		return true;
	}

	int e_cw = GetCCWEdge(e1, n);
	int n_cw = GetOtherNode(e_cw, n);
	if (areNodesEqual(n, n_cw)) {
		return true;
	}

	return false;
}

static bool hasCloseNeighbour(int n)
{
	int e1 = GetIncidentEdge(n);
	int n1 = GetOtherNode(e1, n);

	bool result = false;
//	if (areNodesInSingleBin(n, n1)) {
	if (areNodesNear(n, n1)) {
		result = true;
	}

	int e_ccw = GetCCWEdge(e1, n);
	int n_ccw = GetOtherNode(e_ccw, n);
//	if (areNodesInSingleBin(n, n_ccw)) {
	if (areNodesNear(n, n_ccw)) {
		result = true;
	}

	int e_cw = GetCCWEdge(e1, n);
	int n_cw = GetOtherNode(e_cw, n);
//	if (areNodesInSingleBin(n, n_cw)) {
	if (areNodesNear(n, n_cw)) {
		result = true;
	}

	return result;
}

/* Vroni adds four dummy points to defining sites of the input; dummy points are located at the corners
 * of the bounding box and their indices in the pnts vector are 0, 1, num_pnts-2 and num_pnts-1. */
static bool isDummyPoint(const int & site_id, const t_site & site_type)
{
	// num_pts is Vroni's internal variable
	int last_valid_pnt_ix = num_pnts - 3;
	
	return site_type == PNT && (site_id == 0 || site_id == 1 || site_id > last_valid_pnt_ix);
}

/** Returns true iff none of the defining sites of the edge e is dummy point. */
static bool isEdgeDefinedByDummyPoint(int e)
{
	int s1, s2; t_site t1, t2;
	GetLftSiteData(e, &s1, &t1);
	GetRgtSiteData(e, &s2, &t2);

	return isDummyPoint(s1, t1) || isDummyPoint(s2, t2);
}

static bool isDeg2WmatNode(int n)
{
	int e = GetIncidentEdge(n);
	if (!IsWmatEdge(e)) {
		e = GetCWEdge(e, n);
		if (!IsWmatEdge(e)) {
			e = GetCWEdge(e, n);
			if (!IsWmatEdge(e)) {
				std::cerr << "ERROR in isDeg2WmatNode, the given node (" << n << ") is not WMAT node at all." << std::endl;
				exit(1);
			}
		}
	}

	assert(IsWmatEdge(e));

	int e_ccw = GetCCWEdge(e, n);
	int e_cw  = GetCWEdge(e, n);

	if (e_ccw == e_cw) {
		return IsWmatEdge(e_ccw);
	} else {
		return ( IsWmatEdge(e_ccw) && !IsWmatEdge(e_cw) )
		    || (!IsWmatEdge(e_ccw) &&  IsWmatEdge(e_cw) );
	}
}

// FIXME: this does not work as expected, I think (fj)
// Remove
static bool isDeg2Node(int n)
{
	int e = GetIncidentEdge(n);
	int e_ccw = GetCCWEdge(e, n);
	int e_cw  = GetCWEdge(e, n);
		
	if (e_ccw == e_cw) {
		return IsWmatEdge(e_ccw);
	} else {
		return ( IsWmatEdge(e_ccw) && !IsWmatEdge(e_cw) )
		    || (!IsWmatEdge(e_ccw) &&  IsWmatEdge(e_cw) );
	}
}

// FIXME: rename (WMAT)
static bool isDeg3Node(int n)
{
	int e = GetIncidentEdge(n);
	int e_ccw = GetCCWEdge(e, n);
	int e_cw  = GetCWEdge(e, n);
		
	if (e_ccw != e_cw) {
		return IsWmatEdge(e_ccw) && IsWmatEdge(e_cw) && IsWmatEdge(e);
	} else {
		return false;
	}
}

// FIXME: This does not work in my opinion (fj)
// Remove?
static bool hasDeg3Neighbour(int n)
{
	int e = GetIncidentEdge(n);
	int m = GetOtherNode(e, n);

	return isDeg3Node(m);
}

static bool isFrontierBasedEdge(int e)
{
	int s1, s2; t_site t1, t2;
	GetLftSiteData(e, &s1, &t1);
	GetRgtSiteData(e, &s2, &t2);

	return (t1 == SEG && segs[s1].ext_appl.isFrontier) || (t2 == SEG && segs[s2].ext_appl.isFrontier);
}

static bool isSegmentBasedEdge(int e)
{
	int s1, s2; t_site t1, t2;
	GetLftSiteData(e, &s1, &t1);
	GetRgtSiteData(e, &s2, &t2);

	return (t1 == SEG) || (t2 == SEG);
}

static bool isOuterBasedEdge(int e)
{
	int s1, s2; t_site t1, t2;
	GetLftSiteData(e, &s1, &t1);
	GetRgtSiteData(e, &s2, &t2);

	return (t1 == SEG && segs[s1].ext_appl.isOuter) || (t2 == SEG && segs[s2].ext_appl.isOuter);
}

/* ********************** toString() functions *********************** */

std::string coordToString(const coord & coord, bool addParentheses)
{
	using namespace std;

	string lp, rp;
	if (addParentheses) {
		lp = "(";
		rp = ")";
	} else {
		lp = rp = "";
	}

	stringstream ss;
	ss << fixed << setprecision(2) << right;
	ss << lp << setw(6) << right << UnscaleX(coord.x) << ", " // FIXME
			 << setw(6) << right << UnscaleY(coord.y) << rp;
	return ss.str();
}

std::string coordToString(const coord & coord)
{
	return coordToString(coord, true);
}

std::string nodeToString(int n)
{
	coord c; double r;
	GetNodeData(n, &c, &r);

	using namespace std;
	stringstream ss;
	ss << fixed << setprecision(2) << right;
	ss << "[n=" << n << ", r =" << setw(6) << UnscaleV(r) << ", coord = " << coordToString(c) << "]";

	return ss.str();
}

std::string edgeToString(int e)
{
	int n1 = GetStartNode(e);
	int n2 = GetEndNode(e);

	using namespace std;
	stringstream ss;
	ss << fixed << setprecision(2) << right;
	ss << "{e = " << e << ", " << (IsWmatEdge(e)?"WMAT":"nowm") << ": "
		<< nodeToString(n1) << ", " << nodeToString(n2) << "}";

	return ss.str();
}

static std::string edgeDefiningSitesToString(int e)
{
	int s1, s2; t_site t1, t2;
	GetLftSiteData(e, &s1, &t1);
	GetRgtSiteData(e, &s2, &t2);

	using namespace std;

	stringstream ss;

	ss << "s1 = " << setw(2) << right << s1
		<< " [" << site_type_names[t1] << "],  s2 = "
		<< setw(2) << right << s2
		<< " [" << site_type_names[t2] << "]";
	return ss.str();
}


#endif   /* ----- #ifndef VRONI_UTILS_H_INC  ----- */

// vi:ai:sw=4 ts=4 sts=0 tw=120
