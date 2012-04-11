/*
 * ===========================================================================
 *
 *       Filename:  Color.hpp
 *
 *    Description:  Color definitions
 *
 *        Version:  1.0
 *        Created:  02/26/2012 09:28:08 PM
 *       Compiler:  gcc
 *
 *         Author:  Filip Jares (fj), filipjares@post.cz
 *
 * ===========================================================================
 */

#ifndef  POLY2VD_COLOR_H_INC
#define  POLY2VD_COLOR_H_INC

namespace poly2vd {

struct Color {
	float r, g, b, a;
	Color();
	Color(float _r, float _g, float _b): r(_r), g(_g), b(_b), a(1.0f) {};
	Color(float _r, float _g, float _b, float _a): r(_r), g(_g), b(_b), a(_a) {};

	static const Color BLUE;
	static const Color RED;
	static const Color YELLOW;

	static const Color GREEN;
	static const Color PINK;
	static const Color VIOLET;
	static const Color ORANGE;
};

}        /* ----- namespace poly2vd ----- */

#endif   /* ----- #ifndef POLY2VD_COLOR_H_INC  ----- */

// vi:ai:sw=4 ts=4 sts=0 tw=120
