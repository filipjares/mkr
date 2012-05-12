/*
 * ===========================================================================
 *
 *       Filename:  Color.cpp
 *
 *    Description:  Color definitions
 *
 *        Version:  1.0
 *        Created:  04/11/2012 09:53:10 PM
 *       Compiler:  gcc
 *
 *         Author:  Filip Jares (fj), filipjares@post.cz
 *
 * ===========================================================================
 */
#include "Color.hpp"

namespace poly2vd {

const Color Color::BLUE(0.0f, 0.0f, 1.0f, 0.95f);
const Color Color::RED (1.0f, 0.0f, 0.0f, 0.95f);
const Color Color::YELLOW (1.0f, 1.0f, 0.0f, 0.95f);

const Color Color::GREEN(0.0f, 1.0f, 0.0f, 0.90f);
const Color Color::PINK(1.0f, 192.0/255.0, 203.0/255.0, 0.90f);
const Color Color::VIOLET (0.5f, 0.0f, 1.0f, 0.90f);
const Color Color::ORANGE (1.0f, 0.5f, 0.0f, 0.90f);

}		/* ----- namespace poly2vd ----- */


