
ABOUT
=====

This repository contains our attempt to implement and use some of the algorithms described in
article "Coordinated multi-robot exploration using a segmentation of the environment" by Wurm,
Stachniss and Burgard. See the "Wurm2008Coordinated" record in presentation/bibliography.bib
for reference.

This repository represents our semestral work for the MKR (Mobile and Collective Robotics
course) at CTU / FEE (Czech Technical University, Faculty of Electrical Engineering) from
fall/winter 2011/2012.

Read src/README.txt for information about how to build the source.

Authors:
	Filip Jares	jaresfil@fel.cvut.cz
	Tomas Juchelka	juchetom@fel.cvut.cz

LIBRARIES, CODE AND FRAMEWORKS USED
===================================

Clipper	4.6.3				http://angusj.com/delphi/clipper.php

	Clipper is open source library for polygon clipping. Used C++ sources are located in
	src/clipper.{cpp,hpp}.

Vroni 6.0				http://www.cosy.sbg.ac.at/~held/projects/vroni/vroni.htm

	Application/library that computes the Voronoi diagram of set of points and line segments and
	circular arcs in the plane. Written by Martin Held. We used Vroni to generate Voronoi
	diagrams out of polygons created with help of (modified) Clipper library.

ROS					http://www.ros.org/

	Robot Operating System -- software framework for robot software development.

Boost Graph Library (BGL)		http://www.boost.org/libs/graph

Dijkstra implementation by Jan Faigl


DIRECTORY STRUCTURE:
====================

Source files are located in src/ directory. Important (git) submodule is Vroni
which resides in its VRONI_6.0 directory.

	build/				# In future everything should be built
					# here
	build/vroni			# this is where Vroni is built
	VRONI_6.0/			# Vroni submodule tree
	presentation/			# Slides for end-of-the-class presentation
	src/				# source files (see src/README.txt for details)
	util/				# utilities, scripts
	CMakeLists.txt
	Makefile
	README.txt			# this readme file
	mainpage.dox
	manifest.xml

ROS HOWTO:
==========

% make and run stage:
% must be in stage directory (roscd stage)
------------------------------------------------
rosmake stage
rosrun stage stageros world/willow-erratic.world

% in project directory run:
rosmake
rosrun project listener

running vroni
./bin/vroni --file util/fiddling_with_vroni/data/map_from_the_paper.dxf --wmat --OGL --full

