
DIRECTORY STRUCTURE:
====================

	src/						# source files
	util/						# utilities, scripts
	CMakeLists.txt
	Makefile
	README.txt					# this readme file
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

