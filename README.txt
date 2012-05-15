
ABOUT
=====

This repository contains our attempt to implement and use an algorithm that
performs a search for "critical nodes" in a geometrical map. Criteria for a
critical node were described in the article "Coordinated multi-robot
exploration using a segmentation of the environment" by Wurm, Stachniss and
Burgard. See the "Wurm2008Coordinated" record in
presentation/bibliography.bib for reference.

This repository represents our semestral work for the MKR (Mobile and
Collective Robotics course) at CTU / FEE (Czech Technical University,
Faculty of Electrical Engineering) from fall/winter 2011/2012.

For information on about how to build and run the project, please see the
HOWTO.txt file.

The rest of the file contains list of used libraries and frameworks and
directory tree description.

Authors:
    Filip Jares jaresfil@fel.cvut.cz
    Tomas Juchelka  juchetom@fel.cvut.cz


LIBRARIES, CODE AND FRAMEWORKS USED
===================================

Clipper 4.6.3       http://angusj.com/delphi/clipper.php

    Clipper is open source library for polygon clipping. Used C++ sources
    are located in src/clipper.{cpp,hpp}.

Vroni 6.0           http://www.cosy.sbg.ac.at/~held/projects/vroni/vroni.html

    Application/library that computes the Voronoi diagram of set of points
    and line segments and circular arcs in the plane. Written by Martin
    Held. We used Vroni to generate Voronoi diagrams out of polygons created
    with help of (modified) Clipper library.

ROS                 http://www.ros.org/

    Robot Operating System -- software framework for robot software
    development. It was used for demonstration and visualization.


DIRECTORY STRUCTURE:
====================

Source files are located in src/ directory. Important (git) submodule is
Vroni which resides in its VRONI_6.0 directory.

    bin/            # directory containing build output (binaries)
    build/          # build directory of this project
                    # ideally everything (including Vroni) should be built
                    # here; however currently Vroni is built separately in 
                    # the VRONI_6.0/build/ directory
    VRONI_6.0/      # Vroni submodule tree
    presentation/   # Slides for end-of-the-class presentation
    src/            # source files (see src/README.txt for details)
    srv/            # own ROS services definition
    srv_gen/        # generated files for our services
    util/           # utilities, scripts
    CMakeLists.txt  # CMake build configuration file
    Makefile        # simple makefile referencing ROS's main makefile
    README.txt      # this readme file
    mainpage.dox    # stub of project's doxygen documentation
    manifest.xml    # project's manifest

