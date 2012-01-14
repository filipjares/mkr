
WARNING: this file is not up to date; build directory of Vroni was
moved from build/vroni to VRONI_6.0/build/ to build the project,
run rosmake in the toplevel directory...


INTRODUCTION
============

	Note: All paths are specified with respect to the toplevel
	      directory of this "MKR" repository unless stated
	      otherwise.

This text describes how to build our source and run it successfully
in following two sections. At the end, it contains short description
of the contents of this src/ directory.

Unfortunately there are two inconviences that complicate building our
code.  (1) We are not allowed to publish Vroni and therefore we
mantain its code in separate repository. The VRONI_6.0/ directory forms
a git submodule of this "superproject" MKR repository. (2) We have to
learn how to write proper Makefiles yet and therefore for now, it is
neccessary to build Vroni and the rest of the code separately and "by
hand".


VRONI SUBMODULE CONFIGURATION
=============================

After you have cloned this MKR repository, you have to configure git
and tell it where to find Vroni.  From toplevel directory, run:

	$ git submodule init

This creates [submodule "VRONI_6.0"] record in the .git/config file.
However, it will point to wrong location.  In order to correct it, you
have to edit the `url' property in the .git/config file and set it to
location of the reference Vroni repository. However this reference
Vroni repository (or its url) has to be obtained first.

After the successfull setup the `git submodule update' command should
succeed:

	$ git submodule update


HOW TO BUILD OUR SOURCE
=======================

First it is neccessary to build Vroni.  Then to build our source, run
the make utility from this src directory.

	$ cd src/
	$ make

However, for this to be successfull, Vroni header files and its library
have to be available. Following paragraphs describe how to assure this.

Our Makefile in src/Makefile expects that these parts of Vroni are
available and "up to date" (that is their state corresponds to the
commit that MKR repository currently points to):

  (1) header files at
      	VRONI_6.0/src/
  (2) .so library at
      	build/vroni/src/libvroni.so

In order To assure (1), run the `git submodule update' command in the
toplevel directory. (VRONI_6.0 submodule has to be configured, see the
"VRONI SUBMODULE CONFIGURATION" section for instructions how to do
that.)

	$ git submodule update

Build (source) tree of our code and build tree of Vroni are separated.
(We have to learn how to write Makefiles yet).  In order order to
build Vroni and assure (2), you have to create build/vroni directory
first

	$ mkdir build
	$ mkdir build/vroni

and then configure Vroni:

	$ cd build/vroni
	$ cmake -D WITH_LIB=ON ../../VRONI_6.0/

(you can consider using these options along with `WITH_LIB=ON'):

	-D WITH_VERBOSE=1
	-D WITH_LIB=ON		# build libvroni.so
	-D WITH_EXE=ON		# build Vroni as standalone app
	-D WITH_MIC=ON
	-D CMAKE_BUILD_TYPE=Debug ..

and finally, you build libvroni.so with make:

	$ make
  
In order to RUN our code (currently the poly2vd for example),
libvroni.so has to be present in path of your operating systems'
dynamic loader. (See `man ld.so'. You can also add the directory
containing libvroni.so to the contents of the LD_LIBRARY_PATH
environment variable.)

SOURCE FILES DESCRIPTION
========================

src/README.txt		# This readme file

src/Makefile		# Makefile for our source, in order to make it
			# work, please read the HOW TO BUILD OUR SOURCE
			# section

src/poly2vd.cpp		# This file is intended to contain code, that will
			# take polygonal input and transform it into
			# Voronoi diagram using the Vroni library.
			# Now it contains simple main() function that
			# attempts to utilize Vroni.

src/clipper.hpp		# Clipper library used for manipulation with
src/clipper.cpp		# polygons.

src/dijkstra_heap.h	# Dijkstra algorithm implementation
src/dijkstra_lite.h     # by Jan Faigl

src/listener.h		#
src/listener.cpp	#


