

VRONI_SRC_DIR = /home/tomas/Skola/MKR/project/VRONI_6.0/src/
VRONI_LIB_DIR = /home/tomas/Skola/MKR/project/VRONI_6.0/build/src/

TLIB = $(VRONI_LIB_DIR)
INCLUDES = -I. -I$(VRONI_SRC_DIR)

# set up compiler and options
CXX = g++
CXXFLAGS = -Wall -g $(INCLUDES)

# use clipper.o ass well
poly2vd: poly2vd.cpp
	$(CXX) $(CXXFLAGS) -L. -L$(VRONI_LIB_DIR) -o $@ $< -lvroni 

%.o: %.cpp
	$(CXX) -c $(CXXFLAGS) $<

all: poly2vd
.PHONY: all clean distclean

clean:
	$(RM) *.o

distclean:
	$(RM) poly2vd

