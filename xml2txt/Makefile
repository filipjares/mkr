
CXX=ccache g++
TARGET=xml2txt

LDFLAGS+= 
CXXINCLUDE+=

CXXFLAGS+=$(CXXINCLUDE)
CXXFLAGS+=$(CXXDEFINE)

LDFLAGS+=`pkg-config --libs cairo`
CXXFLAGS+=`pkg-config --cflags cairo`

LDFLAGS+=`xml2-config --libs`
CXXFLAGS+=`xml2-config --cflags`

SRC=$(wildcard *.cc)
OBJS=$(patsubst %.cc,%.o,$(SRC))

all: $(OBJS)
	$(CXX) -o $(TARGET) $(OBJS) $(LIBPATH) $(LDFLAGS)

.cc.o: %.cc
	$(CXX) $(CXXDEFINE) $(CXXFLAGS) -c $<

clean:
	rm -f $(OBJS) $(TARGET)

