#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <libxml/xmlmemory.h>
#include <libxml/parser.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>


using namespace std;

class Point {
    public:
        Point(const double _x=0, const double _y=0):x(_x),y(_y){}
        Point(const Point &rhs):x(rhs.x),y(rhs.y) {}
        ~Point(){}
        double x,y;
};

vector< vector<Point> > boundarySegments;
vector< vector<Point> > voronoiSegments;



/** loads points from <path> ... </path> tag */
static void loadPath(xmlNodePtr node, xmlDocPtr doc, vector<Point> &res) {
            
    if (xmlStrcmp(node->name, (const xmlChar *)"path")) {
        cerr << "This node is supposed to be 'group', not " << node->name << "\n";
        return;
    }
    xmlChar * key = xmlNodeListGetString(doc, node->xmlChildrenNode, 1);

    if (key) {
        stringstream ss;
        ss.str(string((const char *)key));
        string flag;
        double x,y;
        while(ss) {
            if (ss >> x >> y >> flag) {
                res.push_back(Point(x,y));
            }
        }
    }

}


/** loads paths from <group>   </group> */
static void loadPaths(xmlNodePtr node, xmlDocPtr doc, vector< vector<Point> > &res) {

    if (xmlStrcmp(node->name, (const xmlChar *)"group")) {
        cerr << "This node is supposed to be 'group', not " << node->name << "\n";
        return;
    }

    node = node->children;
    vector<Point> tmp;
    while(node) {

        if (!xmlStrcmp(node->name, (const xmlChar *)"path")) {
            loadPath(node, doc,tmp);
            if (tmp.size() > 0) {
                res.push_back(tmp);
            }
            tmp.clear();
        }

        node=node->next;
    }
    
    cerr << "Loaded " << res.size() << " paths !\n";
}


static void parsePage(xmlNodePtr node, xmlDocPtr doc) {

    node = node->children;

    while(node) {
        if (node->type == XML_ELEMENT_NODE) {
            if (!xmlStrcmp(node->name, (const xmlChar *)"group")) {
                xmlChar *propval = xmlGetProp(node, (const xmlChar *)"layer");
                if (!xmlStrcmp(propval, (const xmlChar *)"boundary")) {
                    loadPaths(node,doc,boundarySegments);
                }

                if (!xmlStrcmp(propval, (const xmlChar *)"voronoi")) {
                    loadPaths(node,doc,voronoiSegments);
                }
            }
        }

        node = node->next;
    }
}

static void parseDoc(const char *docname)
{

	xmlDocPtr	doc;
	xmlNodePtr	cur;
	xmlNodePtr	newnode;
	xmlAttrPtr	newattr;

	doc = xmlParseFile(docname);

	if (doc == NULL) {
		fprintf(stderr, "Document not parsed successfully. \n");
		return;
	}
	cur = xmlDocGetRootElement(doc);

	if (cur == NULL) {
		fprintf(stderr, "empty document\n");
		xmlFreeDoc(doc);
		return;
	}
	if (xmlStrcmp(cur->name, (const xmlChar *)"ipe")) {
		fprintf(stderr, "document of the wrong type, root node != ipe !!");
		xmlFreeDoc(doc);
		return;
	}

    cur = cur->children;

    while(cur) {
        if (!xmlStrcmp(cur->name, (const xmlChar *)"page")) {
            parsePage(cur, doc);
        }
        cur = cur->next;
    }
    xmlFreeDoc(doc);

}


/** save to txt file, sequences of points are separated by blank lines.
  * the file can be plotted using gnuplot:

  plot "file" i 0 w l, "" i 1 w l

  */
static void saveToTxt(const char *filename) {
    ofstream ofs(filename);

    ofs <<  "#boundary segments\n";
    for(int i=0;i<(int)boundarySegments.size();i++) {
        for(int j=0;j<(int)boundarySegments[i].size();j++) {
            ofs << boundarySegments[i][j].x << " " << boundarySegments[i][j].y << "\n";
        }
        ofs << "\n";
    }
    ofs << "\n";


    ofs <<  "#voronoi segments\n";
    for(int i=0;i<(int)voronoiSegments.size();i++) {
        for(int j=0;j<(int)voronoiSegments[i].size();j++) {
            ofs << voronoiSegments[i][j].x << " " << voronoiSegments[i][j].y << "\n";
        }
        ofs << "\n";
    }
    ofs << "\n";

    ofs.close();

}



int main(int argc, char **argv)
{

	if (argc < 3) {
        cerr << "usage: " << argv[0] << " <infile.xml> <outfile.txt>\n";
        cerr << "XML: file in Ipe xml format, outtained from vroni code (option --Ipe)\n";
        cerr << "output is txt file with coordinates of each segment\n";
        exit(0);
	}
    const char *infile = argv[1];
    const char *outfile = argv[2];

	parseDoc(infile);

    cerr << "Voronoi segment.size = " << voronoiSegments.size() << "\n";
    cerr << "Boundary segment.size = " << boundarySegments.size() << "\n";
    saveToTxt(outfile);

	return 0;
}


