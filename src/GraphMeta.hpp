/*
 * ===========================================================================
 *
 *       Filename:  GraphMeta.hpp
 *
 *    Description:  Graph meta-information data structures
 *
 *        Version:  1.0
 *        Created:  02/26/2012 09:06:04 PM
 *       Compiler:  gcc
 *
 *         Author:  Filip Jares (fj), filipjares@post.cz
 *
 * ===========================================================================
 */

#ifndef  GRAPH_META_H_INC
#define  GRAPH_META_H_INC

typedef enum {
	UNDEF,
	EXPLORED,
	FRONTIER 
} EdgeStatus;

struct NodeMeta
{
	bool closed;
	int previousNode;
	int previousEdge;
};

struct EdgeMeta
{
	bool used;
	EdgeStatus status;
};

class GraphMeta
{
private:
	int nodeCount;
	int edgeCount;
	NodeMeta * nodes;
	EdgeMeta * edges;

	std::list<int> open;	// open node list for the BFS search

	bool inNodesList(int n)
	{
		return n >= 0 && n < nodeCount;
	}

	bool inEdgesList(int e)
	{
		return e >= 0 && e < edgeCount;
	}
public:

	GraphMeta(int _nodeCount, int _edgeCount):
		nodeCount(_nodeCount),
		edgeCount(_edgeCount),
		nodes(new NodeMeta[nodeCount]),
		edges(new EdgeMeta[edgeCount])
	{
		// node metadata setup
		for (int i = 0; i < nodeCount; i++) {
			nodes[i].closed = false;
			nodes[i].previousNode = -1;
			nodes[i].previousEdge = -1;
		}
		// edge metadata setup
		for (int i = 0; i < edgeCount; i++) {
			edges[i].used = false;
			edges[i].status = UNDEF;
		}
	}

	~GraphMeta()
	{
		delete [] nodes;
		delete [] edges;
	}

	/* Nodes closed status */

	void setNodeClosed(int n)
	{
		assert(inNodesList(n));
		nodes[n].closed = true;
	}

	bool isNodeClosed(int n)
	{
		assert(inNodesList(n));
		return nodes[n].closed;
	}

	/* List of open nodes */

	void addToOpenList(int n)
	{
		assert(inNodesList(n));
		open.push_back(n);
	}

	bool isOpenListEmpty()
	{
		return open.empty();
	}

	int getFirstNodeFromOpenList()
	{
		int n = open.front();
		open.pop_front();
		return n;
	}

	/* Previous Nodes */

	void setPrevious(int n, int nPrev, int edge)
	{
		assert(inNodesList(n));
		assert(inNodesList(nPrev));
		assert(inEdgesList(edge));
		assert(IsEdgeIncident(edge, n));	// IsEdgeIncident() is Vroni's function
		assert(IsEdgeIncident(edge, nPrev));	// IsEdgeIncident() is Vroni's function

		nodes[n].previousNode = nPrev;
		nodes[n].previousEdge = edge;
	}

	/* Edge markers (used/published or not) */

	bool isEdgeUsed(int e)
	{
		assert(inEdgesList(e));
		return edges[e].used;
	}

	void setEdgeUsed(int e)
	{
		assert(inEdgesList(e));
		edges[e].used = true;
	}
};

#endif   /* ----- #ifndef GRAPH_META_H_INC  ----- */

// vi:ai:sw=4 ts=4 sts=0 tw=120
