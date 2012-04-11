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

namespace poly2vd {

/// Status of WMAT Voronoi edges
typedef enum {
	/// Unprocessed edge
	UNDEF,
	/// Edge in already explored area
	EXPLORED,
	/// Edge in neighbourhood of frontiers, located near the unexplored area
	FRONTIER 
} EdgeStatus;

const std::string EdgeStatusNames[] = {"UNDEF", "EXPLORED", "FRONTIER_BASED"};

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
	/**
	 * Id of Node that edge with EXPLORED status leads to.
	 * For nodes in other states it is set to -1.
	 */
	int frontierGoalNode;
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
			edges[i].frontierGoalNode = -1;
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

	int getPreviousEdge(int n)
	{
		assert(inNodesList(n));
		// assert(nodes[n].previousEdge != -1);
		return nodes[n].previousEdge;
	}

	int getPreviousNode(int n)
	{
		assert(inNodesList(n));
		// assert(nodes[n].previousNode != -1);
		return nodes[n].previousNode;
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

	/* Edge status */

	EdgeStatus getEdgeStatus(int e)
	{
		assert(inEdgesList(e));
		return edges[e].status;
	}

	void setEdgeStatus(int e, EdgeStatus status)
	{
		assert(inEdgesList(e));
		edges[e].status = status;
	}

	/* Frontier boundary node info */

	int getFrontierBoundaryNode(int e)
	{
		assert(inEdgesList(e));
		assert(getEdgeStatus(e) == EXPLORED);
		return edges[e].frontierGoalNode;
	}

	bool isFrontierBoundaryNodeSet(int e)
	{
		return getFrontierBoundaryNode(e) != -1;
	}

	void setFrontierBoundaryNode(int e, int frontierGoalNode)
	{
		assert(inEdgesList(e));
		assert(getEdgeStatus(e) == EXPLORED);
		edges[e].frontierGoalNode = frontierGoalNode;
	}

};

}        /* ----- namespace poly2vd ----- */

#endif   /* ----- #ifndef GRAPH_META_H_INC  ----- */

// vi:ai:sw=4 ts=4 sts=0 tw=120
