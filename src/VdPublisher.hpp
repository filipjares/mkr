/*
 * ===========================================================================
 *
 *       Filename:  VdPublisher.hpp
 *
 *    Description:  Helper for publication of graphical representation of
 *                  Voronoi diagrams and other objects through ROS (for RViz)
 *
 *        Version:  1.0
 *        Created:  02/26/2012 09:21:10 PM
 *       Compiler:  gcc
 *
 *         Author:  Filip Jares (fj), filipjares@post.cz
 *
 * ===========================================================================
 */

#ifndef  VD_PUBLISHER_H_INC
#define  VD_PUBLISHER_H_INC

#include "poly2vd.hpp"
#include "Color.hpp"

class VdPublisher
{
private:
	const ros::Publisher & marker_pub;
	std::string frame_id;
	double duration;

	visualization_msgs::Marker wmat_marker;
	visualization_msgs::Marker wmat_f_marker;
	visualization_msgs::Marker path_marker;
public:
	VdPublisher(const ros::Publisher & _marker_pub, const std::string & _frame_id, double _duration): marker_pub(_marker_pub), frame_id(_frame_id), duration(_duration)
	{
		// prepare Markers for both "ordinary" (non-frontier based)...
		wmat_marker.header.frame_id = frame_id;
		wmat_marker.header.stamp = ros::Time::now();
		wmat_marker.ns = "wmat";
		wmat_marker.action = visualization_msgs::Marker::ADD;
		wmat_marker.pose.orientation.w = 1.0;
		wmat_marker.id = 0;
		wmat_marker.lifetime = ros::Duration(duration);
		wmat_marker.type = visualization_msgs::Marker::LINE_LIST;
		wmat_marker.scale.x = 0.25;
		wmat_marker.color.g = 1.0f;
		wmat_marker.color.a = 1.0;

		// ... and frontier-based edges
		wmat_f_marker.header.frame_id = frame_id;
		wmat_f_marker.header.stamp = ros::Time::now();
		wmat_f_marker.ns = "wmatF";
		wmat_f_marker.action = visualization_msgs::Marker::ADD;
		wmat_f_marker.pose.orientation.w = 1.0;
		wmat_f_marker.id = 0;
		wmat_f_marker.lifetime = ros::Duration(duration);
		wmat_f_marker.type = visualization_msgs::Marker::LINE_LIST;
		wmat_f_marker.scale.x = 0.25;
		wmat_f_marker.color.g = 0.5f;
		wmat_f_marker.color.b = 1.0f;
		wmat_f_marker.color.a = 1.0;

		// prepare path marker
		path_marker.header.frame_id = frame_id;
		path_marker.header.stamp = ros::Time::now();
		path_marker.ns = "wmat_paths";
		path_marker.action = visualization_msgs::Marker::ADD;
		path_marker.pose.orientation.w = 1.0;
		path_marker.id = 0;
		path_marker.lifetime = ros::Duration(duration);
		path_marker.type = visualization_msgs::Marker::LINE_LIST;
		path_marker.scale.x = 0.25;
		path_marker.color.r = 1.0f;
		path_marker.color.g = 0.0f;
		path_marker.color.b = 0.0f;
		path_marker.color.a = 1.0;
	}

	/** Publishes the specified sphere immediately */
	void publishSphere(int id, coord location, double diameter, Color color)
	{
		// prepare the Marker
		visualization_msgs::Marker marker;
		marker.header.frame_id = frame_id;
		marker.header.stamp = ros::Time::now();
		marker.ns = "deg2Nodes";
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.orientation.w = 1.0;
		marker.id = id;
		marker.lifetime = ros::Duration(duration);
		marker.type = visualization_msgs::Marker::SPHERE;

		// set its position, diameter and color
		marker.pose.position.x = UnscaleX(location.x);
		marker.pose.position.y = UnscaleY(location.y);
		marker.scale.x = marker.scale.y = marker.scale.z = UnscaleV(diameter);
		marker.color.r = color.r; marker.color.g = color.g; marker.color.b = color.b;
		marker.color.a = color.a;

		// publish it using given publisher
		marker_pub.publish(marker);
	}

	/**
	 * Appends the specified Vroni's resulting edge to list of edges that will
	 * be published with publishEdges().
	 */
	void appendEdge(int edge)
	{
		// get endnodes coordinates
		coord c1 = GetNodeCoord(GetStartNode(edge));
		coord c2 = GetNodeCoord(GetEndNode(edge));
		geometry_msgs::Point p1;
		p1.x = UnscaleX(c1.x);
		p1.y = UnscaleY(c1.y);
		geometry_msgs::Point p2;
		p2.x = UnscaleX(c2.x);
		p2.y = UnscaleY(c2.y);

		// append them
		if (isFrontierBasedEdge(edge)) {
			wmat_f_marker.points.push_back(p1);
			wmat_f_marker.points.push_back(p2);
		} else {
			wmat_marker.points.push_back(p1);
			wmat_marker.points.push_back(p2);
		}
	}

	/**
	 * Appends the whole path leading from the rootNode to goalNode to the list
	 * of edges that will be published with publishEdges().
	 *
	 * \param <goalNode> Node that will be searched 
	 * \param <previous> Pointer to array of indices of previous nodes
	 */
	void appendPath(int goalNode, int * previous)
	{
		
	}

	/** Publishes Vroni's edges */
	void publishEdges()
	{
		marker_pub.publish(wmat_marker);
		marker_pub.publish(wmat_f_marker);
		marker_pub.publish(path_marker);
	}
};

#endif   /* ----- #ifndef VD_PUBLISHER_H_INC  ----- */

// vi:ai:sw=4 ts=4 sts=0 tw=120
