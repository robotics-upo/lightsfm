/***********************************************************************/
/**                                                                    */
/** map.hpp                                                            */
/**                                                                    */
/** Copyright (c) 2016, Service Robotics Lab.                          */ 
/**                     http://robotics.upo.es                         */
/**                                                                    */
/** All rights reserved.                                               */
/**                                                                    */
/** Authors:                                                           */
/** Ignacio Perez-Hurtado (maintainer)                                 */
/** Jesus Capitan                                                      */
/** Fernando Caballero                                                 */
/** Luis Merino                                                        */
/**                                                                    */   
/** This software may be modified and distributed under the terms      */
/** of the BSD license. See the LICENSE file for details.              */
/**                                                                    */
/** http://www.opensource.org/licenses/BSD-3-Clause                    */
/**                                                                    */
/***********************************************************************/


#ifndef _ROSMAP_HPP_
#define _ROSMAP_HPP_


#include <algorithm>
#include <cmath>
#include <vector>
#include <list>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/MapMetaData.h>
#include "vector2d.hpp"
#include "map.hpp"


namespace sfm
{


class RosMap : public Map
{

public:
	RosMap(RosMap const&) = delete;
        void operator=(RosMap const&)  = delete;
	virtual ~RosMap() {delete kdtree;}
	static RosMap& getInstance()
   	{
      		static RosMap singleton;
      		return singleton;
	}
	#define MAP RosMap::getInstance()
	const nav_msgs::MapMetaData& getInfo() const {return srv.response.map.info;}
	virtual const Obstacle& getNearestObstacle(const utils::Vector2d& x);
	virtual bool isObstacle(const utils::Vector2d& x) const;
	void computeDistances();
	void mapToPixel(const utils::Vector2d& x, Pixel& pixel) const;
        void pixelToMap(const Pixel& pixel, utils::Vector2d& x) const;

private:
	struct Pixel
	{
		Pixel() : x(0), y(0) {}
		Pixel(unsigned index, unsigned width)
		: x(index % width), y(index / width) {}
		int x;
		int y;
		int distance(const Pixel& other) const
		{
			return (x-other.x)*(x-other.x) + (y-other.y)*(y-other.y);	
		}
	};

	struct KdTreeNode
	{
		KdTreeNode(const Pixel& pixel) : pixel(pixel), left(NULL), right(NULL) {}
		~KdTreeNode() {delete left; delete right;}
		Pixel pixel;
		KdTreeNode *left;
		KdTreeNode *right;
	};	

	RosMap();
	static RosMap::KdTreeNode* generateKdTree(std::vector<Pixel>& pixels, int begin, int end, bool split_x=true);
	static bool comparePixelsByX(const Pixel& pixel0, const Pixel& pixel1) {return pixel0.x < pixel1.x;}
	static bool comparePixelsByY(const Pixel& pixel0, const Pixel& pixel1) {return pixel0.y < pixel1.y;}
	static void searchNearestNeighbour(const KdTreeNode* node,const Pixel& pixel, Pixel& best, double& distance, bool split_x=true);
 
	nav_msgs::GetMap srv;
	KdTreeNode *kdtree;
	std::vector<Obstacle> obstacles;
	Pixel current;

	
	
};

inline
RosMap::RosMap()
{
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<nav_msgs::GetMap>("static_map");
	if (client.call(srv)) {
		std::vector<Pixel> pixels;
		for (unsigned i=0; i<srv.response.map.data.size(); i++) {
			if (srv.response.map.data[i]==100 || srv.response.map.data[i]==-1) {
				pixels.emplace_back(i,getInfo().width);
			}
		}
		kdtree = generateKdTree(pixels,0,pixels.size());
		ROS_ASSERT(kdtree!=NULL);
		obstacles.resize(srv.response.map.data.size());
		
	} else {
		ROS_FATAL("UNABLE TO GET MAP");
		ROS_BREAK();
	}
}



inline
void RosMap::computeDistances()
{
	if ((unsigned)current.y == getInfo().height) {
		return;
	}

	unsigned index = current.y * getInfo().width + current.x;
	
	Obstacle& obs = obstacles[index];	
	if (obs.distance<0) {
		Pixel best;
		if (srv.response.map.data[index]==100 || srv.response.map.data[index]==-1) {
			obs.distance=0;
			obs.position.setX(((double)(current.x))*getInfo().resolution);
			obs.position.setY(((double)(current.y))*getInfo().resolution);
		} else {
			searchNearestNeighbour(kdtree,current,best,obs.distance);
			obs.distance = std::sqrt(obs.distance) * getInfo().resolution;
			obs.position.setX(((double)(best.x))*getInfo().resolution);
			obs.position.setY(((double)(best.y))*getInfo().resolution);
		}
	}
	current.x ++;
	if ((unsigned)current.x == getInfo().width) {
		current.x = 0;
		current.y ++;
	}
}


inline
void RosMap::pixelToMap(const Pixel& pixel, utils::Vector2d& x) const
{
	double mapX = (double)pixel.x * getInfo().resolution + getInfo().origin.position.x;
	double mapY = (double)pixel.y * getInfo().resolution + getInfo().origin.position.y;
	x.set(mapX,mapY);

}


inline
void RosMap::mapToPixel(const utils::Vector2d& x, Pixel& pixel) const
{
	double mapX = x.getX();
	double mapY = x.getY();
	mapX-=getInfo().origin.position.x;
	mapY-=getInfo().origin.position.y;
	pixel.x = std::round(mapX/getInfo().resolution);
	pixel.y = std::round(mapY/getInfo().resolution);

	if (pixel.x<0) {
		pixel.x=0;
	} else if ((unsigned)pixel.x>=getInfo().width) {
		pixel.x=getInfo().width-1;
	}

	if (pixel.y<0) {
		pixel.y=0;
	} else if ((unsigned)pixel.y>=getInfo().height) {
		pixel.y=getInfo().height-1;
	}
	
}


inline
bool RosMap::isObstacle(const utils::Vector2d& x) const
{
	Pixel pixel;
	mapToPixel(x,pixel);
	unsigned index = pixel.y * getInfo().width + pixel.x;
	return srv.response.map.data[index]==100 || srv.response.map.data[index]==-1;
}



inline
const Map::Obstacle& RosMap::getNearestObstacle(const utils::Vector2d& x)
{
	
	Pixel pixel;
	mapToPixel(x,pixel);
	
	unsigned index = pixel.y * getInfo().width + pixel.x;
	
	Obstacle& obs = obstacles[index];	
	if (obs.distance<0) {
		Pixel best;
		if (srv.response.map.data[index]==100 || srv.response.map.data[index]==-1) {
			obs.distance=0;
			obs.position.setX(((double)(pixel.x))*getInfo().resolution);
			obs.position.setY(((double)(pixel.y))*getInfo().resolution);
		} else {
			searchNearestNeighbour(kdtree,pixel,best,obs.distance);
			obs.distance = std::sqrt(obs.distance) * getInfo().resolution;
			obs.position.setX(((double)(best.x))*getInfo().resolution);
			obs.position.setY(((double)(best.y))*getInfo().resolution);
		}
	}
	
	return obs;
}


inline
RosMap::KdTreeNode *RosMap::generateKdTree(std::vector<Pixel>& pixels, int begin, int end, bool split_x) 
{
	if (begin == end) {
		return NULL;
	}
	if (split_x) {
		std::sort(pixels.begin()+begin,pixels.begin()+end,comparePixelsByX);
	} else {
		std::sort(pixels.begin()+begin,pixels.begin()+end,comparePixelsByY);
	}
	int index = begin + (end - begin) / 2;
	KdTreeNode  *node = new KdTreeNode(pixels[index]);
	node->left  = generateKdTree(pixels,begin,index,!split_x);
	node->right = generateKdTree(pixels,index+1,end,!split_x); 
	return node;
}

inline
void RosMap::searchNearestNeighbour(const KdTreeNode* node, const Pixel& pixel, Pixel& best, double& distance, bool split_x)
{
	
	KdTreeNode *next, *other;
	double current_distance = pixel.distance(node->pixel);
	
	if ((split_x && comparePixelsByX(pixel,node->pixel)) || (!split_x && comparePixelsByY(pixel,node->pixel))) {
		next  = node->left;
		other = node->right; 
	} else {
		next  = node->right;
		other = node->left; 
	}
		
	if (next==NULL) {
		best = node->pixel;
		distance = current_distance;
		
	} else {
		searchNearestNeighbour(next,pixel,best,distance,!split_x);
		if (current_distance<distance) {
			distance = current_distance;
			best = node->pixel;
		}
		int distance_to_plane= split_x ? (pixel.x - node->pixel.x)*(pixel.x - node->pixel.x) : (pixel.y - node->pixel.y) * (pixel.y - node->pixel.y);

		if (other !=NULL && distance_to_plane<distance) {
			Pixel other_best;
			double other_distance=0;
			searchNearestNeighbour(other,pixel,other_best,other_distance,!split_x);
			if (other_distance<distance) {
				distance = other_distance;
				best = other_best;
			}
		}
	}
}

}
#endif
