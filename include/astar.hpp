/***********************************************************************/
/**                                                                    */
/** astar.hpp                                                          */
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


#ifndef _ASTAR_HPP_
#define _ASTAR_HPP_

#include <vector>
#include <list>
#include <unordered_set>
#include <unordered_map>
#include <limits>
#include <tinyxml.h>

namespace utils
{

class AStar
{

public:
	AStar(const std::string& file);
	AStar() {}
	void addNode(const std::string& id, double x, double y, double r, bool goal=false);
	void addEdge(const std::string& id0, const std::string& id1);
	std::string& getClosestNode(double x, double y, std::string& id) const;
	void getPos(const std::string& id, double& x, double& y, double& radius) const
	{
		x = nodes.at(id).x;
		y = nodes.at(id).y;
		radius = nodes.at(id).radius;
	}
	std::list<std::string>& getPath(const std::string& start_id, const std::string& goal_id, std::list<std::string>& path);
	const std::vector<std::string>& getGoals() const {return goals;}

private:
	
	void init(TiXmlNode *pParent);

	struct Node
	{
		std::string id;
		double x;
		double y;
		double radius;
		std::unordered_map<std::string,double> neighbors;
		double gScore;
		double fScore;
		Node *cameFrom;
	};

	void addEdge(const std::string& id0, const std::string& id1, double cost);

	double heuristic_cost_estimate(const Node* n0, const Node* n1) const
	{
		return (n0->x - n1->x) * (n0->x - n1->x) + (n0->y - n1->y)* (n0->y - n1->y);
	}
	
	std::unordered_map<std::string, Node> nodes;
	std::vector<std::string> goals;
	
};


inline
AStar::AStar(const std::string& file)
{
	TiXmlDocument xml(file);
	if (xml.LoadFile()) {
		init(&xml);
	} 
}


inline
void AStar::init(TiXmlNode *pParent)
{
	if ( !pParent ) return;

	if ( pParent->Type() == TiXmlNode::TINYXML_ELEMENT) {
		TiXmlElement* pElement = pParent->ToElement();
		TiXmlAttribute* pAttrib=pElement->FirstAttribute();
		if (strcmp(pParent->Value(),"waypoint")==0 || strcmp(pParent->Value(),"goal")==0) {
			double x,y,r;
			std::string id;
			id.assign(pAttrib->Value());
			pAttrib=pAttrib->Next();
			pAttrib->QueryDoubleValue(&x);
			pAttrib=pAttrib->Next();
			pAttrib->QueryDoubleValue(&y);
			pAttrib=pAttrib->Next();
			pAttrib->QueryDoubleValue(&r);
			addNode(id,x,y,r,strcmp(pParent->Value(),"goal")==0);
		} else if (strcmp(pParent->Value(),"edge")==0) {
			std::string from;
			from.assign(pAttrib->Value());
			pAttrib=pAttrib->Next();
			std::string to;
			to.assign(pAttrib->Value());
			addEdge(from,to);
		} 
	}
	for (TiXmlNode* pChild = pParent->FirstChild(); pChild != 0; pChild = pChild->NextSibling()) {
		init( pChild );
	}
}


inline
void AStar::addNode(const std::string& id, double x, double y, double radius, bool goal)
{
	
	Node &n = nodes[id];
	n.id = id;	
	n.x = x;
	n.y = y;
	n.radius = radius;
	if (goal) {
		goals.push_back(id);
	}
}

inline
std::string& AStar::getClosestNode(double x, double y, std::string& id) const
{
	id="";
	double min = std::numeric_limits<double>::infinity();
	for (auto it = nodes.begin(); it!= nodes.end(); ++it) {
		double dis = (x - it->second.x)  * (x - it->second.x) + (y - it->second.y)  * (y - it->second.y);
		if (dis < min) {
			min = dis;
			id = it->first;
		}
	}
	return id;
}	

inline
void AStar::addEdge(const std::string& id0, const std::string& id1)
{
	double cost = heuristic_cost_estimate(&nodes.at(id0), &nodes.at(id1));
	addEdge(id0,id1,cost);
	addEdge(id1,id0,cost);

}

inline
void AStar::addEdge(const std::string& id0, const std::string& id1, double cost)
{
	nodes[id0].neighbors[id1]=cost;	
}


inline
std::list<std::string>& AStar::getPath(const std::string& start_id, const std::string& goal_id, std::list<std::string>& path)
{
	path.clear();
	std::unordered_set<std::string> closedSet;
	std::unordered_map<std::string,Node*> openSet;
	for (auto it = nodes.begin(); it != nodes.end(); ++it) {
		it->second.gScore = std::numeric_limits<double>::infinity();
		it->second.fScore = std::numeric_limits<double>::infinity();
		it->second.cameFrom = NULL;
	}
	Node* start_node = &nodes.at(start_id);
	Node* goal_node = &nodes.at(goal_id);
	start_node->gScore = 0;
	start_node->fScore = heuristic_cost_estimate(start_node,goal_node);	
	openSet[start_id] = start_node;	
	while (!openSet.empty()) {
		double min = std::numeric_limits<double>::infinity();
		auto current_it = openSet.begin();
		for (auto it = openSet.begin(); it != openSet.end(); ++it) {
			if (it->second->fScore < min) {
				current_it = it;
				min = it->second->fScore;
			}
		}
		std::string current_id = current_it->first;
		
		Node* current_node = current_it->second;		
		if (current_id == goal_id) {
			path.push_front(current_id);
			while (current_node->cameFrom != NULL) {
				current_node = current_node->cameFrom;
				path.push_front(current_node->id);
			}
			return path;
		}
		openSet.erase(current_it);		
		closedSet.insert(current_id);
		for (auto it = current_node->neighbors.begin(); it != current_node->neighbors.end(); ++it) {
			std::string neighbor_id = it->first;
			if (closedSet.count(neighbor_id)>0) {
				continue;
			}
			double neighbor_cost = it->second;
			double tentative_gScore = current_node->gScore + neighbor_cost;
			Node *neighbor_node = &nodes.at(neighbor_id);
			if (openSet.count(neighbor_id)==0) {
				openSet[neighbor_id] = neighbor_node;
			} else if (tentative_gScore >= neighbor_node->gScore) {
				continue;
			}
			neighbor_node->cameFrom = current_node;
			neighbor_node->gScore = tentative_gScore;
			neighbor_node->fScore = tentative_gScore + heuristic_cost_estimate(neighbor_node,goal_node);
		}
	}	
	return path;
}












}

#endif
