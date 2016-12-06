/**
    agent_monitors.h
    author: Michelangelo Fiore

     procedures to calculate geometrical
     information from perception data of agents.

*/

#ifndef AGENT_MONITORS_H
#define AGENT_MONITORS_H

#include <ros/ros.h>
#include <map>
#include <vector>
#include <string>
#include <boost/polygon/polygon.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

#include <simple_agent_monitor/entity.h>

#include <simple_agent_monitor/math_functions.h>

#include <situation_assessment_msgs/Fact.h> 

namespace gtl = boost::polygon;

typedef gtl::polygon_data<double> Polygon;
typedef gtl::polygon_traits<Polygon>::point_type Point;


using namespace std;

typedef map<string,Entity> EntityMap;
typedef map<string,bool> BoolMap;
typedef map<string,vector<string> > StringVectorMap;
typedef map<string,Polygon> PolygonMap; 
typedef map<pair<string, string> ,RingBuffer<double> >  PairMap; 


/**
 * This class contains different methods to compute facts related to geometrical
 * reasoning of agents
 */
class AgentMonitors {
public:
    /**
     * Constructor of the class
     * @param robot_name the name of the robot
     */
	AgentMonitors(string robot_name);


    /**
     * Calculates which entites agents are facing
     * @param map1 the map with all entities which we want to check the facing
     * @param map2 all entities which we will be checking the orientation against
       @return returns a vector with the isFacing facts
     */
    vector<situation_assessment_msgs::Fact> calculateIsFacing(EntityMap map1, EntityMap map2); 

    /**
     * Generates facts to represent which entities are included in a gropu
     * @param map which links a group to its entities
     * @return a vector of generated facts
     */
    vector<situation_assessment_msgs::Fact> getGroupContains(StringVectorMap map); 
    /**
     * Generates facts related to the distance between entities
     * @param map1 starting entities to check the distance 
     * @param map2 ending entities to check the distance
     * @return a vector of generated facts
     */
    vector<situation_assessment_msgs::Fact> getDistances(EntityMap map1, EntityMap map2, PairMap* 
    	entity_distances); 

    /**
     * Generates facts to represent if entites are moving
     * @param map map containing entities information
     * @return a vector of generated facts
     */
    vector<situation_assessment_msgs::Fact> getIsMoving(EntityMap map); //calculates if entities are moving or not
    
    /**
     * Generates facts related to the type of entities (objects, agents...)
     * @param map map containing entities information
     * @return a vector of generated facts
     */
    vector<situation_assessment_msgs::Fact> getEntityType(EntityMap map); 
    /**
     * Generates facts related to the delta distances of entities
     * @param map1 the started entities of these computations
     * @param map2 the ending entities of these computations
     * @return a vector of generated facts
     */
    vector<situation_assessment_msgs::Fact> getDeltaDistances(EntityMap map1, EntityMap map2,
    PairMap entity_distances); 

    /**
     * Generates facts related to the presence of entities in areas
     * @param map information about entities
     * @param areas map linking an area to a polygon
     * @return a vector of generated facts
     */
    vector<situation_assessment_msgs::Fact> getIsInArea(EntityMap map, PolygonMap areas);  //which agents are present in areas
    
    /**
     * Generates facts to represent if an agent is linked to an area
     * @param areas list of areas
     * @return a vector of generated facts 
     */
    vector<situation_assessment_msgs::Fact> getHasArea(vector<string> areas); 

    /**
     * Generates facts related to the poses of entities
     * @param map entities information
     * @return a vector of generated facts
     */
    vector<situation_assessment_msgs::Fact> getEntityPoses(EntityMap map);
    
    /**
     * Generates facts related to the location of entities. The idea is that
     * an agent can be in several areas, but we will generate a single "at"
     * fact to represent its location, based on the maximum depth area he is in.
     * @param depth areas a map linking an area to its depth (areas can include other areas)
     * @is_in_area_facts list of areas where an agent is at the moment
     * @return a vector of generated facts
     */
    std::vector<situation_assessment_msgs::Fact> getAt(
        std::map<std::string,int> depth_areas, std::vector<situation_assessment_msgs::Fact> is_in_area_facts);

	private:
        string robot_name_;
};
#endif