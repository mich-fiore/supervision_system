/**
	data_lib.h
	author: Michelangelo Fiore

	defines the superclass used by other data bridges. The system is generic and can receive perception data from different
	sources, or bridges.

	The published topics are:
  	situation_assessment/robot_pose
	situation_assessment/agent_poses
	situation_assessment/object_poses
	situation_assessment/group_poses
*/


#ifndef DATALIB_H
#define DATALIB_H

#include <ros/ros.h>

#include <vector>
#include <map>

#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/foreach.hpp>

#include <situation_assessment_msgs/NamedPoseList.h>
#include <situation_assessment_msgs/NamedPose.h>
#include <situation_assessment_msgs/Group.h>
#include <situation_assessment_msgs/GroupList.h>



using namespace std;


/**
 * agents or objects
 */
struct Entity {
	string name;
	string type;
	string category;
	geometry_msgs::Pose pose;
};

//useful typedefs
typedef map<string,Entity> EntityMap;
typedef map<string,string> StringMap;
typedef map<string,vector<string> > StringVectorMap;

/**
 * Superclass used by other data bridges. These bridges can track different entities
 * (robot human object groups) and publish them in topics used by the rest of the system
 */
class DataLib {
public:
	
	/**
	 * Constructor of the class
	 * @param bridge_name the name of the brige
	 * @node_handle an handle to the current node
	 */
	DataLib(string bridge_name, ros::NodeHandle node_handle);
	
	/**
	 * Publishes the tracked entities datas in the appropriate topics
	 */
	void publishData();

protected:
	/**
	 * produced a msg with names and poses of the agents from an EntityMap
	 * @param  entity_map map containing entities and their poses
	 * @return            a NamedPoseList msg, generated from the entity map
	 */
	situation_assessment_msgs::NamedPoseList getNamedPoseListMsg(EntityMap entity_map); //poses of entities

	string bridge_name_;

	//list of agents and objects known by the system
	vector<string> agent_list_;
	vector<string> object_list_;
	string robot_name_;

	//each bridge can track or not agents, the robot, groups or objects
	bool track_agents_;
	bool track_robot_;
	bool track_groups_;
	bool track_objects_;

	//poses of the entities
	Entity robot_pose_;
	EntityMap agent_poses_;
	EntityMap group_poses_;
	EntityMap object_poses_;
	StringVectorMap agent_groups_;
	ros::Publisher robot_pub_,agents_pub_,objects_pub_,groups_pub_;

	std::vector<std::string> body_parts_;


	ros::NodeHandle node_handle_;
};

#endif