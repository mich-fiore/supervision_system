/**
    data_reader.h
    author: Michelangelo Fiore

	Produces entity maps for the tracked entities (robot, agents, objects, groups, locations)
*/

#ifndef DATA_READER_H
#define DATA_READER_H

#include <ros/ros.h>

#include <situation_assessment_msgs/NamedPose.h>
#include <situation_assessment_msgs/NamedPoseList.h>
#include <situation_assessment_msgs/Group.h>
#include <situation_assessment_msgs/GroupList.h>
#include <situation_assessment_msgs/GetLocations.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>


#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

#include <simple_agent_monitor/ring_buffer.h>

#include <boost/foreach.hpp>

#include <simple_agent_monitor/math_functions.h>

#include <simple_agent_monitor/entity.h>

typedef map<string,Entity> EntityMap;
typedef map<string,bool> BoolMap;
typedef map<string,vector<string> > StringVectorMap;
typedef map<string,geometry_msgs::Polygon> GeometryPolygonMap;


using namespace std;

/**
 * 	Produces entity maps for the tracked entities (robot, agents, objects, groups, locations)
 */
class DataReader {
public:
	/**
	 * Constructor of this class
	 * @param node_handle an handle for this node
	 */
	DataReader(ros::NodeHandle node_handle);

	/**
	 * Returns the poses of the tracked agents
	 * @return an EntityMap with the agent poses
	 */
	EntityMap getAgentPoses();

	/**
	 * Returns the poses of the centroid of the tracked groups
	 * @return an EntityMap for the gorup poses
	 */
	EntityMap getGroupPoses();
	/**
	 * Calculates information related to membership of agents in groups
	 * @return a map linking a group to a vector of agents
	 */
	StringVectorMap getAgentGroups();
	/**
	 * Returns the poses of the tracked objects
	 * @return an EntityMap with the position of the tracked objects
	 */
	EntityMap getObjectPoses();
	/**
	 * Returns the pose of the robot
	 * @return an EntityMap with the pose of the robot
	 */
	Entity getRobotPoses();
	/**
	 * Return the positions of the known locations
	 * @return an EntityMap with the positions of the known locations
	 */
	EntityMap getLocationPoses();
	/**
	 * Returns the polygons of the known areas
	 * @return a map linking an area to a polygon
	 */
	GeometryPolygonMap getLocationsAreas();

private:
	/**
	 * Callback for the robot position. Updates the EntityMap of the robot
	 * @param msg msg of this callback
	 */
	void robotCallback(situation_assessment_msgs::NamedPose msg);
	/**
	 * Callback for the agent positions. Updates the EntityMap of the agents
	 * @param msg the msg of the callback
	 */
	void agentsCallback(situation_assessment_msgs::NamedPoseList msg);
	/**
	 * Callback for the object positions. Updates the EntityMap of the objects
	 * @param msg the msg of the callback
	 */
	void objectsCallback(situation_assessment_msgs::NamedPoseList msg);
	/**
	 * Callback for the group positions. Updates the EntityMap of the groups
	 * @param msg the msg of the callback
	 */
	void groupsCallback(situation_assessment_msgs::GroupList msg);
	/**
	 * Utility method to create an entity map of locations by using a ros
	 * service
	 */
	void locationsHelper();

	/**
	 * Transforms a NamedPoseList msg in an EntityMap
	 * @param msg      msg containing name of entities and their poses
	 * @param map      the entity map to update
	 * @param category type of entity
	 */
	void handleEntityMap(situation_assessment_msgs::NamedPoseList msg, EntityMap* map, string category);


	ros::NodeHandle node_handle_;

	ros::Subscriber agents_sub_, groups_sub_, objects_sub_, robot_sub_;

	ros::ServiceClient locations_client_;

	boost::mutex mutex_agent_poses_, mutex_group_poses_, mutex_object_poses_, mutex_robot_poses_;

	EntityMap agent_poses_map_, group_poses_map_, object_poses_map_,location_poses_map_;
	Entity robot_pose_;

	StringVectorMap agent_groups_map_;

	GeometryPolygonMap location_areas_;

	int ring_buffer_length_;

};

#endif