#include <simple_agent_monitor/data_reader.h>

DataReader::DataReader(ros::NodeHandle node_handle):node_handle_(node_handle) {
	node_handle.getParam("situation_assessment/ring_buffer_length",ring_buffer_length_);
	bool track_robot;
	bool track_agents;
	bool track_objects;
	bool track_groups;
	node_handle.getParam("situation_assessment/track_robot",track_robot);
	node_handle.getParam("situation_assessment/track_agents",track_agents);
	node_handle.getParam("situation_assessment/track_objects",track_objects);
	node_handle.getParam("situation_assessment/track_groups",track_groups);

	ROS_INFO("DATA_READER tracking: robot %d agents %d objects %d groups %d",track_robot,
		track_agents,track_objects,track_groups);

	ROS_INFO("DATA_READER   Ring buffer length is %d",ring_buffer_length_);

	robot_sub_=node_handle_.subscribe("situation_assessment/robot_pose",1000,
		&DataReader::robotCallback,this);
	agents_sub_=node_handle_.subscribe("situation_assessment/agent_poses",1000,
		&DataReader::agentsCallback,this);
	objects_sub_=node_handle_.subscribe("situation_assessment/object_poses",1000,
		&DataReader::objectsCallback,this);
	groups_sub_=node_handle_.subscribe("situation_assessment/group_poses",1000,
		&DataReader::groupsCallback,this);

	// locations_client_=node_handle_.serviceClient<situation_assessment_msgs::GetLocations>("situation_assessment/get_locations");
	// ROS_INFO("DATA_READER Waiting for location service");
	// locations_client_.waitForExistence();
	// locationsHelper();

	ROS_INFO("DATA_READER Waiting for appropriate topics to be published");
	
	ros::Rate r(3);
	while ( (
		(robot_sub_.getNumPublishers()==0 && track_robot)  
		    || (agents_sub_.getNumPublishers()==0 && track_agents)
		    || (objects_sub_.getNumPublishers()==0 && track_objects)
		    || (groups_sub_.getNumPublishers()==0 && track_groups))  && ros::ok()) {
		r.sleep();
	}	
	ROS_INFO("DATA_READER Done and ready");

	robot_pose_.pose.allocate(ring_buffer_length_);
	robot_pose_.category="agent";
	robot_pose_.type="robot";
}
void DataReader::robotCallback(situation_assessment_msgs::NamedPose msg) {
	boost::lock_guard<boost::mutex> lock(mutex_robot_poses_);

	robot_pose_.name=msg.name;
	robot_pose_.type=msg.type;
	robot_pose_.pose.insert(msg.pose);

}


void DataReader::handleEntityMap(situation_assessment_msgs::NamedPoseList msg, EntityMap* map, string category) {

	BoolMap present_poses;
	for (EntityMap::iterator i=map->begin();i!=map->end();i++) {
		present_poses[i->first]=false;
	}
	BOOST_FOREACH(situation_assessment_msgs::NamedPose pose,msg.poses) {
		if ((*map)[pose.name].pose.isEmpty()) {
			(*map)[pose.name].pose.allocate(ring_buffer_length_);
		}
		(*map)[pose.name].pose.insert(pose.pose);
		(*map)[pose.name].name=pose.name;
		(*map)[pose.name].type=pose.type;
		(*map)[pose.name].category=category;
		present_poses[pose.name]=true;
	}

	for (BoolMap::iterator i=present_poses.begin(); i!=present_poses.end();i++) {
		if (i->second==false) {
			map->erase(i->first);
		}
	}
}


void DataReader::agentsCallback(situation_assessment_msgs::NamedPoseList msg) {
	boost::lock_guard<boost::mutex> lock(mutex_agent_poses_);

	handleEntityMap(msg,&agent_poses_map_,"agent");

}
void DataReader::objectsCallback(situation_assessment_msgs::NamedPoseList msg) {
	boost::lock_guard<boost::mutex> lock(mutex_object_poses_);

	handleEntityMap(msg,&object_poses_map_,"object");

}

void DataReader::groupsCallback(situation_assessment_msgs::GroupList msg) {
	boost::lock_guard<boost::mutex> lock(mutex_group_poses_);


	BoolMap present_groups;
	for (EntityMap::iterator i=group_poses_map_.begin(); i!=group_poses_map_.end(); i++) {
		present_groups[i->first]=false;
	}

	BOOST_FOREACH(situation_assessment_msgs::Group group, msg.list) {
		if (group_poses_map_[group.name].pose.isEmpty()) {
			group_poses_map_[group.name].pose.allocate(ring_buffer_length_);
			group_poses_map_[group.name].pose.insert(group.pose);
		
			present_groups[group.name]=true;

			agent_groups_map_[group.name]=group.members;
		}
	}


	for (BoolMap::iterator i=present_groups.begin(); i!=present_groups.end();i++) {
		if (i->second==false) {
			group_poses_map_.erase(i->first);
			agent_groups_map_.erase(i->first);
		}
	}

}

void DataReader::locationsHelper() {
	situation_assessment_msgs::GetLocations srv;
	if (locations_client_.call(srv)) {
		situation_assessment_msgs::NamedPoseList named_pose_list;
		for (int i=0; i<srv.response.locations.size();i++) {
			situation_assessment_msgs::NamedPose named_pose;
			named_pose.name=srv.response.locations[i];
			named_pose.type="location";
			named_pose.pose.position.x=srv.response.centers[i].x;
			named_pose.pose.position.y=srv.response.centers[i].y;
			named_pose.pose.position.z=srv.response.centers[i].z;
			named_pose.pose.orientation.w=1;
			named_pose_list.poses.push_back(named_pose);
		
			location_areas_[named_pose.name]=srv.response.areas[i];
		}

		handleEntityMap(named_pose_list,&location_poses_map_,"location");
	}
	else {
		ROS_WARN("DATA_READER Couldn't obtain locations");
	}
}

GeometryPolygonMap DataReader::getLocationsAreas() {
	return location_areas_;
}


EntityMap DataReader::getAgentPoses() {
	boost::lock_guard<boost::mutex> lock(mutex_agent_poses_);
	return agent_poses_map_;
}
EntityMap DataReader::getGroupPoses() {
	boost::lock_guard<boost::mutex> lock(mutex_group_poses_);
	return group_poses_map_;

}
StringVectorMap DataReader::getAgentGroups() {
	boost::lock_guard<boost::mutex> lock(mutex_group_poses_);
	return agent_groups_map_;
}
EntityMap DataReader::getObjectPoses() {
	boost::lock_guard<boost::mutex> lock(mutex_object_poses_);
	return object_poses_map_;
}
Entity DataReader::getRobotPoses() {
	boost::lock_guard<boost::mutex> lock(mutex_robot_poses_);
	return robot_pose_;
}

EntityMap DataReader::getLocationPoses() {
	return location_poses_map_;
}

