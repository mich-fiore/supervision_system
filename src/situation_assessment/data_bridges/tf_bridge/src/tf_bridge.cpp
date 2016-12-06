#include <tf_bridge/tf_bridge.h>

TfBridge::TfBridge(ros::NodeHandle node_handle):DataLib("tf_bridge",node_handle) {

}



geometry_msgs::Pose TfBridge::tfToGeometry(tf::StampedTransform transform) {
	geometry_msgs::Pose pose;
	pose.position.x=transform.getOrigin().getX();
	pose.position.y=transform.getOrigin().getY();
	pose.position.z=transform.getOrigin().getZ();
	quaternionTFToMsg(transform.getRotation(),pose.orientation);
	return pose;
}

void TfBridge::getPoses() {
	//get human agents
	geometry_msgs::Pose new_group_pose;
	new_group_pose.position.x=0;
	new_group_pose.position.y=0;
	new_group_pose.position.z=0;

	if (track_agents_) {
		BOOST_FOREACH(string agent_name,agent_list_) {
			
			try {

				BOOST_FOREACH(string part,body_parts_) {

					string tf_name=agent_name+"_"+part;
					tf::StampedTransform transform;
					// ROS_INFO("TF_BRIDGE in track_agents_");

					listener_.waitForTransform("map", tf_name, ros::Time(0), ros::Duration(1.0) );
					if (!ros::ok()) {
						return;
					}
					// ROS_INFO("TF_BRIDGE in track_agents_");

					listener_.lookupTransform("map",tf_name,ros::Time(0),transform);

					// ROS_INFO("TF_BRIDGE in track_agents_");

					geometry_msgs::Pose pose=tfToGeometry(transform);
					agent_poses_[tf_name].pose=pose;
					agent_poses_[tf_name].name=agent_name;
					agent_poses_[tf_name].type="human";
					new_group_pose.position.x=new_group_pose.position.x+transform.getOrigin().getX();
					new_group_pose.position.y=new_group_pose.position.y+transform.getOrigin().getY();
					new_group_pose.position.z=new_group_pose.position.z+transform.getOrigin().getZ();
					// ROS_INFO("TF_BRIDGE in track_agents_");

				}
			}   catch (tf::TransformException ex){
				// ROS_ERROR("%s",ex.what());
			}
			
		}
	}

	if (track_agents_ && track_groups_) {
		new_group_pose.position.x=new_group_pose.position.x/agent_list_.size();
		new_group_pose.position.y=new_group_pose.position.y/agent_list_.size();
		new_group_pose.position.z=new_group_pose.position.z/agent_list_.size();
		group_poses_["GROUP_1"].pose=new_group_pose;
		group_poses_["GROUP_1"].name="GROUP_1";
		group_poses_["GROUP_1"].type="group";
		agent_groups_["GROUP_1"]=agent_list_;

		BOOST_FOREACH(string agent, agent_list_) {
			group_poses_[agent].pose=agent_poses_[agent].pose;
			group_poses_[agent].name=agent;
			group_poses_[agent].type="self_group";
			vector<string> self_members;
			self_members.push_back(agent);
			agent_groups_[agent]=self_members;
		}
	}
	if (track_objects_) {
		for (int i=0; i<object_list_.size(); i++) {
			try {
				tf::StampedTransform transform;
				listener_.waitForTransform("map", object_list_[i], ros::Time(0), ros::Duration(1.0) );
				if (!ros::ok()) {
					return;
				}
				listener_.lookupTransform("map",object_list_[i],ros::Time(0),transform);

				geometry_msgs::Pose pose=tfToGeometry(transform);
				// ROS_INFO("TF_BRIDGE quaternion for %s %f %f %f %f",object_list_[i].c_str(),
					// pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w);
				
				pose.orientation.w=1;
				object_poses_[object_list_[i]].pose=pose;
				object_poses_[object_list_[i]].name=object_list_[i];
			}     catch (tf::TransformException ex){
				// ROS_ERROR("%s",ex.what());
			}
		}

	}
	if (track_robot_) {
		try {
			tf::StampedTransform transform;
			listener_.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(1.0) );
			if (!ros::ok()) {
					return;
				}
			listener_.lookupTransform("map","base_link",ros::Time(0),transform);
			geometry_msgs::Pose pose=tfToGeometry(transform);
			robot_pose_.pose=pose;
			robot_pose_.name=robot_name_;
			robot_pose_.type="robot";
		} catch(tf::TransformException ex) {
			ROS_ERROR("TF_BRIDGE  %s",ex.what());
		}
	}

}



