/*
basic_action.h

Purpose: this node is able to receive data from the optitrack mocap system and to forward
these information in TF.
@author: Michelangelo Fiore
 */



#include <ros/ros.h>

#include <string>
#include <vector>
#include <map>

#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <optitrack_tf/or_pose_estimator_state.h>
#include "optitrack_tf/or_t3d_pos.h"
#include "tf/transform_datatypes.h"

std::vector<ros::Subscriber> subs_;  //subscribers from optitrack
std::map<std::string,geometry_msgs::Pose> body_poses_;
std::vector<std::string> body_names_;  //parameter containing the names of the rigid bodies that we want to track with optitrack
std::string topic_base_; //optitrack topics have the format topic_base/body_name



/**
 * Callback on the optitrack topic
 * @param msg  msg received from the callback
 * @param name the name of this rigid body
 */
void optitrackCallback(const optitrack_tf::or_pose_estimator_state::ConstPtr& msg, 
	const std::string name) {
     if (msg->pos.size()>0)    {
		geometry_msgs::Pose new_pose;
		new_pose.position.x=msg->pos[0].x;
		new_pose.position.y=msg->pos[0].y;
		new_pose.position.z=msg->pos[0].z;
		new_pose.orientation.w=msg->pos[0].qw;
		new_pose.orientation.x=msg->pos[0].qx;
		new_pose.orientation.y=msg->pos[0].qy;
		new_pose.orientation.z=msg->pos[0].qz;
		
		body_poses_[name]=new_pose;    
    }

}

/**
 * Sets subscriber based on the body_names_ parameter	
 * @param node_handle handle of this node
 */
void setSubscribers(ros::NodeHandle node_handle) {
	for (std::string body: body_names_) {
		ROS_INFO("OPTITRACK_TF Body %s",body.c_str());
		ROS_INFO("OPTITRACK_TF Topic is %s",(topic_base_+body).c_str());
		subs_.push_back(node_handle.subscribe<optitrack_tf::or_pose_estimator_state>(topic_base_+body,1,boost::bind(optitrackCallback, _1, body)));
		
		ROS_INFO("OPTITRACK_TF Waiting for publisher");

		while (subs_[subs_.size()-1].getNumPublishers()==0) {
			ros::Duration(0.3).sleep();
		}
		ROS_INFO("OPTITRACK_TF Connected");
		// subs_.push_back(&sub);
	}
} 





int main(int argc, char** argv) {
	ros::init(argc,argv,"optitrack_tf");
	ros::NodeHandle node_handle;

	ROS_INFO("OPTITRACK_TF starting node");

	node_handle.getParam("optitrack/topic_base",topic_base_);
	node_handle.getParam("optitrack/rigid_bodies",body_names_);
	setSubscribers(node_handle);
	ROS_INFO("OPTITRACK_TF set subscribers");

	tf::TransformBroadcaster br;
	ros::Rate r(3);

	while (ros::ok()){
		ros::spinOnce();
		for (std::string body:body_names_) {
			if (body_poses_.find(body)!=body_poses_.end()) {
				tf::Transform transform;
				geometry_msgs::Pose pose=body_poses_[body];

				tf::Quaternion q;
				tf::quaternionMsgToTF(pose.orientation,q);
				double roll, pitch, yaw;
				tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
				yaw=-yaw; //the frame of optitrack and map doesn't completely correspond
				transform.setOrigin( tf::Vector3(pose.position.x,pose.position.y,pose.position.z));
				transform.setRotation(tf::createQuaternionFromRPY(roll,pitch,yaw));
				br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", body));
			}
		}
		r.sleep();
	}
}