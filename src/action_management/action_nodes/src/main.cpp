#include <ros/ros.h>
#include "action_nodes/basic_actions/basic_pick.h"
#include "action_nodes/basic_actions/basic_place.h"

/*
this main limits itself to creating a basic pick and place and starting the services
 */
int main(int argc, char ** argv) {
	ros::init(argc,argv,"basic_actions");
	ros::NodeHandle node_handle;

	ROS_INFO("BASIC ACTIONS - started node");

	BasicPick pick(node_handle);
	BasicPlace place(node_handle);
	ros::spin();
}