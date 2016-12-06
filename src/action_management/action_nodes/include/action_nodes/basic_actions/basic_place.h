/*
basic_pick.h

Purpose: this class provides a simple place function, that tries to put an object in the robot's hand
to a reachable support.

@author: Michelangelo Fiore
@version: 1.0 4/9/16
 */

#ifndef PLACE_H
#define PLACE_H

#include <ros/ros.h>

#include <action_nodes/basic_actions/basic_action.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <situation_assessment_msgs/PutObjectInHand.h>
#include <situation_assessment_msgs/PlaceObject.h>

class BasicPlace: public BasicAction {
	public:
	/**
	 * creates the basic pick action
	 * @param  node_handle an handle to the current node
	 */
	BasicPlace(ros::NodeHandle node_handle);

protected:
	/**
	 * checks if the preconditions for a pick are satisfied
	 * @param  parameters instantiation of the pick request parameeters
	 * @return            true if the preconditions are satisfied
	 */
	bool checkPreconditions(StringMap parameters);
	
	/**
	 * sets the postconditions of the pick
	 * @param parameters instantiation of the pick request parameters
	 */
	void setPostconditions(StringMap parameters);

private:
	//service that can be used to say that the object is no longer in the robot's hand
	ros::ServiceClient put_object_in_hand_client_;
	//service that can be used to tell the system that the object has been placed on the support
	ros::ServiceClient place_object_client_;
};

#endif