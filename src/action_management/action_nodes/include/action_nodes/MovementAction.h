/*
MovementAction.h

Purpose: This abstract class extends ExecutableAction and provides a way for the robot to move in
the environment. Could be used for a single move action or for more complex actions such as 
move and pick

@author: Michelangelo Fiore
@version: 1.0 4/9/16
 */
#ifndef MOVEMENTACTION_H
#define MOVEMENTACTION_H

#include <ros/ros.h>
#include <map>
#include <string>
#include "action_nodes/ExecutableAction.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp> 
#include <boost/thread/condition_variable.hpp>
#include <boost/algorithm/string.hpp>
#include <move_base_msgs/MoveBaseAction.h>


using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class MovementAction:public ExecutableAction {
public:

	/**
	 * Constructor of the class
	 * @param action_name name of the action
	 * @param node_handle a handle to the curret node
	 */
	MovementAction(string action_name, ros::NodeHandle node_handle);

protected:
	MoveBaseClient move_base_client_;

	/**
	 * moves the robot to a position
	 * @param  pose coordinates of the position
	 * @return      true if the movement was successfull
	 */
	bool handleMoveRequest(geometry_msgs::Pose pose);

	private:
	/**
	 * callback for the move base client
	 * @param state state of the move base client
	 * @param result result of the move base client
	 */
	void moveExecutionDoneCB(const actionlib::SimpleClientGoalState& state,
				const move_base_msgs::MoveBaseResultConstPtr& result);


	//used to check in a thread-safe way when the motion is finished
	boost::mutex mutex_is_movement_done_;
	bool is_movement_done_;
	
	/**
	 * checks if the motion has finished
	 * @return true if the motion has finished
	 */
	bool isMovementDone();

};

#endif