/*
basic_action.h

Purpose: this class provides a simple implementation of ExecutableAction providing an execute method
that limits itself to checking preconditions, calling a move request with the name of the action, 
waiting for the results, setting postconditions. Concrete actions that exend BaseAction should still
override the checkPreconditions and setPostconditions methods

@author: Michelangelo Fiore
@version: 1.0 4/9/16
 */

#ifndef BASIC_ACTION_H
#define BASIC_ACTION_H

#include <action_nodes/ExecutableAction.h>

class BasicAction:public ExecutableAction {
public:
	/**
	 * Constructor of the class
	 * @param action_name name of the action
	 * @param node_handle a handle to the curret node
	 */
	BasicAction(string action_name, ros::NodeHandle node_handle);

	/**
	 * method that executes the movements of the action. Should be overridden by concrete actions
	 * @param goal message including the goal of the action
	 */
	void execute(const action_management_msgs::ManageActionGoalConstPtr& goal);

};

#endif