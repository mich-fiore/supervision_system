/*
ExecutableAction.h

Purpose: This Abstract Class extends Action and provides functionalities for actions that
can also be executed by the robot, and not only monitored.

@author: Michelangelo Fiore
@version: 1.0 4/9/16
 */



#ifndef EXECUTABLEACTION_H
#define EXECUTABLEACTION_H

#include <ros/ros.h>
#include <map>
#include <string>
#include "action_nodes/Action.h"
#include <action_management_msgs/ManageActionAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp> 
#include <boost/thread/condition_variable.hpp>
#include <boost/algorithm/string.hpp>



using namespace std;

typedef actionlib::SimpleActionServer<action_management_msgs::ManageActionAction> Server;
typedef actionlib::SimpleActionClient<action_management_msgs::ManageActionAction> Client;

class ExecutableAction:public Action {
public:

	/**
	 * Constructor of the class
	 * @param action_name name of the action
	 * @param node_handle a handle to the curret node
	 */
	ExecutableAction(string action_name, ros::NodeHandle node_handle);
	
	/**
	 * method that executes the movements of the action. Should be overridden by concrete actions
	 * @param goal message including the goal of the action
	 */
	virtual void execute(const action_management_msgs::ManageActionGoalConstPtr& goal)=0;

protected:
	//action server msgs
	action_management_msgs::ManageActionFeedback feedback_;
	action_management_msgs::ManageActionResult result_;
	Server action_server_;

	//client to execute the robot's motions
	Client motion_execution_client_;

	/**
	 * set the result message with a status and detail string
	 * @param status  high level status of the task (e.g. OK, FAILED)
	 * @param details detailed string of error (e.g. motion planner error)
	 * @param ok      if true will print an information msg on screen, if false it will print an error msg
	 */
	void setResult(string status, string details, bool ok);

	/**
	 * sends feedback to the current client
	 * @param status  high level status of the task (e.g. OK, FAILED)
	 * @param details detailed string of error (e.g. motion planner error)
	 */
	void sendFeedback(string status, string details);

	/**
	 * checks the conditions when the action should be stopped. The default behavior
	 * is never stopping. Can be overridden to put specific conditions (e.g. human too close)
	 * @param  parameters instance parameters of the action
	 * @return            true if the action should be stopped
	 */
	virtual bool shouldStop(StringMap parameters);

	/**
	 * handles requests to move the robot's body
	 * @param  goal goal of the motion
	 * @return      the results of the movement
	 */
	action_management_msgs::ManageActionResultConstPtr handleMotionRequest(const action_management_msgs::ManageActionGoalConstPtr& goal);
	
	/**
	 * handles requests to move the robot's body
	 * @param  goal goal of the motion
	 * @return      the results of the movement
	 */
	action_management_msgs::ManageActionResultConstPtr handleMotionRequest(action_management_msgs::ManageActionGoal goal);	


	/**
	 * handles requests to execute sub-actions 
	 * @param  goal goal of the sub-action
	 * @return      the results of sub-action
	 */
	action_management_msgs::ManageActionResultConstPtr handleOtherActionRequest(const action_management_msgs::ManageActionGoalConstPtr& goal, Client *action_client);
	
	/**
	 * handles requests to execute sub-actions 
	 * @param  goal goal of the sub-action
	 * @return      the results of sub-action
	 */
	action_management_msgs::ManageActionResultConstPtr handleOtherActionRequest(action_management_msgs::ManageActionGoal goal, Client *action_client);


	/**
	 * checks if the result of an action is successfull
	 * @param  result result msg
	 * @return        true if it's a success
	 */
	bool isResultSuccessfull(action_management_msgs::ManageActionResultConstPtr result);

	/**
	 * check if the name of the requested actually corresponds to this action name
	 * @param  name name of the requested action
	 * @return      true if the name is the same as this action's name
	 */
	bool checkActionName(string name);

	/**
	 * utility method to abort the current action if there has been a failure
	 * @param  result result msg
	 * @return        true if it should abort
	 */
	bool abortIfFailed(action_management_msgs::ManageActionResultConstPtr result);

	private:
	void clientExecutionDoneCB(const actionlib::SimpleClientGoalState& state,
				const action_management_msgs::ManageActionResultConstPtr& result);
	boost::mutex mutex_is_client_done_;
	bool is_client_done_;

	bool isClientDone();

};

#endif