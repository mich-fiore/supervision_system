/*
Action.h

Purpose:This Abstract Class provides a standard interface that can be used both to execute the robot's 
actions and to monitor human's actions. Subclasses should override the checkPreconditions and
setPostconditions methods, which are provides as a ros service by the node and can be accessed
by other nodes. The idea of this method is that we can add new actions without recompiling the
planning\execution\monitoring stack, but only by adding a new node that instantiates one or more
actions and  provides the related services.

Actions that can be only monitored should extend this class. For actions that can also be executed by
the robot there are two subclasses: ExecutableAction and MovementAction

@author: Michelangelo Fiore
@version: 1.0 4/9/16
 */

#ifndef ACTION_H
#define ACTION_H

#include <ros/ros.h>
#include <map>
#include <string>
#include <action_management_msgs/GetActionParameters.h>
#include <action_management_msgs/GetActionParameters.h>
#include <situation_assessment_msgs/QueryDatabase.h>
#include <situation_assessment_msgs/DatabaseRequest.h>
#include <action_management_msgs/CheckPreconditions.h>
#include <action_management_msgs/SetPostconditions.h>
#include <common_msgs/Parameter.h>


using namespace std;

typedef map<string,string> StringMap;

class Action {
public:
	/**
	 * Creates an action object
	 *
	 * @param action_name: the name of the action
	 * @param node_handle: the handle of the current node
	 *
	 */
	Action(string action_name, ros::NodeHandle node_handle);
protected:
	string action_name_;  //name of the action
	vector<string> parameters_; //list of parameters of the action
	string robot_name_; //name of the robot 

	ros::NodeHandle node_handle_; //handle of the node

	//clients for the database functions
	ros::ServiceClient database_query_client_;
	ros::ServiceClient database_add_facts_client_;
	ros::ServiceClient database_remove_facts_client_;
	ros::ServiceClient database_set_facts_client_;

	//services provided by the action
	ros::ServiceServer get_parameters_server;
	ros::ServiceServer check_preconditions_server_;
	ros::ServiceServer set_postconditions_server_;

	/**
	 * checks if the preconditions of the actions are satisfied and it can be executed. Should
	 * be overriden by concrete actions
	 * @param  parameters instantiation of the parameters 
	 * @return            true if the preconditions are satisfied
	 */
	virtual bool checkPreconditions(StringMap parameters)=0;
	/**
	 * sets the postconditions of the action. Should be overridden by concrete actions
	 * @param parameters instantation of the parameters
	 */
	virtual void setPostconditions(StringMap parameters)=0;
	/**
	 * Converts a parameter  message into a map, easier to use in the methods
	 * @param  parameters parameters instantiation of the action
	 * @return            a map with parameter name and value
	 */
	StringMap extractParametersFromMsg(vector<common_msgs::Parameter> parameters);

	/**
	 * queries the database returning the first value found (often it's enough)
	 * @param  query  fact that needs to be found in the db
	 * @return       a string representing the value or "" if nothing is found
	 */
	string queryDatabase(situation_assessment_msgs::Fact query);

	/**
	 * queries the database returning all the values found
	 * @param query fact that needs to be found in the db
	 * @return a vector of all the values found
	 */
	vector<string> queryDatabaseComplete(situation_assessment_msgs::Fact query);

	/**
	 * adds facts in the database, overwriting them if they are already there.
	 * useful when a fact can have a single value (e.g. the human can be only
	 * in one location)
	 * @param facts vector of facts that needs to be added
	 */
	void setFacts(std::vector<situation_assessment_msgs::Fact> facts);

	/**
	 * adds facts in the database, without overwriting. Useful when a fact can have
	 * more than one value (e.g. the human can have more than one object in hand)
	 * @param facts [description]
	 */
	void addFacts(std::vector<situation_assessment_msgs::Fact> facts);

	/**
	 * remove facts from the db
	 * @param facts vector of facts that should be removed
	 */
	void removeFacts(std::vector<situation_assessment_msgs::Fact> facts);

	/**
	 * check if every needed parameter has been received in the request
	 * @param  request_parameters map containing the parameters of the request
	 * @return                    true if the parameters are present
	 */
	bool checkParameterPresence(StringMap request_parameters);



private:
	/**
	 * service that can be used to get a list of parameters used by the action
	 * @param  req request of the service
	 * @param  res response of the service
	 * @return     true if the service is called correctly
	 */
	bool getParameters(action_management_msgs::GetActionParameters::Request  &req,
         action_management_msgs::GetActionParameters::Response &res);

	/**
	 * service that can be used to check the preconditions of an action
	 * @param  req request of the service
	 * @param  res response of the service
	 * @return     true if the service is called correctly
	 */
	bool checkPreconditionsService(action_management_msgs::CheckPreconditions::Request &req,
		action_management_msgs::CheckPreconditions::Response &res);
	
	/**
	 * service that can be used to set the postconditions of an action
	 * @param  req request of the service
	 * @param  res response of the service
	 * @return     true if the service is called correctly
	 */
	bool setPostconditionsService(action_management_msgs::SetPostconditions::Request &req,
		action_management_msgs::SetPostconditions::Response &res);
};
#endif