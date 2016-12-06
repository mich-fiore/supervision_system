/*
action_monitors.h

Purpose: this class checks which actions are executed

Advertised Topics:
/situation_assessment/humans_executed_actions


@author: Michelangelo Fiore
@version: 1.0 4/9/16
 */

#ifndef ACTION_MONITORS_H
#define ACTION_MONITORS_H
#include <ros/ros.h>
#include <string>
#include <vector>
#include <utility>
#include <set>

#include <action_management_msgs/Action.h>
#include <action_management_msgs/ActionList.h>
#include <common_msgs/ParameterList.h>
#include <common_msgs/Parameter.h>
#include <action_management_msgs/CheckPreconditions.h>
#include <action_management_msgs/SetPostconditions.h>

#include <situation_assessment_msgs/QueryDatabase.h>
#include <situation_assessment_actions_msgs/ExecutableActions.h>
#include <situation_assessment_actions_msgs/ExecutableAgentActions.h>


#include <situation_assessment_msgs/FactList.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/thread.hpp>
#include <boost/lexical_cast.hpp>

#include "supervision_timer/supervision_timer.h"

using namespace std;

class ActionMonitors {
/**
 * this class checks which actions are executed. These actions are then published
 * in a topic
 */
public:
	/**
	 * Constructor of the class
	 * @param @node_handle an handle to this node
	 */
	ActionMonitors(ros::NodeHandle node_handle);
	/**
	 * Main loop of this process
	 */
	void actionLoop();
private:
	/**
	 * Callback for the actions that are executable
	 * @param msg msg of the callback
	 */
	void executableActionsCallback(
	 	const situation_assessment_actions_msgs::ExecutableActions::ConstPtr& msg);

	/**
	 * Converts a vector of parameter msgs to a map of parameters, which can be used
	 * more easily in the class
	 * @param parameter_message vector of parameter messages
	 * @return a map of parameters
	 */
	std::map<std::string,std::string> getParameterMap(
		std::vector<common_msgs::Parameter> parameter_message);

	/**
	 * Gets the distance between the monitor part of the agent and a target
	 * (ex. agent's arm to bottle) 
	 * @param  agent        name of the agent
	 * @param  target       name of the target
	 * @param  monitor_part body part of the agent
	 * @return              distance between monitor part and target
	 */
	double getDistance(string agent, string target, string monitor_part);

	/**
	 * Get all the possible move actions
	 * @return vector of move actions
	 */
	std::vector<action_management_msgs::Action> getMoveActions();


	ros::NodeHandle node_handle_; 
	ros::ServiceClient database_query_client_; //used only if we are using the database to get human observation
	ros::Subscriber fact_subscriber_; //used if we are looping on a topic to get observations

	ros::Subscriber executable_actions_subscriber_;
	vector<situation_assessment_actions_msgs::ExecutableAgentActions> executable_actions_;  
	vector<std::string> actions_to_monitor_;

	ros::Publisher executed_actions_pub_;


	//a map that links an action name with a service to get its postconditions
	map<string,ros::ServiceClient> action_postconditions_services_;

	//values read from parameters
	vector<string> object_list_;
	map<string,vector<string> > object_affordances_; //links an object to possible actions
	vector<string> human_list_;
	map<string,string> human_locations_;


	double trigger_distance_;  //trigger for deciding that an action is done

	string robot_name_; 

	bool use_database_; //parameter that decides if we use the db or a topic to get observations

	ros::ServiceClient database_client_;  //puts facts in the db

	std::map<std::string,std::string> action_monitor_parts_;
	std::map<std::string,std::string> action_targets_;

	double time_threshold_;

	std::map<std::string,SupervisionTimer*> agent_timers_;
	std::map<std::string,boost::thread*> timers_threads_; 

	ros::Subscriber inference_sub_;

};


#endif