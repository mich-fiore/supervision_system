/*
action_preconditions_checker.h

Purpose: this class checks which action preconditions's are satistied, publishing
executable actions in a topic

Advertised Topics:
situation_assessment/human_executable_actions


@author: Michelangelo Fiore
@version: 1.0 4/9/16
 */
#ifndef ACTION_PRECONDITONS_CHECKER
#define ACTION_PRECONDITONS_CHECKER

#include <ros/ros.h>
#include <string>
#include <vector>
#include <map>
#include <utility>

#include <common_msgs/ParameterList.h>
#include <common_msgs/Parameter.h>
#include <action_management_msgs/Action.h>
#include <action_management_msgs/CheckPreconditions.h>
#include <action_management_msgs/SetPostconditions.h>
#include <situation_assessment_actions_msgs/ExecutableActions.h>
#include <situation_assessment_actions_msgs/ExecutableAgentActions.h>

#include <situation_assessment_msgs/QueryDatabase.h>
#include <situation_assessment_msgs/FactList.h>



#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/lexical_cast.hpp>


class ActionPreconditionsChecker {
/**
 * this class checks which action preconditions's are satistied, publishing
executable actions in a topic
 */
public:
	/**
	 * Constructor of the class
	 * @parameters node_handle an handle to the current node
	 */
	ActionPreconditionsChecker(ros::NodeHandle node_handle);
	/**
	 * Starts the action preconditions process
	 */
	void start();

private:
	/**
	 * The main loop of the process
	 */
	void monitorLoop();
	/**
	 * Obtains human observations from the Database
	 */
	void databaseLoop();

	/**
	 * Callback for agent facts
	 * @param msg msg of the callback
	 */
	void agentFactCallback(const situation_assessment_msgs::FactList::ConstPtr& msg);

	/**
	 * returns the object (if any) held by the human
	 * @param  human human to query
	 * @return       object held by the agent or ""
	 */
	std::string getHumanObject(std::string human); 

	/**
	 * Sets that a human holds a particular object
	 * @param human  human to modify
	 * @param object object to set
	 */
	void setHumanObject(std::string human,std::string object);  


	ros::NodeHandle node_handle_; 
	ros::ServiceClient database_client_; //used only if we are using the database to get human observation
	ros::Subscriber fact_subscriber_; //used if we are looping on a topic to get observations

	// std::map<std::string,ros::Publisher> human_action_topics_; //actions performed by each humans are published here
	ros::Publisher human_executable_actions_pub_;

	//a std::map that links an action name with a service to get its preconditions\postconditions
	std::map<std::string,ros::ServiceClient> action_preconditions_services_;

	//values read from parameters
	std::vector<std::string> human_list_;
	std::vector<std::string> actions_to_monitor_;  
	std::vector<std::string> object_list_;
	std::map<std::string,std::vector<std::string> > object_affordances_; //links an object to possible actions
	std::vector<std::string> locations_;

	boost::mutex mutex_human_objects_;
	std::map<std::string,std::string> human_objects_;  //std::maps each human to an object that he holds

	std::string robot_name_; 

	bool use_database_; //parameter that decides if we use the db or a topic to get observations
	std::map<std::string,std::string> action_targets_;
	std::map<std::string,std::string> action_monitor_part_;

};

#endif