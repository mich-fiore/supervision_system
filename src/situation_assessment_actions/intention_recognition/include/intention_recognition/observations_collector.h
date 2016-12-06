/**
 * Purpose: This class collects observations for the MDPs and BN
 * @author=Michelangelo Fiore
 */

#ifndef OBSERVATIONS_COLLECTOR_H
#define OBSERVATIONS_COLLECTOR_H

#include <ros/ros.h>
#include <IntentionGraph.h>
#include <Mdp.h>
#include <VariableSet.h>
#include <situation_assessment_msgs/Fact.h>
#include <situation_assessment_msgs/QueryDatabase.h>
#include <situation_assessment_msgs/QueryDatabase.h>

class ObservationsCollector {
	/**
	 *  collects observations for the MDPs and BN
	 */
public:
	/**
	 * Constructor of the class
	 * @param node_handle an handle to the current node
	 */
	ObservationsCollector(ros::NodeHandle node_handle);

	/**
	 * Gets the initial state of the MDPs
	 * @param  agent agent to get the state
	 * @param  mdps  list of MDPs
	 * @return       the initial state for the MDPs
	 */
	VariableSet getInitialState(std::string agent, std::vector<Mdp*> mdps);

	/**
	 * Gets the current evidence an IG
	 * @param  agent agent to get the evidence
	 * @param  ig    an intention graph
	 * @return       the evidence for the IG
	 */
	VariableSet getEvidence(std::string agent, IntentionGraph* ig);
	
	/**
	 * Gets the location of an agent
	 * @param  agent agent to check
	 * @return       the agent's location
	 */
	std::string getAt(std::string agent);

	/**
	 * Utility that queries a database and returns the first value found
	 * @param  f fact of the query
	 * @return   first value of found, or ""
	 */
	string queryDatabase(situation_assessment_msgs::Fact f);
	/**
	 * Utility that queries the database and returns all the values that are found
	 * @param f fact of the query
	 * @return vector with found values
	 */
	std::vector<std::string> queryDatabaseVector(situation_assessment_msgs::Fact f);

private:
	ros::NodeHandle node_handle_;
	ros::ServiceClient database_service_;
	std::string robot_name_;
	double reach_,close_,medium_,far_;

};

#endif