/*
drink.h

Purpose: Simple action to drink from a mug
@author: Michelangelo Fiore
@version: 1.0 4/9/16
 */

#ifndef Drink_H
#define Drink_H

#include <ros/ros.h>

#include <action_nodes/Action.h>
#include <string>
#include <vector>
#include <situation_assessment_msgs/Fact.h>

/**
 *  Action to drink from a mug. Used to monitor th ehuman 
 */
class Drink: public Action {
public:
    /**
     * Constructor of the class
     * @parameters node_handle handle of this node
     */
	Drink(ros::NodeHandle node_handle);
	
protected:
    /**
     * Checks the preconditions for this action
     * @param  parameters parameter instance for this action
     * @return            true if the action's preconditions are satisfied
     */
	bool checkPreconditions(StringMap parameters);
    /**
     * Sets the postconditions for this action
     * @param parameters parameter instance for this action
     */
	void setPostconditions(StringMap parameters);
};

#endif