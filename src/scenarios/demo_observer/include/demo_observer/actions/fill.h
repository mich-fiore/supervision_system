/*
drink.h

Purpose: Simple action to fill a mug with water or tea
@author: Michelangelo Fiore
@version: 1.0 4/9/16
 */


#ifndef Fill_H
#define Fill_H

#include <ros/ros.h>

#include <action_nodes/Action.h>
#include <string>
#include <vector>
#include <situation_assessment_msgs/Fact.h>

/*
Action class to fill a mug with water or tea. Used to monitor the human.
*/
class Fill: public Action {
	public:
    /**
     * Constructor of the class
     * @parameters node_handle handle of this node
     */    
	Fill(ros::NodeHandle node_handle);
	
protected:
    /**
     * Checks the preconditions for this action
     * @param  parameters parameter instance for this action
     * @return            true if the action's preconditions are satisfied
     */
	bool checkPreconditions(StringMap parameters);
	/**
     * Sets the preconditions for this action
     * @param  parameters parameter instance for this action
     */
    void setPostconditions(StringMap parameters);
};

#endif