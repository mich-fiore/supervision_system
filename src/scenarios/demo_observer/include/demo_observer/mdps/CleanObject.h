/**
 * MDP to clean the 
 * @author: Michelangelo Fiore
 */
#ifndef CLEANOBJECT_H
#define CLEANOBJECT_H

#include "Hmdp.h"

#include <string>
#include <vector>
#include "MdpBasicActions.h"

using namespace std;
class CleanObject: public Hmdp {
public:
    /**
     * Constructor of this class
     * @agent_name name of the agent for planning
     * @locations list of locations
     */
    CleanObject(string agent_name, std::vector<std::string> locations);
    CleanObject(const CleanObject& orig);
    virtual ~CleanObject();
    
private:

    /**
     * Transition function of the MDP 
     * @param state the starting state of the transition
     * @param action the action of the transition
     * @return the belief of the MDP after the transition
     */
    std::map<VariableSet, double> transitionFunction(VariableSet state, string action);
    
    /**
     * Reward function of the mdp
     * @param  state  state of the reward function
     * @param  action action of the reward function
     * @return        the reward
     */
    int rewardFunction(VariableSet state, string action);

    /**
     * Checks if a state is a goal state
     * @param  state state to check
     * @return       true if the state is a goal state
     */
    bool isGoalState(VariableSet state);

    /**
     * Checks if a state is a starting state
     * @param  state the state to check
     * @return       true if the state is a starting state
     */
    bool isStartingState(VariableSet state);
    
    string agent_loc_var_;
    string object_loc_var_;

    string agent_name_;
    string object_name_;
    string object_placement_;
    
    
    
};
#endif