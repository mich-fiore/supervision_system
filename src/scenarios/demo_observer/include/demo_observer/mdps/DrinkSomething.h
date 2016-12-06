/**
 * Class to represent a generic MDP for an agent to drink a liquid from a container
 * @author: Michelangelo Fiore
 */
#ifndef DRINKSOMETHING_H
#define DRINKSOMETHING_H

#include "Mdp.h"

#include <string>
#include <vector>
#include "MdpBasicActions.h"
#include "ConcreteMdp.h"

using namespace std;
/**
 * Class to represent a generic MDP for an agent to drink a liquid from a container
 */
class DrinkSomething: public ConcreteMdp {
public:
    /**
     * Constructor of this class
     * @agent_name name of the agent for planning
     * @glass_name name of the recipient that the agent will drink from
     * @bottle_name name of the bottle used to fill the recipient
     * @locations list of locations
     */
    DrinkSomething(string agent_name, string glass_name, string bottle_name, 
        std::vector<std::string> locations,std::map<std::string,std::vector<std::string> > connections);
    DrinkSomething(const DrinkSomething& orig);
    virtual ~DrinkSomething();
    
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
    string bottle_loc_var_;
    string glass_loc_var_;
    // string glass_capacity_var_;
    string bottle_capacity_var_;
    string has_drunk_var_;
    string glass_contains_var_;
    
    string agent_name_;
    string glass_name_;
    string bottle_name_;
    string liquid_name_;
    
    std::map<std::string,std::vector<std::string> > connections_;
    
};
#endif /* DRINKSOMETHING_H */