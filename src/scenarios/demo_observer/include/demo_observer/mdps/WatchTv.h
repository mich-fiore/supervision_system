 /**
  * MDP for an agente to watch tv
  * @author: Michelangelo Fiore
  */
#ifndef WATCHTV_H
#define WATCHTV_H

#include "Mdp.h"

#include <string>
#include <vector>
#include "MdpBasicActions.h"
#include "ConcreteMdp.h"

using namespace std;
/**
 * MDP for an agent to watch tv
 */
class WatchTv: public ConcreteMdp {
public:
     /**
     * Constructor of this class
     * @agent_name name of the agent for planning
     * @locations list of locations
     * @connections links a location to its connected locations
     */
    WatchTv(string agent_name, std::vector<std::string> locations,
        std::map<std::string,std::vector<std::string> > connections);
    WatchTv(const WatchTv& orig);
    virtual ~WatchTv();
    
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
    string remote_loc_var_;
    
    string agent_name_;
    string remote_name_;
    string sofa_name_;

    std::map<std::string,std::vector<std::string> > connections_;
};
#endif /* WATCHTV_H */