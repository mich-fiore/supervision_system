/**
 * MDP to clean the books in the room
 * @author: Michelangelo Fiore
 */
#ifndef CLEANBOOK_H
#define CLEANBOOK_H

#include "Mdp.h"

#include <string>
#include <vector>
#include "MdpBasicActions.h"
#include "ConcreteMdp.h"

using namespace std;

/**
 * MDP to clean the books in the room
 */
class CleanBooks: public ConcreteMdp {
public:
    /**
     * Constructor of this class
     * @agent_name name of the agent for planning
     * @read_location location where the agent should read
     * @locations list of locations
     * @connections links each location to its connected locations
     */
    CleanBooks(string agent_name, string read_location, std::vector<std::string> locations,
        std::map<std::string,std::vector<std::string> > connections);
    CleanBooks(const CleanBooks& orig);
    virtual ~CleanBooks();
    
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
    
    //variable names
    string agent_loc_var_;
    string book1_loc_var_;
    string book2_loc_var_;
    string book3_loc_var_;
    
    //some useful names to avoid doing error in the code
    string agent_name_;
    string book_name_;
    string place_location_name_;

    //list of connections
    std::map<std::string,std::vector<std::string> > connections_;
    
};
#endif /* CleanBooks_H */