/**
 * MDP for an agente to read a book
 * @author: Michelangelo Fiore
 */
#ifndef READBOOK_H
#define READBOOK_H

#include "Mdp.h"

#include <string>
#include <vector>
#include "MdpBasicActions.h"
#include "ConcreteMdp.h"

using namespace std;

/**
 * MDP for an agente to read a book
 */
class ReadBook: public ConcreteMdp {
public:
   /**
    * Constructor of this class
    * @agent_name name of the agent for planning
    * @read_location location where the agent will read
    * @locations list of locations of the demo
    * @connections links a location to its connected locations
    */
    ReadBook(string agent_name, string read_location, std::vector<std::string> locations,
        std::map<std::string,std::vector<std::string> > connections);
    ReadBook(const ReadBook& orig);
    virtual ~ReadBook();
    
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
    string book1_loc_var_;
    string book2_loc_var_;
    string book3_loc_var_;
    
    string agent_name_;
    string book_name_;
    string read_location_name_;
    std::map<std::string,std::vector<std::string> > connections_;
    
    
};
#endif /* READBOOK_H */