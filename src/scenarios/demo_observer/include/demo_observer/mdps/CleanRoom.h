/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Clean.h
 * Author: mfiore
 *
 * Created on August 12, 2016, 11:22 AM
 */

#ifndef CLEAN_H
#define CLEAN_H

#include "Mdp.h"

#include <string>
#include <vector>
#include "MdpBasicActions.h"
#include "Hmdp.h"
#include "CleanObject.h"

using namespace std;
class CleanRoom: public Hmdp {
public:
    /**
     * Constructor of this class
     * @agent_name name of the agent for planning
     * @locations list of locations
     */
    CleanRoom(string agent_name, std::vector<std::string> locations);
    CleanRoom(const CleanRoom& orig);
    virtual ~CleanRoom();
    
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
    string waterbottle_loc_var_;
    string teabottle_loc_var_;
    string glass_loc_var_;
    string book1_loc_var_;
    string book2_loc_var_;
    string book3_loc_var_;
    string keys_loc_var_;
    string remote_loc_var_;
    string food_loc_var_;

    
    string agent_name_;
    string glass_name_;
    string waterbottle_name_;
    string teabottle_name_;
    string keys_name_;
    string remote_name_;
    string book_name_;
    string food_name_;


    string waterbottle_placement_;
    string teabottle_placement_;
    string keys_placement_;
    string glass_placement_;
    string remote_placement_;
    string book_placement_;
    string food_placement_;
    
    
    
};

#endif