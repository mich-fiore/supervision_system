/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   GoOut.cpp
 * Author: mfiore
 * 
 * Created on August 12, 2016, 11:22 AM
 */

#include "demo_observer/mdps/GoOut.h"

GoOut::GoOut(string agent_name, std::vector<std::string> locations,
    std::map<std::string,std::vector<std::string> > connections):
agent_name_(agent_name),connections_(connections) {
    agent_loc_var_ = agent_name_+"_isAt";
    keys_name_="keys";
    keys_loc_var_=keys_name_+"_isAt";
    outside_name_="outside";

    variables_.push_back(agent_loc_var_);
    variables_.push_back(keys_loc_var_);

    std::map<string, std::vector<string> > var_values;
    var_values[agent_loc_var_] = locations;
    var_values[keys_loc_var_] = locations;
    var_values[keys_loc_var_].push_back(agent_name_);   

    this->var_values_ = var_values;

    abstract_states_[agent_loc_var_]["human1"] = "other_agent";
    abstract_states_[agent_loc_var_]["human2"] = "other_agent";

    std::vector<string> actions;
    for (string l : locations) {
        actions.push_back(agent_name_ + "_move_" + l);
    }
    actions.push_back(agent_name_+"_pick_"+keys_name_);

    this->actions_ = actions;

    parameters_.push_back(agent_name_);
    vector<string> par_var;
    par_var.push_back(agent_loc_var_);
    parameter_variables_[agent_name_] = par_var;
    variable_parameter_[par_var[0]] = agent_name_;


    name_ = "agent_go_out";

}

GoOut::GoOut(const GoOut& orig) {
}

GoOut::~GoOut() {
}

VarStateProb GoOut::transitionFunction(VariableSet state, string action) {
    string agent_isAt = state.set[agent_loc_var_];
    string keys_loc=state.set[keys_loc_var_];

    vector<string> action_parameters = MdpBasicActions::getActionParameters(action);
    VarStateProb future_beliefs;
    string action_name = action_parameters[1];

    if (action_name == "pick" && keys_loc!=agent_name_ ) {
        future_beliefs = MdpBasicActions::applyTake(agent_isAt, keys_loc, agent_name_,
         keys_loc_var_, state);
    }    
    else if (action_name == "move") {
        future_beliefs = MdpBasicActions::applyMove(agent_loc_var_, action_parameters[2], state,
            connections_,agent_isAt);
    }
    return future_beliefs;
}

int GoOut::rewardFunction(VariableSet state, string action) {

}

bool GoOut::isGoalState(VariableSet state) {
    string keys_loc=state.set[keys_loc_var_];
    string agent_loc=state.set[agent_loc_var_];

    return keys_loc==agent_name_ && agent_loc==outside_name_;
}

bool GoOut::isStartingState(VariableSet state) {
    string keys_loc=state.set[keys_loc_var_];
    string agent_loc=state.set[agent_loc_var_];

    return keys_loc!=agent_name_ || agent_loc!=outside_name_;
}
