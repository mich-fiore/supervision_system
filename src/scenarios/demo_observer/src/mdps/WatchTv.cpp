/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   WatchTv.cpp
 * Author: mfiore
 * 
 * Created on August 12, 2016, 11:22 AM
 */

#include "demo_observer/mdps/WatchTv.h"

WatchTv::WatchTv(string agent_name, std::vector<std::string> locations,
    std::map<std::string,std::vector<std::string> > connections):
agent_name_(agent_name),connections_(connections) {
    agent_loc_var_ = agent_name_+"_isAt";
    remote_name_="remote";
    remote_loc_var_=remote_name_+"_isAt";
    sofa_name_="sofa";

    variables_.push_back(agent_loc_var_);
    variables_.push_back(remote_loc_var_);

    std::map<string, std::vector<string> > var_values;
    var_values[agent_loc_var_] = locations;
    var_values[remote_loc_var_] = locations;
    var_values[remote_loc_var_].push_back(agent_name_);   

    this->var_values_ = var_values;

    abstract_states_[agent_loc_var_]["human1"] = "other_agent";
    abstract_states_[agent_loc_var_]["human2"] = "other_agent";

    std::vector<string> actions;
    for (string l : locations) {
        actions.push_back(agent_name_ + "_move_" + l);
    }
    actions.push_back(agent_name_+"_pick_"+remote_name_);

    this->actions_ = actions;

    parameters_.push_back(agent_name_);
    vector<string> par_var;
    par_var.push_back(agent_loc_var_);
    parameter_variables_[agent_name_] = par_var;
    variable_parameter_[par_var[0]] = agent_name_;


    name_ = "agent_go_out";

}

WatchTv::WatchTv(const WatchTv& orig) {
}

WatchTv::~WatchTv() {
}

VarStateProb WatchTv::transitionFunction(VariableSet state, string action) {
    string agent_isAt = state.set[agent_loc_var_];
    string remote_loc=state.set[remote_loc_var_];

    vector<string> action_parameters = MdpBasicActions::getActionParameters(action);
    VarStateProb future_beliefs;
    string action_name = action_parameters[1];

    if (action_name == "pick" && remote_loc!=agent_name_ ) {
        future_beliefs = MdpBasicActions::applyTake(agent_isAt, remote_loc, agent_name_,
         remote_loc_var_, state);
    }    
    else if (action_name == "move") {
        future_beliefs = MdpBasicActions::applyMove(agent_loc_var_, action_parameters[2], state,
            connections_,agent_isAt);
    }
    return future_beliefs;
}

int WatchTv::rewardFunction(VariableSet state, string action) {

}

bool WatchTv::isGoalState(VariableSet state) {
    string remote_loc=state.set[remote_loc_var_];
    string agent_loc=state.set[agent_loc_var_];

    return remote_loc==agent_name_ && agent_loc==sofa_name_;
}

bool WatchTv::isStartingState(VariableSet state) {
    string remote_loc=state.set[remote_loc_var_];
    string agent_loc=state.set[agent_loc_var_];

    return remote_loc!=agent_name_ || agent_loc!=sofa_name_;
}
