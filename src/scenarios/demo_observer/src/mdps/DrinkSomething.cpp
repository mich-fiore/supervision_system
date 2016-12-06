/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   DrinkSomething.cpp
 * Author: mfiore
 * 
 * Created on August 12, 2016, 11:22 AM
 */

#include "demo_observer/mdps/DrinkSomething.h"

DrinkSomething::DrinkSomething(std::string agent_name, std::string glass_name, std::string bottle_name, 
    std::vector<std::string> locations,std::map<std::string,std::vector<std::string> > connections):agent_name_(agent_name),glass_name_(glass_name),
bottle_name_(bottle_name),connections_(connections) {
    agent_loc_var_ = agent_name_+"_isAt";
    bottle_loc_var_ = bottle_name_+"_isAt";
    glass_loc_var_ = glass_name_+"_isAt";
    // glass_capacity_var_ = glass_name_+"_capacity";
    bottle_capacity_var_ = bottle_name_+"_capacity";
    has_drunk_var_ = agent_name_+"_hasDrunk";
    glass_contains_var_=glass_name_+"_contains";

    variables_.push_back(agent_loc_var_);
    variables_.push_back(bottle_loc_var_);
    variables_.push_back(glass_loc_var_);
    // variables_.push_back(glass_capacity_var_);
    variables_.push_back(bottle_capacity_var_);
    // variables_.push_back(has_drunk_var_);
    variables_.push_back(glass_contains_var_);

    std::map<string, std::vector<string> > var_values;
    var_values[agent_loc_var_] = locations;
    var_values[bottle_loc_var_] = locations;
    var_values[bottle_loc_var_].push_back(agent_name_);
    var_values[glass_loc_var_] = locations;
    var_values[glass_loc_var_].push_back(agent_name_);
    var_values[bottle_capacity_var_] = {"0", "1"};
    // var_values[glass_capacity_var_] = {"0", "1"};
    // var_values[has_drunk_var_] = {"0", "1"};
    var_values[glass_contains_var_] = {bottle_name_, "nothing" ,"other_liquid"};

    this->var_values_ = var_values;

    abstract_states_[agent_loc_var_]["human1"] = "other_agent";
    abstract_states_[agent_loc_var_]["human2"] = "other_agent";
    abstract_states_[glass_contains_var_]["waterbottle"] = "other_liquid";
    abstract_states_[glass_contains_var_]["teabottle"] = "other_liquid";


    std::vector<string> actions;
    for (string l : locations) {
        actions.push_back(agent_name_ + "_move_" + l);
        actions.push_back(agent_name_+"_place_"+glass_name_+"_"+l);
        actions.push_back(agent_name_+"_place_"+bottle_name_+"_"+l);
    }
    actions.push_back(agent_name_ + "_pick_" + glass_name_);
    actions.push_back(agent_name_ + "_pick_" + bottle_name_);
    actions.push_back(agent_name_ + "_fill_" + bottle_name_ + "_" + glass_name_);
    // actions.push_back(agent_name_ + "_drink_" + glass_name_);

    this->actions_ = actions;

    parameters_.push_back(agent_name_);
    vector<string> par_var;
    par_var.push_back(agent_loc_var_);
    // par_var.push_back(has_drunk_var_);
    parameter_variables_[agent_name_] = par_var;
    variable_parameter_[par_var[0]] = agent_name_;

    name_ = "agent_drink_something";

}

DrinkSomething::DrinkSomething(const DrinkSomething& orig) {
}

DrinkSomething::~DrinkSomething() {
}

VarStateProb DrinkSomething::transitionFunction(VariableSet state, string action) {
    string agent_isAt = state.set[agent_loc_var_];
    string glass_isAt = state.set[glass_loc_var_];
    string waterbottle_isAt = state.set[bottle_loc_var_];
    string bottle_capacity = state.set[bottle_capacity_var_];
    // string has_drunk = state.set[has_drunk_var_];
    string glass_contains = state.set[glass_contains_var_];

    vector<string> action_parameters = MdpBasicActions::getActionParameters(action);
    VarStateProb future_beliefs;
    string action_name = action_parameters[1];


    if (action_name == "pick" && action_parameters[2] == glass_name_ && waterbottle_isAt!=agent_name_) {
        future_beliefs = MdpBasicActions::applyTake(agent_isAt, glass_isAt, agent_name_, glass_loc_var_, state);
    } else if (action_name == "pick" && action_parameters[2] == bottle_name_ && glass_isAt!=agent_name_) {
        future_beliefs = MdpBasicActions::applyTake(agent_isAt, waterbottle_isAt, agent_name_, bottle_loc_var_, state);
    } else if (action_name == "move") {
        future_beliefs = MdpBasicActions::applyMove(agent_loc_var_, action_parameters[2], state,
            connections_,agent_isAt);
    } else if (action_name == "fill" && waterbottle_isAt == agent_name_ && 
        glass_isAt ==agent_isAt && bottle_capacity == "1" && glass_contains=="nothing") {
        VariableSet new_s = state;
        new_s.set[glass_contains_var_]=bottle_name_;
        future_beliefs[new_s] = 1;
    }
    // } else if (action_name == "drink" && glass_isAt == agent_name_ && glass_contains == bottle_name_) {
    //     VariableSet new_s = state;
    //     // new_s.set[has_drunk_var_] = "1";
    //     new_s.set[glass_contains_var_]="nothing";
    //     future_beliefs[new_s] = 1;
    // }
    else if (action_name=="place" && action_parameters[2]==glass_name_) {
        future_beliefs = MdpBasicActions::applyPlace(glass_loc_var_,glass_isAt, agent_isAt,action_parameters[3],
            agent_name_,state);
    }    
    else if (action_name=="place" && action_parameters[2]==bottle_name_) {
        future_beliefs = MdpBasicActions::applyPlace(bottle_loc_var_,waterbottle_isAt, agent_isAt,action_parameters[3],
            agent_name_,state);
    }

    return future_beliefs;
}

int DrinkSomething::rewardFunction(VariableSet state, string action) {
    // string agent_isAt = state.set[agent_loc_var_];
    // string glass_isAt = state.set[glass_loc_var_];
    // string waterbottle_isAt = state.set[bottle_loc_var_];
    // // string glass_capacity = state.set[glass_capacity_var_];
    // string bottle_capacity = state.set[bottle_capacity_var_];
    // string glass_contains = state.set[glass_contains_var_];

    // if (glass_isAt == agent_name_ && glass_contains ==bottle_name_ && action == agent_name_ + "_drink_" + glass_name_) {
    //     return 100;
    // } else return 0;
}

bool DrinkSomething::isGoalState(VariableSet state) {
    string agent_isAt = state.set[agent_loc_var_];
    string glass_isAt = state.set[glass_loc_var_];
    string waterbottle_isAt = state.set[bottle_loc_var_];
    string bottle_capacity = state.set[bottle_capacity_var_];
    // string has_drunk = state.set[has_drunk_var_];
    string glass_contains = state.set[glass_contains_var_];

    return glass_isAt==agent_name_ && glass_contains==bottle_name_;
}

bool DrinkSomething::isStartingState(VariableSet state) {

    return !isGoalState(state);
}
