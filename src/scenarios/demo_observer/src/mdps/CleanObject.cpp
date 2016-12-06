/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   CleanObject.cpp
 * Author: mfiore
 * 
 * Created on August 12, 2016, 11:22 AM
 */

#include "demo_observer/mdps/CleanObject.h"

CleanObject::CleanObject(std::string agent_name, std::vector<std::string> locations):
agent_name_(agent_name) {
    object_name_="object";

    agent_loc_var_ = agent_name_+"_isAt";
    object_loc_var_ = object_name_+"_isAt";

    object_placement_="placement";

    variables_.push_back(agent_loc_var_);
    variables_.push_back(object_loc_var_);

    std::map<string, std::vector<string> > var_values;
    var_values[agent_loc_var_] = locations;
    var_values[agent_loc_var_].push_back(object_placement_);
    var_values[object_loc_var_] = locations;
    var_values[object_loc_var_].push_back(agent_name_);
    var_values[object_loc_var_].push_back(object_placement_);

    this->var_values_ = var_values;

    abstract_states_[agent_loc_var_]["human1"] = "other_agent";
    abstract_states_[agent_loc_var_]["human2"] = "other_agent";

    std::vector<string> actions;
    for (string l : locations) {
        actions.push_back(agent_name_ + "_move_" + l);
    }
    actions.push_back(agent_name_ + "_move_" + object_placement_);


    actions.push_back(agent_name_ + "_pick_" + object_name_);

    actions.push_back(agent_name_+"_place_"+object_name_+"_"+object_placement_);

    this->actions_ = actions;

    parameter_action_place_[0] = agent_name_;
    parameter_action_place_[2] = object_name_;
    parameter_action_place_[3] = object_placement_;

    parameters_.push_back(agent_name_);
    vector<string> par_var;
    par_var.push_back(agent_loc_var_);
    parameter_variables_[agent_name_] = par_var;
    variable_parameter_[par_var[0]] = agent_name_;

    parameters_.push_back(object_name_);
    // vector<string> par_var;
    par_var.clear();
    par_var.push_back(object_loc_var_);
    parameter_variables_[object_name_] = par_var;
    variable_parameter_[par_var[0]]=object_name_;

    // parameters_.push_back(object_name_);
    // par_var.clear();
    // par_var.push_back(object_loc_var_);
    // parameter_variables_[object_loc_var_] = par_var;
    // variable_parameter_[par_var[0]] = object_name_;

    parameters_.push_back(object_placement_);

    name_ = "agent_clean_object";


}

CleanObject::CleanObject(const CleanObject& orig) {
}

CleanObject::~CleanObject() {
}





VarStateProb CleanObject::transitionFunction(VariableSet state, string action) {
    string agent_isAt=state.set[agent_loc_var_];
    string object_isAt=state.set[object_loc_var_];

    vector<string> action_parameters = MdpBasicActions::getActionParameters(action);
    VarStateProb future_beliefs;
    string action_name = action_parameters[1];

    if (action_name == "pick" &&  object_isAt!=agent_name_) {
         future_beliefs= MdpBasicActions::applyTake(agent_isAt, object_isAt, agent_name_,
            object_loc_var_, state);
    }
    else if (action_name=="place") {
        // cout<<"in place\n";
        // state.set[object_isAt]="shelf3";
        // future_beliefs[state]=1;
        future_beliefs = MdpBasicActions::applyPlace(object_loc_var_,object_isAt, agent_isAt,
            object_placement_, agent_name_,state);

        // MdpBasicActions::applyPlace(object_loc_var_, object_isAt, human_isAt, goal_location_,
                        // agent_name_, state)
    }
    else if (action_name == "move") {
        future_beliefs = MdpBasicActions::applyMove(agent_loc_var_, action_parameters[2], state
           );
    }
    else {
        future_beliefs[state]=1;
    }

   return future_beliefs;
}

int CleanObject::rewardFunction(VariableSet state, string action) {

}

bool CleanObject::isGoalState(VariableSet state) {
    string agent_isAt=state.set[agent_loc_var_];
    string object_isAt=state.set[object_loc_var_];
    return object_isAt==object_placement_;
}

bool CleanObject::isStartingState(VariableSet state) {
    string agent_isAt=state.set[agent_loc_var_];
    string object_isAt=state.set[object_loc_var_];
    return !isGoalState(state);
}
