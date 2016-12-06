/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   ReadBook.cpp
 * Author: mfiore
 * 
 * Created on August 12, 2016, 11:22 AM
 */

#include "demo_observer/mdps/ReadBook.h"

ReadBook::ReadBook(string agent_name, string read_location, std::vector<std::string> locations,
    std::map<std::string,std::vector<std::string> > connections):
agent_name_(agent_name),read_location_name_(read_location),connections_(connections) {
    agent_loc_var_ = agent_name_+"_isAt";
    book_name_="book";
    book1_loc_var_=book_name_+"1_isAt";
    book2_loc_var_=book_name_+"2_isAt";
    book3_loc_var_=book_name_+"3_isAt";

    variables_.push_back(agent_loc_var_);
    variables_.push_back(book1_loc_var_);
    variables_.push_back(book2_loc_var_);
    variables_.push_back(book3_loc_var_);

    std::map<string, std::vector<string> > var_values;
    var_values[agent_loc_var_] = locations;
    var_values[book1_loc_var_] = locations;
    var_values[book1_loc_var_].push_back(agent_name_);   
    var_values[book2_loc_var_] = locations;
    var_values[book2_loc_var_].push_back(agent_name_);    
    var_values[book3_loc_var_] = locations;
    var_values[book3_loc_var_].push_back(agent_name_);

    this->var_values_ = var_values;

    abstract_states_[agent_loc_var_]["human1"] = "other_agent";
    abstract_states_[agent_loc_var_]["human2"] = "other_agent";

    std::vector<string> actions;
    for (string l : locations) {
        actions.push_back(agent_name_ + "_move_" + l);
    }
    for (int i=1;i<4;i++) {
        actions.push_back(agent_name_ + "_pick_" + book_name_+to_string(i));
    }
    this->actions_ = actions;

    parameters_.push_back(agent_name_);
    vector<string> par_var;
    par_var.push_back(agent_loc_var_);
    parameter_variables_[agent_name_] = par_var;
    variable_parameter_[par_var[0]] = agent_name_;


    name_ = "agent_read_book";

}

ReadBook::ReadBook(const ReadBook& orig) {
}

ReadBook::~ReadBook() {
}

VarStateProb ReadBook::transitionFunction(VariableSet state, string action) {
    string agent_isAt = state.set[agent_loc_var_];
    string book1_loc=state.set[book1_loc_var_];
    string book2_loc=state.set[book2_loc_var_];
    string book3_loc=state.set[book3_loc_var_];

    vector<string> action_parameters = MdpBasicActions::getActionParameters(action);
    VarStateProb future_beliefs;
    string action_name = action_parameters[1];

    bool has_book=book1_loc==agent_name_ || book2_loc==agent_name_ || book3_loc==agent_name_;

    if (action_name == "pick" && !has_book && action_parameters[2]==(book_name_+"1")) {
        future_beliefs = MdpBasicActions::applyTake(agent_isAt, book1_loc, agent_name_,
         book1_loc_var_, state);
    }    
    else if (action_name == "pick" && !has_book && action_parameters[2]==(book_name_+"2")) {
        future_beliefs = MdpBasicActions::applyTake(agent_isAt, book2_loc, agent_name_,
         book2_loc_var_, state);
    }    
    else if (action_name == "pick" && !has_book && action_parameters[2]==(book_name_+"3")) {
        future_beliefs = MdpBasicActions::applyTake(agent_isAt, book3_loc, agent_name_,
         book3_loc_var_, state);
    } 
     else if (action_name == "move") {
        future_beliefs = MdpBasicActions::applyMove(agent_loc_var_, action_parameters[2], state,
            connections_,agent_isAt);
    }
    return future_beliefs;
}

int ReadBook::rewardFunction(VariableSet state, string action) {

}

bool ReadBook::isGoalState(VariableSet state) {
    string book1_loc=state.set[book1_loc_var_];
    string book2_loc=state.set[book2_loc_var_];
    string book3_loc=state.set[book3_loc_var_];
    string agent_loc=state.set[agent_loc_var_];
    bool has_book=book1_loc==agent_name_ || book2_loc==agent_name_ || book3_loc==agent_name_;

    return has_book && agent_loc==read_location_name_;
}

bool ReadBook::isStartingState(VariableSet state) {
    string book1_loc=state.set[book1_loc_var_];
    string book2_loc=state.set[book2_loc_var_];
    string book3_loc=state.set[book3_loc_var_];
    string agent_loc=state.set[agent_loc_var_];
    bool has_book=book1_loc==agent_name_ || book2_loc==agent_name_ || book3_loc==agent_name_;

    return !has_book || agent_loc==read_location_name_;
}
