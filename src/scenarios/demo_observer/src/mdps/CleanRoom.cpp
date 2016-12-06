/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   CleanRoom.cpp
 * Author: mfiore
 * 
 * Created on August 12, 2016, 11:22 AM
 */

#include "demo_observer/mdps/CleanRoom.h"

CleanRoom::CleanRoom(std::string agent_name, std::vector<std::string> locations):
agent_name_(agent_name) {
    waterbottle_name_="waterbottle";
    teabottle_name_="teabottle";
    keys_name_="keys";
    remote_name_="remote";
    glass_name_="glass";
    book_name_="book";
    food_name_="food";


    agent_loc_var_ = agent_name_+"_isAt";
    waterbottle_loc_var_ = waterbottle_name_+"_isAt";
    teabottle_loc_var_ = teabottle_name_+"_isAt";
    glass_loc_var_=glass_name_+"_isAt";
    remote_loc_var_=remote_name_+"_isAt";
    keys_loc_var_=keys_name_+"_isAt";
    book1_loc_var_=book_name_+"1_isAt";
    book2_loc_var_=book_name_+"2_isAt";
    book3_loc_var_=book_name_+"3_isAt";
    food_loc_var_=food_name_+"_isAt";

    waterbottle_placement_="table";
    teabottle_placement_="shelf2";
    keys_placement_="shelf1";
    glass_placement_="table";
    remote_placement_="sidetable";
    book_placement_="shelf3";
    food_placement_="table";


    // variables_.push_back(agent_loc_var_);
    variables_.push_back(waterbottle_loc_var_);
    variables_.push_back(teabottle_loc_var_);
    variables_.push_back(glass_loc_var_);
    variables_.push_back(remote_loc_var_);
    variables_.push_back(book1_loc_var_);
    variables_.push_back(book2_loc_var_);
    variables_.push_back(book3_loc_var_);
    variables_.push_back(food_loc_var_);

    
 
    std::map<string, std::vector<string> > var_values;
    // var_values[agent_loc_var_] = locations;
    var_values[waterbottle_loc_var_] = {waterbottle_placement_,"other"};
    var_values[teabottle_loc_var_] = {teabottle_placement_,"other"};
    var_values[glass_loc_var_] = {glass_placement_,"other"};
    var_values[remote_loc_var_]={remote_placement_,"other"};
    var_values[keys_loc_var_]={keys_placement_,"other"};
    var_values[book1_loc_var_] = {book_placement_,"other"};
    var_values[book2_loc_var_] = {book_placement_,"other"};
    var_values[book3_loc_var_] = {book_placement_,"other"};
    var_values[food_loc_var_] = {food_placement_,"other"};

    this->var_values_ = var_values;

    abstract_states_[waterbottle_loc_var_]["shelf1"] = "other";
    abstract_states_[waterbottle_loc_var_]["shelf2"] = "other";
    abstract_states_[waterbottle_loc_var_]["shelf3"] = "other";
    abstract_states_[waterbottle_loc_var_]["sofa"] = "other";
    abstract_states_[waterbottle_loc_var_]["table"] = "other";
    abstract_states_[waterbottle_loc_var_]["sidetable"] = "other";
    abstract_states_[waterbottle_loc_var_]["livingroom"] = "other";
    abstract_states_[waterbottle_loc_var_]["bathroom"] = "other";
    abstract_states_[waterbottle_loc_var_]["outside"] = "other";
    abstract_states_[waterbottle_loc_var_]["human1"] = "other";

    abstract_states_[teabottle_loc_var_]=abstract_states_[waterbottle_loc_var_];
    abstract_states_[glass_loc_var_]=abstract_states_[waterbottle_loc_var_];
    abstract_states_[remote_loc_var_]=abstract_states_[waterbottle_loc_var_];
    abstract_states_[keys_loc_var_]=abstract_states_[waterbottle_loc_var_];
    abstract_states_[book1_loc_var_]=abstract_states_[waterbottle_loc_var_];
    abstract_states_[book2_loc_var_]=abstract_states_[waterbottle_loc_var_];
    abstract_states_[book3_loc_var_]=abstract_states_[waterbottle_loc_var_];
    abstract_states_[food_loc_var_]=abstract_states_[waterbottle_loc_var_];

    std::vector<string> actions;
    for (string l : locations) {
        actions.push_back(agent_name_ + "_move_" + l);
    }
    actions.push_back(agent_name_ + "_clean_" + glass_name_+"_"+glass_placement_);
    actions.push_back(agent_name_ + "_clean" + waterbottle_name_+"_"+glass_placement_);
    actions.push_back(agent_name_ + "_clean" + teabottle_name_+"_"+teabottle_placement_);
    actions.push_back(agent_name_ + "_clean" + remote_name_+"_"+remote_placement_);
    actions.push_back(agent_name_ + "_clean" + keys_name_+"_"+keys_placement_);
    actions.push_back(agent_name_ + "_clean" + food_name_+"_"+food_placement_);
    actions.push_back(agent_name_ + "_clean" + book_name_+"1"+"_"+book_placement_);
    actions.push_back(agent_name_ + "_clean" + book_name_+"2"+"_"+book_placement_);
    actions.push_back(agent_name_ + "_clean" + book_name_+"3"+"_"+book_placement_);


    CleanObject* clean_object=new CleanObject(agent_name_,locations);
    hierarchy_map_[agent_name_ + "_clean_" + glass_name_+"_"+glass_placement_]=clean_object;
    hierarchy_map_[agent_name_ + "_clean_" + waterbottle_name_+"_"+glass_placement_]=clean_object;
    hierarchy_map_[agent_name_ + "_clean_" + teabottle_name_+"_"+teabottle_placement_]=clean_object;
    hierarchy_map_[agent_name_ + "_clean_" + remote_name_+"_"+remote_placement_]=clean_object;
    hierarchy_map_[agent_name_ + "_clean_" + keys_name_+"_"+keys_placement_]=clean_object;
    hierarchy_map_[agent_name_ + "_clean_" + food_name_+"_"+food_placement_]=clean_object;
    hierarchy_map_[agent_name_ + "_clean_" + book_name_+"1"+"_"+book_placement_]=clean_object;
    hierarchy_map_[agent_name_ + "_clean_" + book_name_+"2"+"_"+book_placement_]=clean_object;
    hierarchy_map_[agent_name_ + "_clean_" + book_name_+"3"+"_"+book_placement_]=clean_object;
    this->actions_ = actions;

    parameters_.push_back(agent_name_);
    vector<string> par_var;
    par_var.push_back(agent_loc_var_);
    parameter_variables_[agent_name_] = par_var;
    variable_parameter_[par_var[0]] = agent_name_;

    name_ = "agent_clean_room";

}

CleanRoom::CleanRoom(const CleanRoom& orig) {
}

CleanRoom::~CleanRoom() {
}

VarStateProb CleanRoom::transitionFunction(VariableSet state, string action) {


    VarStateProb future_beliefs;
   

   return future_beliefs;
}

int CleanRoom::rewardFunction(VariableSet state, string action) {

}

bool CleanRoom::isGoalState(VariableSet state) {
    string agent_isAt=state.set[agent_loc_var_];
    string waterbottle_isAt=state.set[waterbottle_loc_var_];
    string teabottle_isAt=state.set[teabottle_loc_var_];
    string glass_isAt=state.set[glass_loc_var_];
    string keys_isAt=state.set[keys_loc_var_];
    string remote_isAt=state.set[remote_loc_var_];
    string food_isAt=state.set[food_loc_var_];
    string book1_isAt=state.set[book1_loc_var_];
    string book2_isAt=state.set[book2_loc_var_];
    string book3_isAt=state.set[book3_loc_var_];

    return waterbottle_isAt==waterbottle_placement_ &&
            teabottle_isAt==teabottle_placement_ &&
            glass_isAt==glass_placement_ &&
            keys_isAt==keys_placement_ &&
            remote_isAt==remote_placement_ && 
            food_isAt==food_placement_ &&
            book1_isAt==book_placement_ &&
            book2_isAt==book_placement_ &&
            book3_isAt==book_placement_; 

}

bool CleanRoom::isStartingState(VariableSet state) {
    return !isGoalState(state);
}
