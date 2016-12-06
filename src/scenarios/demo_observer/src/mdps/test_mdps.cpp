/**
 * Simple Node to test an MDP
 * Receives as argument the name of the MDP to test
 * @author Michelangelo Fiore
 */


#include <ros/ros.h>
#include "demo_observer/mdps/DrinkSomething.h"
#include "demo_observer/mdps/ReadBook.h"
#include "demo_observer/mdps/GoOut.h"
#include "demo_observer/mdps/WatchTv.h"

#include "Mdp.h"
#include <string>
#include <vector>
#include <map>

int main(int argc, char** argv) {
	ros::init(argc,argv,"test_mdps");

	ROS_INFO("TEST_MDPS - started node test-mdps");

	string mdp_name=argv[1];

	ROS_INFO("TEST_MDPS - mdp name is %s",mdp_name.c_str());

	std::vector<std::string> locations = {"table", "sidetable", "bathroom", "outside", "shelf1", "shelf2", "shelf3", "sofa"};
	  std::map<std::string, std::string> initial_state;
	  initial_state["agent_isAt"] = "table";
	  initial_state["mug_isAt"] = "table";
	  initial_state["waterbottle_isAt"] = "table";
	  initial_state["teabottle_isAt"] = "shelf1";
	  initial_state["has_drunk"] = "0";
	  initial_state["mug_contains"] = "nothing";
	  initial_state["waterbottle_capacity"] = "1";
	  initial_state["teabottle_capacity"] = "1";
	  initial_state["book1_isAt"] = "shelf2";
	  initial_state["book2_isAt"] = "shelf2";
	  initial_state["book3_isAt"] = "table";
	  initial_state["keys_isAt"] = "shelf1";
	  initial_state["remote_isAt"] = "table";
	  VariableSet v_i;
	  v_i.set = initial_state;

	  std::map<std::string,std::string> parameter_map;
	  parameter_map["agent"]="agent";
	  parameter_map["object"]="mug";
	  parameter_map["placement"]="table";
	  
	  ROS_INFO("Reading mdp");

	  Mdp drink_water;
	  if (drink_water.readMdp("/home/theworld/ros_workspaces/indigo_ws/src/scenarios/demo_observer/mdp_models/"+mdp_name)) {
	  	ROS_INFO("Read mdp");
	  	drink_water.assignParameters(parameter_map);
		  drink_water.simulate(8, v_i);
		// drink_water.printTransitionFunction();
		}
		else {
			ROS_WARN("Error reading mdp");
		}

}