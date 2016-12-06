/**
 * Simple Node to create and solve an MDP
 * Receives as argument the name of the MDP to create
 * @author Michelangelo Fiore
 */

#include <ros/ros.h>
#include "demo_observer/mdps/DrinkSomething.h"
#include "demo_observer/mdps/ReadBook.h"
#include "demo_observer/mdps/GoOut.h"
#include "demo_observer/mdps/WatchTv.h"
#include "demo_observer/mdps/CleanRoom.h"
#include "demo_observer/mdps/CleanObject.h"
#include "demo_observer/mdps/CleanBooks.h"


#include <string>
#include <vector>

int main(int argc, char** argv) {
	ros::init(argc,argv,"create_mdps");

	ROS_INFO("CREATE_MDP - started node");
	string mdp_name=argv[1];

	ROS_INFO("CREATE_MDP - mdp name is %s",mdp_name.c_str());

	std::vector<std::string> locations={"table","outside","shelf1","shelf2","sofa"};
// 

	std::map<std::string,std::vector<std::string> > connections;
	connections["table"]={"shelf1","shelf2","outside","sofa"};
	connections["outside"]={"table"};
	connections["shelf1"]={"table"};
	connections["shelf2"]={"table"};
	connections["sofa"]={"table"};	

	if (mdp_name=="drink_water") {
		DrinkSomething drink_water("agent","mug","waterbottle",locations,connections);
		drink_water.create("drink_water",true);
	}
	else if (mdp_name=="drink_tea") {
		DrinkSomething drink_tea("agent","mug","teabottle",locations,connections);
		drink_tea.create("drink_tea",true);
	}
	else if (mdp_name=="read_book") {
		ReadBook read_book("agent","sofa",locations,connections);
		read_book.create("read_book",true);
	}	
	else if (mdp_name=="go_out") {
		GoOut go_out("agent",locations,connections);
		go_out.create("go_out",true);
	}	
	else if (mdp_name=="watch_tv") {
		WatchTv watch_tv("agent",locations,connections);
		watch_tv.create("watch_tv",true);
	}	
	else if (mdp_name=="clean_room") {
		CleanRoom clean_room("agent",locations);
		clean_room.create("clean_room",true,true);
		// CleanObject clean_object("agent",locations);
		// clean_object.create("clean_object",true,false);
	}	
	else if (mdp_name=="clean_books") {
		CleanBooks clean_books("agent","shelf2",locations,connections);
		clean_books.create("clean_books",true);
		// CleanObject clean_object("agent",locations);
		// clean_object.create("clean_object",true,false);
	}

}