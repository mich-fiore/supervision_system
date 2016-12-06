/**
 * Node that sets the Demo Observer scenario
 * @author Michelangelo Fiore
 */
#include <ros/ros.h>
#include "demo_observer/actions/drink.h"
#include "demo_observer/actions/fill.h"
#include "situation_assessment_msgs/AddArea.h"


ros::ServiceClient add_area_client_;

/**
 * Adds an Area in the environment
 * @param name   name of the area
 * @param points polygon of the area
 */
void addArea(string name, std::vector<geometry_msgs::Point32> points) {
	situation_assessment_msgs::AddArea s;
	s.request.name=name;
	s.request.area.points=points;
	s.request.linked_to_entity="";

	if (add_area_client_.call(s)) {
		ROS_INFO("DEMO_SCENARIO - added area for %s",name.c_str());
	}
	else {
		ROS_WARN("DEMO_SCENARIO - error contacting add area service");
	}

}
/**
 * Utility function to create a geometry_msgs::Point from coordinates
 * @param  x x of the point
 * @param  y y of the point
 * @return   the geometry_msgs::Point object
 */
geometry_msgs::Point32 createPoint(double x, double y) {
	geometry_msgs::Point32 p;
	p.x=x;
	p.y=y;
	p.z=0;
	return p;
}

int main(int argc,char **argv) {
	ros::init(argc,argv,"demo_scenario");
	ROS_INFO("DEMO_OBSERVER - starting node");

	ros::NodeHandle node_handle;

	Drink drink(node_handle);
	Fill fill(node_handle);
	// Move move(node_handle);
	// R
	ROS_INFO("DEMO_OBSERVER created actions");

	add_area_client_=node_handle.serviceClient<situation_assessment_msgs::AddArea>("situation_assessment/add_area",1000);

	ROS_INFO("DEMO_OBSERVER - Connecting to add area");
	add_area_client_.waitForExistence();
	ROS_INFO("DEMO_OBSERVER - connected");

     vector<geometry_msgs::Point32> points_table,points_shelf1,points_shelf2,points_sofa,
     points_outside;


     points_shelf1={createPoint(0.2,1.85),createPoint(-1,1.85),
     	createPoint(-1,2.86),createPoint(0.2,2.86)};

   	points_shelf2={createPoint(1.2,2.86),createPoint(0.2,2.86),
     		createPoint(0.2,3.68),createPoint(1.2,3.68)};

     points_table={createPoint(0.2,2.86),createPoint(-2.52,2.86),
     	createPoint(-2.52,5.49),createPoint(0.2,5.49)};
     points_outside={createPoint(1.2,5.49),createPoint(-0.76,5.49),
     	createPoint(-0.76,6.5),createPoint(1.2,6.5)};
     points_sofa={createPoint(-2.52,2.86),createPoint(-4,2.86),
     	createPoint(-4,5.49),createPoint(-2.52,5.49)};

     addArea("shelf1",points_shelf1);
     addArea("shelf2",points_shelf2);
     addArea("table",points_table);
     addArea("outside",points_outside);
     addArea("sofa",points_sofa);

	ros::spin();
}