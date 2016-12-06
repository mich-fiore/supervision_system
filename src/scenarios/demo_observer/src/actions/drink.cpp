#include "demo_observer/actions/drink.h"


Drink::Drink(ros::NodeHandle node_handle):Action("drink",node_handle) {
	parameters_.push_back("main_agent");
	parameters_.push_back("main_object");
}

bool Drink::checkPreconditions(StringMap parameters) {
	ROS_INFO("%s - checking preconditions",action_name_.c_str());

	if (!checkParameterPresence(parameters)) return false;

	

	situation_assessment_msgs::Fact has_query;
	has_query.model=robot_name_;
	has_query.subject=parameters.at("main_agent");
	has_query.predicate.push_back("has");
	has_query.predicate.push_back(parameters.at("main_object"));

	bool has_object=queryDatabase(has_query)=="true";

	if (!has_object) return false;

	situation_assessment_msgs::Fact capacity_query;
	capacity_query.model=robot_name_;
	capacity_query.subject=parameters.at("main_object");
	capacity_query.predicate.push_back("capacity");

	int capacity=std::stoi(queryDatabase(capacity_query));

	return capacity==1;

}
void Drink::setPostconditions(StringMap parameters) {
	string object=parameters.at("object");

	situation_assessment_msgs::Fact object_capacity;
	object_capacity.model=robot_name_;
	object_capacity.subject=object;
	object_capacity.predicate.push_back("capacity");
	object_capacity.value.push_back("0");

	std::vector<situation_assessment_msgs::Fact> fact_vector={object_capacity};
	setFacts(fact_vector);
}
