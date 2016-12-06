#include <simple_agent_monitor/agent_monitors.h>

AgentMonitors::AgentMonitors(string robot_name):robot_name_(robot_name) {

}

vector<situation_assessment_msgs::Fact> AgentMonitors::calculateIsFacing(EntityMap map1, EntityMap map2) {
	vector<situation_assessment_msgs::Fact> result;

	for (EntityMap::iterator entity1=map1.begin();entity1!=map1.end();entity1++) {
		geometry_msgs::Pose pose1,pose2;
		pose1=entity1->second.pose.getSequence(1)[0];
		vector<string> facing_values;
		for (EntityMap::iterator entity2=map2.begin(); entity2!=map2.end();entity2++) {
			if (entity2->first!=entity1->first) {
				pose2=entity2->second.pose.getSequence(1)[0];
				bool value=(isFacing(pose1,pose2.position)>0.0);
				if (value) {
					facing_values.push_back(entity2->first);
				}
			}
		}
		if (facing_values.size()>0) {
			situation_assessment_msgs::Fact f;
			f.model=robot_name_;
			f.subject=entity1->first;
			f.predicate.push_back("isFacing");
			f.value=facing_values;
			result.push_back(f);
		}
	}
	return result;
}
vector<situation_assessment_msgs::Fact> AgentMonitors::getGroupContains(StringVectorMap map) {
	vector<situation_assessment_msgs::Fact> result;
	for (StringVectorMap::iterator group_members=map.begin();group_members!=map.end();group_members++) {
		situation_assessment_msgs::Fact f;
		f.model=robot_name_;
		f.subject=group_members->first;
		f.predicate.push_back("contains");
		f.value=group_members->second;
		result.push_back(f);
	}
	return result;
} 

vector<situation_assessment_msgs::Fact> AgentMonitors::getDistances(EntityMap map1, EntityMap map2,
	PairMap* entity_distances) {
	vector<situation_assessment_msgs::Fact> result;

	for (EntityMap::iterator entity1=map1.begin(); entity1!=map1.end();entity1++) {
		geometry_msgs::Pose pose1,pose2;
		pose1=entity1->second.pose.getSequence(1)[0];
		for (EntityMap::iterator entity2=map2.begin(); entity2!=map2.end();entity2++) {

			if (entity2->first!=entity1->first) {
				pose2=entity2->second.pose.getSequence(1)[0];
				double d=calculateDistance(pose1.position,pose2.position);
				pair<string,string> distanceInput(entity1->first,entity2->first);
				if ((*entity_distances)[distanceInput].isEmpty()) {
					(*entity_distances)[distanceInput].allocate(10);
				}
				(*entity_distances)[distanceInput].insert(d);
				situation_assessment_msgs::Fact f;
				f.model=robot_name_;
				f.subject=entity1->first;
				f.predicate.push_back("distance");
				f.predicate.push_back(entity2->first);
				f.value.push_back(boost::lexical_cast<string>(d));
				result.push_back(f);
				// cout<<"Distance "<<entity1->first<<" "<<entity2->first<<" "<<d<<"\n";
			}
		}
	}
	return result;
}
vector<situation_assessment_msgs::Fact> AgentMonitors::getIsMoving(EntityMap map) {

	vector<situation_assessment_msgs::Fact> result;

	for (EntityMap::iterator entity=map.begin();entity!=map.end();entity++) {
		vector<geometry_msgs::Pose> poses=entity->second.pose.getSequence(5);
		vector<geometry_msgs::Point> points;
		BOOST_FOREACH(geometry_msgs::Pose p, poses) {
			points.push_back(p.position);
		}
		bool value=isMoving(points);

		situation_assessment_msgs::Fact f;
		f.model=robot_name_;
		f.subject=entity->first;
		f.predicate.push_back("isMoving");
		f.value.push_back(boost::lexical_cast<string>(value));
		result.push_back(f);

	}
	return result;
}

vector<situation_assessment_msgs::Fact> AgentMonitors::getDeltaDistances(EntityMap map1, EntityMap map2,
	PairMap entity_distances) {
		vector<situation_assessment_msgs::Fact> result;

		for (EntityMap::iterator entity1=map1.begin();entity1!=map1.end();entity1++) {
			geometry_msgs::Pose pose1,pose2;
			pose1=entity1->second.pose.getSequence(1)[0];
			for (EntityMap::iterator entity2=map2.begin();entity2!=map2.end();entity2++) {
				if (entity2->first!=entity1->first) {
					pose2=entity2->second.pose.getSequence(1)[0];

					pair<string,string> distanceInput(entity1->first,entity2->first);
					vector<double> distances=entity_distances[distanceInput].getSequence(10);

					double delta_distance=distances[9]-distances[0];
					situation_assessment_msgs::Fact f;
					f.model=robot_name_;
					f.subject=entity1->first;
					f.predicate.push_back("deltaDistance");
					f.predicate.push_back(entity2->first);
					f.value.push_back(boost::lexical_cast<string>(delta_distance));
					result.push_back(f);
				}
			}
		}
	return result;
}
vector<situation_assessment_msgs::Fact> AgentMonitors::getIsInArea(EntityMap map, PolygonMap areas) {
		vector<situation_assessment_msgs::Fact> result;

		for (EntityMap::iterator entity=map.begin();entity!=map.end();entity++) {
			geometry_msgs::Pose pose=entity->second.pose.getSequence(1)[0];

			//boost::tuple<double, double> p = boost::make_tuple(pose.position.x, pose.position.y);
			// bg::model::d2::point_xy<double>  p(pose.position.x,pose.position.y);
			vector<string> value_areas;
			Point p=gtl::construct<Point>(pose.position.x, pose.position.y);
			for (PolygonMap::iterator an_area=areas.begin();an_area!=areas.end();an_area++) {
				if(gtl::contains(an_area->second, p )){
					value_areas.push_back(an_area->first);
				}
			}
			value_areas.push_back("this");
			if (value_areas.size()>0) {
				situation_assessment_msgs::Fact new_fact;
				new_fact.model=robot_name_;
				new_fact.subject=entity->first;
				new_fact.predicate.push_back("isInArea");
				new_fact.value=value_areas;
				result.push_back(new_fact);
			}		
		}
		return result;

} 

std::vector<situation_assessment_msgs::Fact> AgentMonitors::getAt(
	std::map<std::string,int> depth_areas, std::vector<situation_assessment_msgs::Fact> is_in_area_facts) {
	
	std::vector<situation_assessment_msgs::Fact> result;

	for (int i=0;i<is_in_area_facts.size();i++) {
		string agent=is_in_area_facts[i].subject;
		std::vector<std::string> areas=is_in_area_facts[i].value;

		int max_depth=-1;
		string location="";
		for (string a:areas) {
			if (depth_areas.at(a)>max_depth) {
				max_depth=depth_areas.at(a);
				location=a;
			}
		}
		situation_assessment_msgs::Fact f;
		f.model=robot_name_;
		f.subject=agent;
		f.predicate={"isAt"};
		f.value={location};
		result.push_back(f);
	}
	return result;

}



vector<situation_assessment_msgs::Fact> AgentMonitors::getEntityType(EntityMap map) {
	vector<situation_assessment_msgs::Fact> result;
	for (EntityMap::iterator entity=map.begin(); entity!= map.end(); entity++) {
		situation_assessment_msgs::Fact new_fact;
		new_fact.model=robot_name_;
		new_fact.subject=entity->second.name;
		new_fact.predicate.push_back("type");
		new_fact.value.push_back(entity->second.category);
		new_fact.value.push_back(entity->second.type);
		result.push_back(new_fact);
	}
	situation_assessment_msgs::Fact new_fact,new_fact_2;
	new_fact.model=robot_name_;
	new_fact.subject="this";
	new_fact.predicate.push_back("type");
	new_fact.value.push_back("location");
	new_fact.value.push_back("location that includes everyone");
	result.push_back(new_fact);	

	new_fact_2.model=robot_name_;
	new_fact_2.subject=robot_name_;
	new_fact_2.predicate.push_back("type");
	new_fact_2.value.push_back("agent");
	new_fact_2.value.push_back("robot");
	result.push_back(new_fact_2);	

	return result;
}

vector<situation_assessment_msgs::Fact> AgentMonitors::getHasArea(vector<string> area_names) {
	vector<situation_assessment_msgs::Fact> result;
	for (int i=0; i<area_names.size();i++) {
		situation_assessment_msgs::Fact new_fact;

		new_fact.model=robot_name_;
		new_fact.subject=area_names[i];
		new_fact.predicate.push_back("hasArea");
		new_fact.value.push_back("true");
		result.push_back(new_fact);
	}
	return result;
	
}

vector<situation_assessment_msgs::Fact> AgentMonitors::getEntityPoses(EntityMap map) {
	vector<situation_assessment_msgs::Fact> result;
	for (EntityMap::iterator it=map.begin(); it!=map.end();it++) {
		situation_assessment_msgs::Fact new_fact;

		new_fact.model=robot_name_;
		new_fact.subject=it->second.name;
		new_fact.predicate.push_back("pose");
		geometry_msgs::Pose pose=it->second.pose.getSequence(1)[0];
		new_fact.value.push_back(boost::lexical_cast<string>(pose.position.x));
		new_fact.value.push_back(boost::lexical_cast<string>(pose.position.y));
		new_fact.value.push_back(boost::lexical_cast<string>(pose.position.z));
		new_fact.value.push_back(boost::lexical_cast<string>(pose.orientation.x));
		new_fact.value.push_back(boost::lexical_cast<string>(pose.orientation.y));
		new_fact.value.push_back(boost::lexical_cast<string>(pose.orientation.z));
		new_fact.value.push_back(boost::lexical_cast<string>(pose.orientation.w));

		result.push_back(new_fact);
	}
	return result;
}
