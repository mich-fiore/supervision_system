/**
 * Bridge that takes data from TF and makes it available to the situation assessment layer
 * @author= Michelangelo Fiore
 */

#ifndef TF_BRIDGE_H
#define TF_BRIDGE_H

#include <ros/ros.h>
#include <data_lib/data_lib.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <vector>
#include <map>

#include <situation_assessment_msgs/NamedPoseList.h>
#include <situation_assessment_msgs/NamedPose.h>
#include <situation_assessment_msgs/Group.h>
#include <situation_assessment_msgs/GroupList.h>
#include <situation_assessment_msgs/QueryDatabase.h>

#include <geometry_msgs/Pose.h>

using namespace std;


class TfBridge:public DataLib {
    /**
     * Bridge that takes data from TF and makes it available to the situation assessment layer
     */
public:
    /**
     * Class constructor
     * @param node_handle handle of this node
     */
	TfBridge(ros::NodeHandle node_handle);
    /**
     * Get the poses of entities from tf
     */
	void getPoses();
private:
    /**
     * converts a tf transform to a geometry msg 
     * @param  transform tf transform
     * @return           converted msg
     */
	geometry_msgs::Pose tfToGeometry(tf::StampedTransform transform);

	tf::TransformListener listener_;


};

#endif
