#!/usr/bin/env python
"""
Simple script to follow the human's head

__author__=Michelangelo Fiore
"""


import sys
import rospy
import pr2_controllers_msgs.msg
import geometry_msgs.msg
import actionlib

def lookAt(frame_id, x, y, z):
	"""
	Moves the robot's head to look at a specific frame
	:param frame_id: name of the frame to look at
	:param x: x coordinate in the frame_id (usually 0)
	:param y: y coordinate in the frame_id (usually 0)
	:param z: z coordinate in the frame_id (usually 0)
	:return: returns nothing
	"""
	point_stamped=geometry_msgs.msg.PointStamped()
	point_stamped.header.frame_id=frame_id
	point_stamped.point.x=x;
	point_stamped.point.y=y;
	point_stamped.point.z=z;

	goal=pr2_controllers_msgs.msg.PointHeadGoal()
	goal.target=point_stamped
	goal.pointing_frame="head_plate_frame"
	goal.pointing_axis.x=1
	goal.pointing_axis.y=0
	goal.pointing_axis.z=0
	goal.min_duration = rospy.Duration(0.5)
	goal.max_velocity = 1.0

	point_head_client = actionlib.SimpleActionClient('/head_traj_controller/point_head_action',pr2_controllers_msgs.msg.PointHeadAction)

	point_head_client.wait_for_server()
	point_head_client.send_goal(goal)
	point_head_client.wait_for_result()
	res=point_head_client.get_result()

if __name__ == '__main__':
	rospy.init_node("head_follower")
	rospy.loginfo("HEAD_FOLLOWER started node")
	human_name=rospy.get_param("/handover/human_name")
	rospy.loginfo("HEAD_FOLLOWER human name is %s",human_name)
	r=rospy.Rate(3)
	while not rospy.is_shutdown():
		lookAt(human_name+"_head",0,0,0)
		r.sleep()






