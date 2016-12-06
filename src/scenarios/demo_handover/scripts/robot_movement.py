#!/usr/bin/env python
"""
move base movements for the demo_handover (not tested)
__author__=Michelangelo Fiore
"""
import sys
import rospy
import geometry_msgs.msg
import move_base_msgs.msg
import actionlib

class RobotMovement:
	"""
	move base movements for the demo_handover (not tested)
	"""
	def __init__(self):
		"""
		Constructor for the RobotMovement class
		:return: returns nothing
		"""
		self.client_move_base = actionlib.SimpleActionClient('/move_base', move_base_msgs.msg.MoveBaseGoal)

		self.client_move_base.wait_for_server()

		rospy.loginfo("RobotMovement connected to move base")

		self.location_list=rospy.get_param("handover/location_list")
		for location in location_list:
			self.locations[location]=rospy.get_param("handover/"+location)


	def moveTo(self,l):
		"""
		Moves the robot to a location
		:param l: location to move the robot to (should be present in the handover/location_list parameter)
		:return: returns nothing
		"""
		if l in self.location_list:
			point=self.locations[l]

			move_base_goal=move_base_msgs.msg.MoveBaseGoal()
			move_base_goal.target_pose.header.frame_id = "map"
			move_base_goal.target_pose.header.stamp = rospy.Time.now()

			geometry_msgs.msg.Pose
			pose.position.x=l[0]
			pose.position.y=l[1]
			pose.orientation.w=1
			move_base_goal.target_pose.pose=pose

			self.client_move_base.send_goal(move_base_goal)
			self.client_move_base.wait_for_result()
			