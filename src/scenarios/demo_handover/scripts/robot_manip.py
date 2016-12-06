#!/usr/bin/env python
"""
implements arm and gripper actions needed for the handover

__author__=Michelangelo Fiore
"""

import sys
import rospy
import actionlib
import pr2motion.msg
import threading
import time
import pr2_controllers_msgs.msg
import trajectory_msgs.msg
import pr2_gripper_sensor_msgs.msg

class RobotManip:
	"""
	Implements arm and gripper actions needed for the handover
	"""
	def __init__(self):
		"""
		Constructor of the RobotManip class
		:return: returns nothing
		"""
		self.client_arm = actionlib.SimpleActionClient('/pr2motion/Arm_Right_MoveToQGoal', pr2motion.msg.Arm_Right_MoveToQGoalAction)

		self.poses_list=rospy.get_param("handover/poses_list")
		self.poses=dict()
		for pose in self.poses_list:
			self.poses[pose]=rospy.get_param("handover/"+pose)

		self.is_waiting_pressure=False
		self.has_released_gripper=False
		self.waiting_lock=threading.Lock()
		self.released_gripper_lock=threading.Lock()

		rospy.loginfo("ROBOT_MANIP Waiting for arm server")
		self.traj_client = actionlib.SimpleActionClient("r_arm_controller/joint_trajectory_action", pr2_controllers_msgs.msg.JointTrajectoryAction);
		self.traj_client.wait_for_server()

		rospy.loginfo("ROBOT_MANIP Waiting for gripper server")
		self.release_client=actionlib.SimpleActionClient("r_gripper_sensor_controller/release",
		 	pr2_gripper_sensor_msgs.msg.PR2GripperReleaseAction)
		self.release_client.wait_for_server()

		self.grab_client =actionlib.SimpleActionClient("r_gripper_sensor_controller/grab",
			pr2_gripper_sensor_msgs.msg.PR2GripperGrabAction)
		self.grab_client.wait_for_server()

	def moveArm(self,pose):
		"""
		Moves the right arm to a specific pose
		:param pose: the pose to move the arm. This pose needs to be included in self.poses_list (which is read from the the parameter handover/poses_list) 
		:return: returns True if the motion is successful
		"""
		rospy.loginfo("ROBOT_MANIP moving to %s",pose)
		if pose in self.poses_list:
			q_values=self.poses[pose]
			goal=pr2_controllers_msgs.msg.JointTrajectoryGoal()

			goal.trajectory.joint_names.append("r_shoulder_pan_joint");
			goal.trajectory.joint_names.append("r_shoulder_lift_joint");
			goal.trajectory.joint_names.append("r_upper_arm_roll_joint");
			goal.trajectory.joint_names.append("r_elbow_flex_joint");
			goal.trajectory.joint_names.append("r_forearm_roll_joint");
			goal.trajectory.joint_names.append("r_wrist_flex_joint");
			goal.trajectory.joint_names.append("r_wrist_roll_joint");

			point=trajectory_msgs.msg.JointTrajectoryPoint();
			point.positions=[q_values[0],
			q_values[1],
			q_values[2],
			q_values[3],
			q_values[4],
			q_values[5],
			q_values[6]]
			point.velocities=[0,0,0,0,0,0,0]
			point.accelerations=[0,0,0,0,0,0,0]

			point.time_from_start = rospy.Duration(2.0);

			goal.trajectory.points.append(point)

			self.traj_client.send_goal(goal)
			self.traj_client.wait_for_result()
			return True

		else:
			rospy.loginfo("RobotManip unknown pose")
			return False

	def gripperRelease(self):
		"""
		Waits for pressure on the gripper and then releases it
		:return: returns nothing
		"""
		print "releasing gripper"
		self.setReleasedGripper(False)
		self.setWaitingGripper(True)

		place=pr2_gripper_sensor_msgs.msg.PR2GripperReleaseGoal()
		place.command.event.trigger_conditions = place.command.event.FINGER_SIDE_IMPACT_OR_SLIP_OR_ACC
		place.command.event.acceleration_trigger_magnitude = 5.0
		place.command.event.slip_trigger_magnitude = 0.01

		rospy.loginfo("ROBOT_MANIP sending release goal")
		self.release_client.send_goal(place);
		self.release_client.wait_for_result();
		self.setWaitingGripper(False)
		rospy.loginfo("Released")
	
	def grab(self):
		"""
		Waits for pressure on the gripper and then closes it
		:return: returns nothing
		"""
		grip=pr2_gripper_sensor_msgs.msg.PR2GripperGrabGoal()
		grip.command.hardness_gain = 0.03
		self.grab_client.send_goal(grip)
		self.grab_client.wait_for_result()



	def cancelGripperRelease(self):
		"""
		Cancel the gripper grab\release wait
		:return: returns nothing
		"""
		print "not releasing gripper anymore"
		self.client_gripper.cancel_all_goals()

	def isWaitingGripper(self):
		with self.waiting_lock:
			result=self.is_waiting_pressure
			return result

	def setWaitingGripper(self,state):
		with self.waiting_lock:
			self.is_waiting_pressure=state

	def hasReleasedGripper(self):
		with self.released_gripper_lock:
			res=self.has_released_gripper
			return res

	def setReleasedGripper(self,state):
		with self.released_gripper_lock:
			self.has_released_gripper=state
		

