#!/usr/bin/env python
"""
Gets observations for the handover by contacting the database

__author__=Michelangelo Fiore

"""
import situation_assessment_msgs.srv
import situation_assessment_msgs.msg
import sys
import math
import rospy

class ObservationsCollector:
	"""
	Collects observations useful for the handover
	"""

	def __init__(self):
		"""
		Constructor of this class
		:return: returns nothing
		"""
		self.extended_distance_=rospy.get_param("/handover/extended_distance")
		self.human_name_=rospy.get_param("/handover/human_name")
		self.handover_location_=rospy.get_param("/handover/handover_location")
		self.robot_name_=rospy.get_param("/situation_assessment/robot_name")		
		
		rospy.loginfo("ObservationsCollector extended distance is %f",self.extended_distance_)
		rospy.loginfo("ObservationsCollector human name is %s",self.human_name_)
		rospy.loginfo("ObservationsCollector handover_location  is %s",self.handover_location_)
		rospy.loginfo("ObservationsCollector robot_name_ is %s",self.robot_name_)

		rospy.wait_for_service('situation_assessment/query_database')
		self.query_database_=rospy.ServiceProxy('situation_assessment/query_database',situation_assessment_msgs.srv.QueryDatabase)
		rospy.loginfo("ObservationsCollector connected to query database")


	def queryDatabase(self,fact):
		"""
		Generic method to query the database
		:param fact: the fact of this query
		:return: returns the result of the query
		"""
		result=[]
		try:
			database_res=self.query_database_(fact)
			if len(database_res.result)>0:
				result=database_res.result[0].value
		except rospy.ServiceException as e:
			rospy.loginfo('HANDOVER error in contacting database %s',str(e))
		return result

	def isHandoverPose(self):
		"""
		Checks if the human is in the handover pose
		:return: returns true if the human is in the handover pose
		"""
		fact=situation_assessment_msgs.msg.Fact()
		fact.model=str(self.robot_name_)
		fact.subject=str(self.human_name_)+"_hand"
		fact.predicate=["pose"]

		hand_pose=self.queryDatabase(fact)[0:3]

		fact.subject=self.human_name_+"_head"

		body_pose=self.queryDatabase(fact)[0:3]

		distance=math.sqrt(

			(float(hand_pose[0])-float(body_pose[0]))**2 +
			(float(hand_pose[1])-float(body_pose[1]))**2)
			# (float(hand_pose[2])-float(body_pose[2]))**2)
		print distance
		print self.extended_distance_
		return str(distance>self.extended_distance_).lower()

	def isHumanLocation(self):
		"""
		Checks if the human is at the handover location
		:return: returns true if the human is at the handover location
		"""
		fact=situation_assessment_msgs.msg.Fact()
		fact.model=str(self.robot_name_)
		fact.subject=str(self.human_name_)+"_torso"
		fact.predicate=['isAt']

		res=self.queryDatabase(fact)
		if (len(res)>0):
			# rospy.loginfo("ObservationsCollector handover location is %s",res[0])
			return str(res[0]==self.handover_location_).lower()
		else:
			# rospy.loginfo("ObservationsCollector no response for isAt")
			return 'false'

	def isTowardRobot(self):
		"""
		check if the human is oriented toward the robot
		:return: returns true if the human is turned toward the robot
		"""
		fact=situation_assessment_msgs.msg.Fact()
		fact.model=str(self.robot_name_)
		fact.subject=str(self.human_name_)+"_head"
		fact.predicate=["isFacing"]

		res=self.queryDatabase(fact)
		if (len(res)>0):
			return str(res[0]==self.robot_name_).lower()
		else:
			return 'false'
			

	def getFullState(self,task_completed,timer_engage_expired,timer_double_expired,
						touch_timer_expired):
	"""
	returns the state in the appl format, which orders variables alphabetically
	by name and concats their values (ex: truetruefalse).
	:param task_completed: true if the task is over
	:param timer_engage_expired: true if the engage timer has expired
	:param timer_double_expired: true if the timer has expired twice
	:param touch_timer_expired: true if the touch timer has expired
	:return returns a list of two elements containing the system state and observations
	"""
		state=str(task_completed).lower()+str(timer_engage_expired).lower()

		obs=self.isHandoverPose()+self.isHumanLocation()+str(timer_double_expired).lower()+str(touch_timer_expired).lower()+self.isTowardRobot()
		return (state,obs)
