#!/usr/bin/env python

from flexbe_core import EventState, Logger
from math import radians
import random
import time

MAX_STEP = 10

pose_1 = [radians(88.5), radians(-131), radians(-56.5), radians(-80.4), radians(88.8), radians(0)]
pose_2 = [radians(-28.8), radians(-131), radians(-56.5), radians(-80.4), radians(88.8), radians(0)]

pose = [pose_1, pose_2]

class GetJointSymple(EventState):
	"""
	Publishes a pose from userdata so that it can be displayed in rviz.

	#> joint_config             float[]		   Target joint values of robot
	
	<= done										Pose has been published.
	<= finish									Task finished

	"""
	
	def __init__(self):
		"""Constructor"""
		super(GetJointSymple, self).__init__(outcomes=['done', 'finish'], output_keys=['joint_config'])
		self.pose_indx = 0

	def execute(self, userdata):
		if self.pose_indx == MAX_STEP:
			return 'finish'
		else:
			return 'done'
	
	def on_enter(self, userdata):
		try:
			userdata.joint_config = (pose[self.pose_indx % 2])
			self.pose_indx += 1
		except Exception as e:
			Logger.logwarn('Failed to get pose in defalt!\n%s' % str(e))
