#!/usr/bin/env python

from flexbe_core import EventState, Logger
from math import radians
import time

pose_1 = [-2.3759277711705376,
          -1.5296394975453613,
          1.8446046006373687,
          -1.5524816186736439,
          -1.3525832291431448,
          -9.982456039710583e-06]

pose_2 = [-4.761497594000985,
          -0.9237441220666166,
          1.1992470626830014,
          -1.8635214638400424,
          -2.0507101389103815,
          2.9725204703584715e-05]

pose = [pose_1, pose_2]

class GetPoseState(EventState):
	"""
	Publishes a pose from userdata so that it can be displayed in rviz.

	-- joint_values_string 		string 			Index of point to move

	#> joint_values                             Joint angles to move

	<= done										Pose has been published.
	<= finish									Task finished

	"""
	
	def __init__(self, joint_values_string):
		"""Constructor"""
		super(GetPoseState, self).__init__(outcomes=['done', 'finish'], output_keys=['joint_values'])
		self._joint_values_string = joint_values_string
		self.joint_values = []
		self.pose_indx = 0

	def execute(self, userdata):
		userdata.joint_values = self.joint_values
		time.sleep(0.5)
		if self.pose_indx == 5:
			return 'finish'
		else:
			return 'done'
	
	def on_enter(self, userdata):
		if self._joint_values_string is not None and self._joint_values_string != '0':
			try:
				self.joint_values = [radians(float(x)) for x in self._joint_values_string.split(',')]
				self.pose_indx += 1
				print('pose ', self.pose_indx, ' : ', self.joint_values, ' go!')
			except Exception as e:
				Logger.logwarn('Failed to get pose!\n%s' % str(e))
		else:
			try:
				# self.joint_values = [radians(float(x)) for x in self._joint_values_string.split(',')]
				self.joint_values = pose[self.pose_indx % 2]
				self.pose_indx += 1
				print('pose ', self.pose_indx, ' go!')
			except Exception as e:
				Logger.logwarn('Failed to get pose!\n%s' % str(e))
