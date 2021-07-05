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

	-- robot_ids                string[]        Robots id to move
	-- joint_values_input 		string[] 		Value of points to move

	#> robot_ids                int[]           Robots id to move
	#> joint_values             float[]			joint values of robots, 2 dim list
	#> plan_mode                string          plan_only or offline or online

	<= done										Pose has been published.
	<= finish									Task finished

	"""
	
	def __init__(self, robot_ids, joint_values_input):
		"""Constructor"""
		super(GetPoseState, self).__init__(outcomes=['done', 'finish'], output_keys=['robot_ids', 'joint_values', 'plan_mode'])
		self.joint_values_input = eval(joint_values_input) if type(joint_values_input) == str else joint_values_input
		self.robot_ids = eval(robot_ids) if type(robot_ids) == str else robot_ids
		self.plan_mode = 'plan_only'
		self.pose_indx = 0

	def execute(self, userdata):
		plan_mode = ['plan_only', 'offline', 'online']
		userdata.plan_mode = plan_mode[self.pose_indx % 3]
		userdata.robot_ids = self.robot_ids
		userdata.joint_values = self.joint_values
		time.sleep(0.5)
		if self.pose_indx == 7:
			return 'finish'
		else:
			return 'done'
	
	def on_enter(self, userdata):
		if self.joint_values_input is not None:
			try:
				self.joint_values = [[radians(float(x)) for x in joint_values] for joint_values in self.joint_values_input]
				assert len(self.joint_values) == len(self.robot_ids)
				self.pose_indx = 5
				print('pose ', self.pose_indx, ' : ', self.joint_values, ' go!')
			except Exception as e:
				Logger.logwarn('Failed to get pose in param!\n%s' % str(e))
		else:
			try:
				self.joint_values = []
				self.joint_values.append(pose[self.pose_indx % 2])
				self.pose_indx += 1
				print('pose ', self.pose_indx, ' go!')
			except Exception as e:
				Logger.logwarn('Failed to get pose in defalt!\n%s' % str(e))
