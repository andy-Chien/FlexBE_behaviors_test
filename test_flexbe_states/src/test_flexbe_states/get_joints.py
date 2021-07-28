#!/usr/bin/env python

from flexbe_core import EventState, Logger
from math import radians
import random
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

pose_7 = [radians(-164),
          -1.5296394975453613,
          1.8446046006373687,
          -1.5524816186736439,
          -1.3525832291431448,
          0]

pose_8 = [-0.761497594000985,
          -0.9237441220666166,
          1.1992470626830014,
          -1.8635214638400424,
          -2.0507101389103815,
          2.9725204703584715e-05]

pose_5 = [  -3.121793025962227,
            -2.295012056287639,
            1.8813583340468238,
            -1.227925987886334,
            -1.678778616515387,
            8.26826297048188e-06]
pose_6 = [  -3.186723267606602,
            -2.2944543721383375,
            1.0642376792200423,
            -1.6311959993542955,
            -2.0023213886298015,
            2.925580068927249e-05]

pose_3 = [radians(-71), radians(45), radians(-98), radians(-51), radians(-42), radians(0)]
pose_4 = [radians(79), radians(36), radians(-70), radians(30), radians(-93), radians(0)]

pose = [pose_1, pose_2, pose_7, pose_5, pose_6]

class GetJointState(EventState):
	"""
	Publishes a pose from userdata so that it can be displayed in rviz.

	-- robot_id                int              Robots id to move
	-- plan_mode               string           plan mode

	#> robot_id                int             Robots id to move
	#> joint_values             float[]		   Target joint values of robot
	#> plan_mode                string          plan_only or offline or online
	#> start_config            float[]         Start joint values of robot

	<= done										Pose has been published.
	<= finish									Task finished

	"""
	
	def __init__(self, robot_id, plan_mode):
		"""Constructor"""
		super(GetJointState, self).__init__(outcomes=['done', 'finish'], output_keys=['robot_id', 'joint_values', 'plan_mode', 'start_config'])
		# self.joint_values_input = eval(joint_values_input) if type(joint_values_input) == str else joint_values_input
		self.joint_values_input = None
		# self.robot_ids = eval(robot_ids) if type(robot_ids) == str else robot_ids
		self.robot_id = robot_id
		self.plan_mode = plan_mode
		self.pose_indx = 0

	def execute(self, userdata):
		plan_mode = ['offline', 'offline', 'offline']
		userdata.plan_mode = self.plan_mode # plan_mode[self.pose_indx % 3]
		userdata.robot_id = self.robot_id
		userdata.joint_values = self.joint_values
		userdata.start_config = None
		if self.pose_indx == 7:
			return 'finish'
		else:
			return 'done'
	
	def on_enter(self, userdata):
		if self.joint_values_input is not None:
			try:
				self.joint_values = [radians(float(x)) for x in self.joint_values_input]
				# assert len(self.joint_values) == len(self.robot_id)
				self.pose_indx = 7
				print('pose ', self.pose_indx, ' : ', self.joint_values, ' go!')
			except Exception as e:
				Logger.logwarn('Failed to get pose in param!\n%s' % str(e))
		else:
			try:
				p = random.randint(0,4)
				print('pppppppppppppppppppppppppppppppppppp = ', p)
				self.joint_values = (pose[p])
				self.pose_indx += 1
				print('pose ', self.pose_indx, ' go!')
			except Exception as e:
				Logger.logwarn('Failed to get pose in defalt!\n%s' % str(e))
