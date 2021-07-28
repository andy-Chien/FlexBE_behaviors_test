#!/usr/bin/env python

from flexbe_core import EventState, Logger
from math import radians
import random
import time
import transformations as tf

pose_1 = [[-0.361074158122, 
		  0.139718910582, 
		   0.661356381303], [[0.0, 0.0,  1.0], 
							 [0.0, 1.0, 0.0], 
							 [-1.0, 0.0, 0.0]]]
pose_2 = [[0.29048904877493978, 
		  -0.147408276622308135, 
		   0.4229090515842802], [[0.0, 0.0,  1.0], 
								 [0.0, 1.0, 0.0], 
								 [-1.0, 0.0, 0.0]]]
pose_3 = [[0.37048904877493978, 
		  -0.09408276622308135, 
		   0.5229090515842802], [[0.0, 0.0,  1.0], 
								 [0.0, 1.0, 0.0], 
								 [-1.0, 0.0, 0.0]]]
pose_4 = [[0.29048904877493978,
 		  -0.147408276622308135, 
 		   0.4229090515842802], [[0.0, 0.0,  1.0], 
								 [0.0, 1.0, 0.0], 
								 [-1.0, 0.0, 0.0]]]
pose = [pose_1, pose_2, pose_3, pose_4]

class GetPoseState(EventState):
	"""
	Publishes a pose from userdata so that it can be displayed in rviz.


	#> tar_trans                 float[]		   Target pose values of robot
	#> tar_rot                 float[]		   Target pose values of robot

	<= done									   Pose has been published.
	<= fail									   Task fail and finished

	"""
	
	def __init__(self):
		"""Constructor"""
		super(GetPoseState, self).__init__(outcomes=['done', 'fail'], output_keys=['tar_trans', 'tar_rot'])
		self.index = 0

	def execute(self, userdata):
		if pose_1 is not None:
			userdata.tar_trans = pose[self.index % 4][0]
			userdata.tar_rot = pose[self.index % 4][1]
			return 'done'
		else:
			return 'fail'
	
	def on_enter(self, userdata):
		self.index += 1