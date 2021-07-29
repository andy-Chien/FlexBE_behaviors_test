#!/usr/bin/env python

from flexbe_core import EventState, Logger
from math import radians
import random
import time
# import transformations as tf

pose_1 = [[-0.421074158122, 
		  0.239718910582, 
		   0.361356381303], [[0.0, 0.0,  1.0], 
							 [0.0, 1.0, 0.0], 
							 [-1.0, 0.0, 0.0]]]
pose_2 = [[-0.42048904877493978, 
		  -0.247408276622308135, 
		   0.3229090515842802], [[0.0, 0.0,  1.0], 
								 [0.0, 1.0, 0.0], 
								 [-1.0, 0.0, 0.0]]]
pose_3 = [[-0.42048904877493978, 
		  0.39408276622308135, 
		   0.2829090515842802], [[0.0, 0.0,  1.0], 
								 [0.0, 1.0, 0.0], 
								 [-1.0, 0.0, 0.0]]]
pose_4 = [[-0.42048904877493978,
 		  -0.347408276622308135, 
 		   0.3229090515842802], [[0.0, 0.0,  1.0], 
								 [0.0, 1.0, 0.0], 
								 [-1.0, 0.0, 0.0]]]
pose_5 = [[-0.42048904877493978,
 		  -0.087408276622308135, 
 		   0.2429090515842802], [[0.0, 0.0,  1.0], 
								 [0.0, 1.0, 0.0], 
								 [-1.0, 0.0, 0.0]]]
pose = [pose_1, pose_2, pose_3, pose_4, pose_5]

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
		p = random.randint(0,4)
		if pose_1 is not None:
			userdata.tar_trans = pose[p][0]
			userdata.tar_rot = pose[p][1]
			return 'done'
		else:
			return 'fail'
	
	def on_enter(self, userdata):
		self.index += 1