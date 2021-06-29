#!/usr/bin/env python

from flexbe_core import EventState, Logger
import rospy
import time
from flexbe_core.proxy import ProxyPublisher
from test_flexbe_behaviors.msg import PlanCmd

class SendCmdState(EventState):
	"""
	Publishes a pose from userdata so that it can be displayed in rviz.

	-- topic 		string 			Topic to which the pose will be published.
	-- robot_id		int 			ID of the robot.

	># joint_values	float[]			Pose to be published.

	<= done							Pose has been published.

	"""
	
	def __init__(self, topic, robot_id):
		"""Constructor"""
		super(SendCmdState, self).__init__(outcomes=['done'],
												input_keys=['joint_values'])
		self.pose_indx = 0
		self._topic = topic
		self._id = robot_id
		self._pub = ProxyPublisher({self._topic: PlanCmd})

	def execute(self, userdata):
		time.sleep(0.5)
		return 'done'

	def on_enter(self, userdata):
		msg = PlanCmd()
		msg.id = self._id
		msg.tar_joint = userdata.joint_values
		self._pub.publish(self._topic, msg)
