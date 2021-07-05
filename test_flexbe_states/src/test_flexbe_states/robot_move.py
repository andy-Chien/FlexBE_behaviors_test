#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient

from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, Constraints, JointConstraint, MoveItErrorCodes
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction, JointTrajectoryControllerState, FollowJointTrajectoryResult

'''
Created on 15.06.2015

@author: Philipp Schillinger
'''

class RobotMoveState(EventState):
	'''
	Uses MoveIt to plan and move the specified joints to the target configuration.

	-- robot_topic 	string 		Topic on which MoveIt is listening for action calls.

	># joint_trajectory JointTrajectory  planned trajectory

	<= done 						Robot move done.
	<= failed 						Robot move failed.
	'''


	def __init__(self, robot_topic):
		'''
		Constructor
		'''
		super(RobotMoveState, self).__init__(outcomes=['done', 'failed'],
											input_keys=['joint_trajectory'])
		self._robot_topic = robot_topic
		self._client = ProxyActionClient({self._robot_topic: FollowJointTrajectoryAction})
		self._goal = FollowJointTrajectoryGoal()
		self._goal_time_tolerance = rospy.Time(0.1)
		self._goal.goal_time_tolerance = self._goal_time_tolerance
		self._goal = FollowJointTrajectoryGoal()
		self._goal.goal_time_tolerance = rospy.Time(0.1)
		self._success = False
		self._failed = False	    

	def stop(self):
		self._client.cancel_goal()

	def execute(self, userdata):
		'''
		Execute this state
		'''
		if self._success:
			return 'done'
		elif self._failed:
			return 'failed'

		if self._client.has_result(self._robot_topic):
			result = self._client.get_result(self._robot_topic)

			if result.error_code == FollowJointTrajectoryResult.SUCCESSFUL:
				self._success = True
				return 'done'
			else:
				self._failed = False
				return 'failed'


	def on_enter(self, userdata):
		self._success = False
		self._failed = False

		self._goal = FollowJointTrajectoryGoal()
		self._goal.goal_time_tolerance = rospy.Time(0.1)
		self._goal.trajectory = userdata.joint_trajectory
		self._goal.trajectory.header.stamp = rospy.Time.now()
		try:
			self._client.send_goal(self._robot_topic, self._goal)
		except Exception as e:
			Logger.logwarn('Failed to send action goal: %s\n%s' % (self._robot_topic, str(e)))
			self._failed = True

	def on_stop(self):
		try:
			if self._client.is_available(self._robot_topic) \
			and not self._client.has_result(self._robot_topic):
				self._client.cancel(self._robot_topic)
		except:
			# client already closed
			pass

	def on_pause(self):
		self._client.cancel(self._robot_topic)

	def on_resume(self, userdata):
		self.on_enter(userdata)
