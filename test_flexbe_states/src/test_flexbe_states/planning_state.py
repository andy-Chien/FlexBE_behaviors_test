#!/usr/bin/env python

import enum
import rospy
from actionlib.action_server import ros_timer
from enum import Enum
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, Constraints, JointConstraint, MoveItErrorCodes

'''
Created on 15.06.2015

@author: Philipp Schillinger
'''

class PlanMode(Enum):
    PLAN_ONLY = 'plan_only'
    OFFLINE = 'offline'
    ONLINE = 'online'

class PlanningState(EventState):
	'''
	Uses MoveIt to plan and move the specified joints to the target configuration.

	-- move_group		string		Name of the move group to be used for planning.
									Specified joint names need to exist in the given group.
	-- action_topic 	string 		Topic on which MoveIt is listening for action calls.
	-- robot_id		    int         ID of the robot to plan, first robot's id is 0
	-- plan_mode        string      plan_only or offline or online

	># joint_config		float[]		Target configuration of the joints.
									Same order as their corresponding names in joint_names.
	
	># start_config     float[]     Start configuration of the joints.


	#> joint_trajectory JointTrajectory  planned or executed trajectory

	<= planned 						Target joint configuration has been planned.
	<= reached 						Target joint configuration has been reached.
	<= planning_failed 				Failed to find a plan to the given joint configuration.
	<= control_failed 				Failed to move the arm along the planned trajectory.

	'''


	def __init__(self, move_group, action_topic='/move_group', robot_id=0, plan_mode='plan_only'):
		'''
		Constructor
		'''
		super(PlanningState, self).__init__(outcomes=['reached', 'planning_failed', 'control_failed', 'planned'],
											input_keys=['joint_config'],
											output_keys=['joint_trajectory'])
		
		self._action_topic = action_topic
		
		self._client = ProxyActionClient({self._action_topic: MoveGroupAction})
		self._move_group = move_group
		self._joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
		self._plan_mode = plan_mode
		self.robot_id = robot_id

		self._planning_failed = False
		self._control_failed = False
		self._plan_success = False
		self._excute_success = False

	def execute(self, userdata):
		'''
		Execute this state
		'''
		if self._planning_failed:
			return 'planning_failed'
		if self._control_failed:
			return 'control_failed'
		if self._plan_success:
			return 'planned'
		if self._excute_success:
			return 'reached'

		if self._client.has_result(self._action_topic):
			result = self._client.get_result(self._action_topic)
			
			if result.error_code.val == MoveItErrorCodes.CONTROL_FAILED:
				Logger.logwarn('Control failed for move action of group: %s (error code: %s)' % (self._move_group, str(result.error_code)))
				self._control_failed = True
				return 'control_failed'
			elif result.error_code.val != MoveItErrorCodes.SUCCESS:
				Logger.logwarn('Move action failed with result error code: %s' % str(result.error_code))
				self._planning_failed = True
				return 'planning_failed'
			else:
				if self._plan_mode == PlanMode.PLAN_ONLY.value:
					self._plan_success = True
					userdata.joint_trajectory = result.planned_trajectory.joint_trajectory
					return 'planned'
				else:
					self._excute_success = True
					userdata.joint_trajectory = result.executed_trajectory.joint_trajectory
					return 'reached'

			
	def on_enter(self, userdata):
		self._planning_failed = False
		self._control_failed = False
		self._plan_success = False
		self._excute_success = False
	
		# self._plan_mode = userdata.plan_mode


		action_goal = MoveGroupGoal()
		action_goal.request.group_name = self._move_group
		if self._plan_mode == PlanMode.PLAN_ONLY.value:
			action_goal.planning_options.plan_only = True
			action_goal.planning_options.look_around = False
			print('mode 1')
		elif self._plan_mode == PlanMode.OFFLINE.value:
			action_goal.planning_options.plan_only = False
			action_goal.planning_options.look_around = False
			print('mode 2')
		elif self._plan_mode == PlanMode.ONLINE.value:
			action_goal.planning_options.plan_only = False
			action_goal.planning_options.look_around = True
			print('mode 3')
		else:
			print('FAIL')
		
		goal_constraints = Constraints()
		goal_constraints.name = 'robot_' + str(self.robot_id)
		print(goal_constraints.name)

		for i, (joint_config, joint_name) in enumerate(zip(userdata.joint_config, self._joint_names)):
			if i == 5:
				joint_config = 0.0
			goal_constraints.joint_constraints.append(JointConstraint(joint_name=joint_name, position=joint_config))
		print('=====')
		print(goal_constraints.joint_constraints)
		print('~~~~~')
		action_goal.request.goal_constraints.append(goal_constraints)
		action_goal.request.start_state.joint_state.name = self._joint_names
		# if type(userdata.start_config) == list:
		# 	action_goal.request.start_state.joint_state.position = userdata.start_config if len(userdata.start_config) == 6 else [] 
		action_goal.request.start_state.joint_state.position = []
		try:
			self._client.send_goal(self._action_topic, action_goal)
		except Exception as e:
			Logger.logwarn('Failed to send action goal for group: %s\n%s' % (self._move_group, str(e)))
			self._planning_failed = True
			

	def on_stop(self):
		try:
			if self._client.is_available(self._action_topic) \
			and not self._client.has_result(self._action_topic):
				self._client.cancel(self._action_topic)
		except:
			# client already closed
			pass

	def on_pause(self):
		self._client.cancel(self._action_topic)

	def on_resume(self, userdata):
		self.on_enter(userdata)
