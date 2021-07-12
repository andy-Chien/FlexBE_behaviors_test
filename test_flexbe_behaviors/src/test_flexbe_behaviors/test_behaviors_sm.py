#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from test_flexbe_states.get_pose import GetPoseState
from test_flexbe_states.planning_state import PlanningState
from test_flexbe_states.robot_move import RobotMoveState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jun 24 2021
@author: Andy Chien
'''
class test_behaviorsSM(Behavior):
	'''
	test behaviors
	'''


	def __init__(self):
		super(test_behaviorsSM, self).__init__()
		self.name = 'test_behaviors'

		# parameters of this behavior
		self.add_parameter('planner_topic', 'robot_0/dcma_planner/move_group')
		self.add_parameter('robot_topic', 'robot_0/arm_controller/follow_joint_trajectory')
		self.add_parameter('robot_id', 0)
		self.add_parameter('plan_mode', 'plan_only')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:169 y:180, x:535 y:165
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:132 y:25
			OperatableStateMachine.add('get_pose',
										GetPoseState(robot_id=self.robot_id, plan_mode=self.plan_mode),
										transitions={'done': 'plan', 'finish': 'finished'},
										autonomy={'done': Autonomy.Off, 'finish': Autonomy.Off},
										remapping={'robot_id': 'robot_id', 'joint_values': 'joint_values', 'plan_mode': 'plan_mode', 'start_config': 'start_config'})

			# x:312 y:148
			OperatableStateMachine.add('move_robot',
										RobotMoveState(robot_topic=self.robot_topic),
										transitions={'done': 'get_pose', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:486 y:22
			OperatableStateMachine.add('plan',
										PlanningState(move_group='move_group', action_topic=self.planner_topic),
										transitions={'reached': 'get_pose', 'planning_failed': 'failed', 'control_failed': 'failed', 'planned': 'move_robot'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'planned': Autonomy.Off},
										remapping={'robot_id': 'robot_id', 'plan_mode': 'plan_mode', 'joint_config': 'joint_values', 'start_config': 'start_config', 'joint_trajectory': 'joint_trajectory'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
