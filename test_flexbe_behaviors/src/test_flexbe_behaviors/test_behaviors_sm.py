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
		self.add_parameter('wait_time', 5)
		self.add_parameter('planner_topic', '/dcma_planner/move_group')
		self.add_parameter('robot_1_id', 0)
		self.add_parameter('point_1', 'None')
		self.add_parameter('robot_ids', '[0]')
		self.add_parameter('plan_only', True)

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
										GetPoseState(robot_ids=self.robot_ids, joint_values_input=self.point_1),
										transitions={'done': 'plan', 'finish': 'finished'},
										autonomy={'done': Autonomy.Off, 'finish': Autonomy.Off},
										remapping={'robot_ids': 'robot_ids', 'plan_only': 'plan_only', 'joint_values': 'joint_values'})

			# x:312 y:148
			OperatableStateMachine.add('move_robot',
										RobotMoveState(robot_topic='/arm_controller/follow_joint_trajectory'),
										transitions={'done': 'get_pose', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:486 y:22
			OperatableStateMachine.add('plan',
										PlanningState(move_group='move_group', action_topic=self.planner_topic),
										transitions={'reached': 'get_pose', 'planning_failed': 'failed', 'control_failed': 'failed', 'planned': 'move_robot'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'planned': Autonomy.Off},
										remapping={'robot_ids': 'robot_ids', 'plan_only': 'plan_only', 'joint_config': 'joint_values', 'joint_trajectory': 'joint_trajectory'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
