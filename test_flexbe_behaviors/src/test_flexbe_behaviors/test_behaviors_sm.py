#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_manipulation_states.moveit_to_joints_state import MoveitToJointsState
from flexbe_states.wait_state import WaitState
from test_flexbe_states.get_pose import GetPoseState
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
		self.add_parameter('robot_1_topic', 'motion_planner/start_online_plan')
		self.add_parameter('robot_1_id', 0)
		self.add_parameter('point_1', '0')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:19 y:62, x:56 y:209
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:132 y:25
			OperatableStateMachine.add('get_pose',
										GetPoseState(joint_values_string=self.point_1),
										transitions={'done': 'traj_plan', 'finish': 'finished'},
										autonomy={'done': Autonomy.Off, 'finish': Autonomy.Off},
										remapping={'joint_values': 'joint_values'})

			# x:355 y:17
			OperatableStateMachine.add('traj_plan',
										MoveitToJointsState(move_group='robot_0', joint_names=["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"], action_topic='/dcma_planner/move_group'),
										transitions={'reached': 'wait', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'joint_config': 'joint_values'})

			# x:384 y:143
			OperatableStateMachine.add('wait',
										WaitState(wait_time=self.wait_time),
										transitions={'done': 'get_pose'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
