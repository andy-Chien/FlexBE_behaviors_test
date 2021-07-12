#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from test_flexbe_behaviors.test_behaviors_sm import test_behaviorsSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jul 08 2021
@author: And
'''
class three_arms_testSM(Behavior):
	'''
	test three arms in one time
	'''


	def __init__(self):
		super(three_arms_testSM, self).__init__()
		self.name = 'three_arms_test'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(test_behaviorsSM, 'Container/test_behaviors')
		self.add_behavior(test_behaviorsSM, 'Container/test_behaviors_2')
		self.add_behavior(test_behaviorsSM, 'Container/test_behaviors_3')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:463, x:130 y:463, x:230 y:463, x:330 y:463
		_sm_container_0 = ConcurrencyContainer(outcomes=['finished', 'failed'], conditions=[
										('finished', [('test_behaviors', 'finished'), ('test_behaviors_2', 'finished'), ('test_behaviors_3', 'finished')]),
										('failed', [('test_behaviors', 'failed'), ('test_behaviors_2', 'failed'), ('test_behaviors_3', 'failed')])
										])

		with _sm_container_0:
			# x:83 y:103
			OperatableStateMachine.add('test_behaviors',
										self.use_behavior(test_behaviorsSM, 'Container/test_behaviors'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:332 y:104
			OperatableStateMachine.add('test_behaviors_2',
										self.use_behavior(test_behaviorsSM, 'Container/test_behaviors_2'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:588 y:107
			OperatableStateMachine.add('test_behaviors_3',
										self.use_behavior(test_behaviorsSM, 'Container/test_behaviors_3'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})



		with _state_machine:
			# x:138 y:86
			OperatableStateMachine.add('Container',
										_sm_container_0,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
