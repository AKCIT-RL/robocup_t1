#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               RoboCup Chase Behavior                    #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, Logger
from flex_bt_flexbe_states.bt_execute_state import BtExecuteState

'''
Created on May 10 2026
@author: RoboCup Team
'''
class RobocupChaseSM(Behavior):
	'''
	Ball chasing demonstration behavior for RoboCup.
	Executes the chase.xml BehaviorTree which finds and chases the ball with audio feedback.
	'''

	def __init__(self, node):
		super(RobocupChaseSM, self).__init__()
		self.name = 'RoboCup Chase'

		# Initialize ROS components
		OperatableStateMachine.initialize_ros(node)
		Logger.initialize(node)
		BtExecuteState.initialize_ros(node)

	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _state_machine:
			# Execute the chase.xml BehaviorTree
			OperatableStateMachine.add('Execute_Chase_BT',
										BtExecuteState(
											bt_topic='bt_executor',
											bt_file='brain/behavior_trees/chase.xml'
										),
										transitions={'done': 'finished', 'canceled': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'canceled': Autonomy.Off, 'failed': Autonomy.Off})

		return _state_machine
