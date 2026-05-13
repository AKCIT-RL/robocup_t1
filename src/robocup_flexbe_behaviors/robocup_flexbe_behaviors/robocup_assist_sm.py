#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               RoboCup Assist Behavior                   #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, Logger
from flex_bt_flexbe_states.bt_execute_state import BtExecuteState

'''
Created on May 10 2026
@author: RoboCup Team
'''
class RobocupAssistSM(Behavior):
	'''
	AI-assisted teleoperation behavior for RoboCup.
	Executes the assist.xml BehaviorTree which tracks ball and assists with chase and kick.
	'''

	def __init__(self, node):
		super(RobocupAssistSM, self).__init__()
		self.name = 'RoboCup Assist'

		# Initialize ROS components
		OperatableStateMachine.initialize_ros(node)
		Logger.initialize(node)
		BtExecuteState.initialize_ros(node)

	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _state_machine:
			# Execute the assist.xml BehaviorTree
			OperatableStateMachine.add('Execute_Assist_BT',
										BtExecuteState(
											bt_topic='bt_executor',
											bt_file='brain/behavior_trees/assist.xml'
										),
										transitions={'done': 'finished', 'canceled': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'canceled': Autonomy.Off, 'failed': Autonomy.Off})

		return _state_machine
