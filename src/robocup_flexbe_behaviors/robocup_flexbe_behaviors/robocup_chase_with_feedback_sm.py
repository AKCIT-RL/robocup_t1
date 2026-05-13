#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#       RoboCup Chase with Feedback Behavior              #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, Logger
from flex_bt_flexbe_states.bt_execute_state import BtExecuteState
from robocup_flexbe_states.get_ball_location_state import GetBallLocationState
from robocup_flexbe_states.play_sound_action_state import PlaySoundActionState
from flexbe_states.log_state import LogState

'''
Created on May 12 2026
@author: RoboCup Team
'''
class RobocupChaseWithFeedbackSM(Behavior):
	'''
	Enhanced ball chase behavior with vision feedback and sound notifications.
	Demonstrates integration of custom states with BehaviorTree execution.
	'''

	def __init__(self, node):
		super(RobocupChaseWithFeedbackSM, self).__init__()
		self.name = 'RoboCup Chase with Feedback'

		# Initialize ROS components
		OperatableStateMachine.initialize_ros(node)
		Logger.initialize(node)
		BtExecuteState.initialize_ros(node)
		GetBallLocationState.initialize_ros(node)
		PlaySoundActionState.initialize_ros(node)
		LogState.initialize_ros(node)

	def create(self):
		# x:30 y:365, x:130 y:365, x:230 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'not_detected'])

		with _state_machine:
			# Play start sound
			OperatableStateMachine.add('Play_Start_Sound',
										PlaySoundActionState(sound_name='start', blocking=False),
										transitions={'done': 'Check_Ball_Location', 'failed': 'Check_Ball_Location'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# Check if ball is detected before starting chase
			OperatableStateMachine.add('Check_Ball_Location',
										GetBallLocationState(topic='/vision/detections', timeout=3.0),
										transitions={'detected': 'Log_Ball_Found', 'not_detected': 'Play_Search_Sound', 
													'timeout': 'Play_Search_Sound', 'failed': 'failed'},
										autonomy={'detected': Autonomy.Off, 'not_detected': Autonomy.Off, 
												 'timeout': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'ball_detected': 'ball_detected', 'ball_position': 'ball_position', 
												  'ball_confidence': 'ball_confidence'})

			# Log ball detection
			OperatableStateMachine.add('Log_Ball_Found',
										LogState(text='Ball detected! Starting chase...', severity=Logger.REPORT_HINT),
										transitions={'done': 'Execute_Chase_BT'},
										autonomy={'done': Autonomy.Off})

			# Play search sound if ball not detected
			OperatableStateMachine.add('Play_Search_Sound',
										PlaySoundActionState(sound_name='search', blocking=False),
										transitions={'done': 'Execute_Chase_BT', 'failed': 'Execute_Chase_BT'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# Execute the chase.xml BehaviorTree
			OperatableStateMachine.add('Execute_Chase_BT',
										BtExecuteState(
											bt_topic='bt_executor',
											bt_file='brain/behavior_trees/chase.xml'
										),
										transitions={'done': 'Play_Success_Sound', 'canceled': 'Play_Success_Sound', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'canceled': Autonomy.Off, 'failed': Autonomy.Off})

			# Play success sound when chase completes
			OperatableStateMachine.add('Play_Success_Sound',
										PlaySoundActionState(sound_name='playful', blocking=True, wait_time=2.0),
										transitions={'done': 'finished', 'failed': 'finished'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

		return _state_machine
