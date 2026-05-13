#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
PlaySoundActionState - FlexBE State for playing sounds via sound_play.

Publishes sound command to trigger robot audio feedback.
"""

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from std_msgs.msg import String
import time


class PlaySoundActionState(EventState):
    """
    State to play a sound through the robot's sound system.
    
    This state publishes a sound command to the sound_play topic
    to trigger audio feedback on the robot.
    
    -- sound_name       string      Name/identifier of the sound to play (e.g., 'goal', 'start', 'warning')
    -- topic            string      Topic name for sound commands (default: '/robotsound')
    -- blocking         bool        Wait for sound to complete before transitioning (default: False)
    -- wait_time        float       Time to wait if blocking (default: 1.0)
    
    <= done                         Sound command published successfully
    <= failed                       Failed to publish sound command
    """
    
    def __init__(self, sound_name, topic='/robotsound', blocking=False, wait_time=1.0):
        """Initialize PlaySoundActionState."""
        super(PlaySoundActionState, self).__init__(
            outcomes=['done', 'failed']
        )
        
        self._sound_name = sound_name
        self._topic = topic
        self._blocking = blocking
        self._wait_time = wait_time
        self._start_time = None
        self._published = False
        
        # Initialize publisher
        ProxyPublisher.initialize(PlaySoundActionState._node)
        self._pub = ProxyPublisher({self._topic: String})
    
    def execute(self, userdata):
        """Execute state - publish sound command."""
        if not self._published:
            try:
                # Create and publish sound message
                msg = String()
                msg.data = self._sound_name
                self._pub.publish(self._topic, msg)
                
                Logger.loginfo(f'{self.name}: Published sound command: {self._sound_name}')
                self._published = True
                self._start_time = time.time()
                
            except Exception as e:
                Logger.logerr(f'{self.name}: Failed to publish sound: {str(e)}')
                return 'failed'
        
        # If blocking, wait for sound to complete
        if self._blocking:
            if time.time() - self._start_time >= self._wait_time:
                return 'done'
            return None
        else:
            return 'done'
    
    def on_enter(self, userdata):
        """Called when state is entered."""
        self._published = False
        self._start_time = None
        Logger.loginfo(f'{self.name}: Preparing to play sound: {self._sound_name}')
    
    def on_exit(self, userdata):
        """Called when state is exited."""
        pass
