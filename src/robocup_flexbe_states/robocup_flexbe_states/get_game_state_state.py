#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
GetGameStateState - FlexBE State for reading game controller state.

Subscribes to game controller topic and outputs current game state information.
"""

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from std_msgs.msg import String


class GetGameStateState(EventState):
    """
    State to retrieve the current game state from game controller.
    
    This state subscribes to the game controller topic and retrieves
    the current game phase, state, and other relevant information.
    
    -- topic            string      Topic name for game controller state (default: '/game_controller/state')
    -- timeout          float       Timeout in seconds to wait for message (default: 2.0)
    
    #> game_state       string      Current game state (e.g., 'INITIAL', 'READY', 'SET', 'PLAYING')
    #> game_phase       string      Current game phase
    
    <= received                     Successfully received game state
    <= timeout                      Timeout waiting for game state
    <= failed                       Failed to retrieve game state
    """
    
    def __init__(self, topic='/game_controller/state', timeout=2.0):
        """Initialize GetGameStateState."""
        super(GetGameStateState, self).__init__(
            outcomes=['received', 'timeout', 'failed'],
            output_keys=['game_state', 'game_phase']
        )
        
        self._topic = topic
        self._timeout = timeout
        self._start_time = None
        self._received = False
        
        # Initialize subscriber
        ProxySubscriberCached.initialize(GetGameStateState._node)
        self._sub = ProxySubscriberCached({self._topic: String})
    
    def execute(self, userdata):
        """Execute state - check if message received."""
        if self._received:
            return 'received'
        
        # Check for timeout
        elapsed = (GetGameStateState._node.get_clock().now() - self._start_time).nanoseconds / 1e9
        if elapsed > self._timeout:
            Logger.logwarn(f'{self.name}: Timeout waiting for game controller state')
            return 'timeout'
        
        # Check if message available
        if self._sub.has_msg(self._topic):
            msg = self._sub.get_last_msg(self._topic)
            self._sub.remove_last_msg(self._topic)
            
            # Parse game state message
            # Assuming message format: "STATE:PHASE" or similar
            # Adjust based on actual game_controller message format
            try:
                if hasattr(msg, 'data'):
                    parts = msg.data.split(':')
                    userdata.game_state = parts[0] if len(parts) > 0 else 'UNKNOWN'
                    userdata.game_phase = parts[1] if len(parts) > 1 else 'UNKNOWN'
                else:
                    userdata.game_state = str(msg)
                    userdata.game_phase = 'UNKNOWN'
                
                Logger.loginfo(f'{self.name}: Received game state: {userdata.game_state}, phase: {userdata.game_phase}')
                self._received = True
                return 'received'
            except Exception as e:
                Logger.logerr(f'{self.name}: Error parsing game state: {str(e)}')
                return 'failed'
        
        return None
    
    def on_enter(self, userdata):
        """Called when state is entered."""
        self._start_time = GetGameStateState._node.get_clock().now()
        self._received = False
        Logger.loginfo(f'{self.name}: Waiting for game controller state on topic: {self._topic}')
    
    def on_exit(self, userdata):
        """Called when state is exited."""
        pass
