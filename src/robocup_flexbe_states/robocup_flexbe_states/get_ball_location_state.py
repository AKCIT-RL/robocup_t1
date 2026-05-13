#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
GetBallLocationState - FlexBE State for retrieving ball position from vision system.

Subscribes to vision detections and outputs ball location information.
"""

from flexbe_core import EventState, Logger

# Try to import vision messages and proxy
try:
    from vision.msg import Detections, DetectedObject
    from flexbe_core.proxy import ProxySubscriberCached
    VISION_AVAILABLE = True
except ImportError:
    # Vision messages not available - graceful degradation
    Detections = None
    DetectedObject = None
    VISION_AVAILABLE = False
    ProxySubscriberCached = None


class GetBallLocationState(EventState):
    """
    State to retrieve ball location from vision system.
    
    This state subscribes to the vision detections topic and extracts
    the ball position if detected.
    
    -- topic            string      Topic name for vision detections (default: '/vision/detections')
    -- timeout          float       Timeout in seconds to wait for detection (default: 2.0)
    -- ball_label       string      Label for ball object in detections (default: 'ball')
    
    #> ball_detected    bool        Whether ball was detected
    #> ball_position    list        Ball position [x, y, z] in camera/world frame
    #> ball_confidence  float       Detection confidence (0.0 - 1.0)
    
    <= detected                     Ball successfully detected
    <= not_detected                 Ball not found in detections
    <= timeout                      Timeout waiting for vision data
    <= failed                       Failed to process vision data
    """
    
    def __init__(self, topic='/vision/detections', timeout=2.0, ball_label='ball'):
        """Initialize GetBallLocationState."""
        super(GetBallLocationState, self).__init__(
            outcomes=['detected', 'not_detected', 'timeout', 'failed'],
            output_keys=['ball_detected', 'ball_position', 'ball_confidence']
        )
        
        self._topic = topic
        self._timeout = timeout
        self._ball_label = ball_label
        self._start_time = None
        self._processed = False
        self._sub = None
    
    def on_start(self):
        """Called when state machine starts."""
        # Only initialize subscriber if vision is available
        if VISION_AVAILABLE and ProxySubscriberCached is not None:
            ProxySubscriberCached.initialize(GetBallLocationState._node)
            self._sub = ProxySubscriberCached({self._topic: Detections})
    
    def execute(self, userdata):
        """Execute state - check for ball detection."""
        # If vision not available, return timeout immediately
        if not VISION_AVAILABLE or self._sub is None:
            userdata.ball_detected = False
            userdata.ball_position = [0.0, 0.0, 0.0]
            userdata.ball_confidence = 0.0
            return 'timeout'
        
        if self._processed:
            if userdata.ball_detected:
                return 'detected'
            else:
                return 'not_detected'
        
        # Check for timeout
        elapsed = (GetBallLocationState._node.get_clock().now() - self._start_time).nanoseconds / 1e9
        if elapsed > self._timeout:
            Logger.logwarn(f'{self.name}: Timeout waiting for vision detections')
            userdata.ball_detected = False
            userdata.ball_position = [0.0, 0.0, 0.0]
            userdata.ball_confidence = 0.0
            return 'timeout'
        
        # Check if vision data available
        if self._sub.has_msg(self._topic):
            msg = self._sub.get_last_msg(self._topic)
            self._sub.remove_last_msg(self._topic)
            
            try:
                # Search for ball in detected objects
                ball_found = False
                for obj in msg.detected_objects:
                    if obj.label.lower() == self._ball_label.lower():
                        ball_found = True
                        userdata.ball_detected = True
                        userdata.ball_position = list(obj.position) if len(obj.position) > 0 else [0.0, 0.0, 0.0]
                        userdata.ball_confidence = float(obj.confidence)
                        
                        Logger.loginfo(f'{self.name}: Ball detected at position {userdata.ball_position} with confidence {userdata.ball_confidence}')
                        self._processed = True
                        return 'detected'
                
                if not ball_found:
                    Logger.loginfo(f'{self.name}: Ball not found in detections')
                    userdata.ball_detected = False
                    userdata.ball_position = [0.0, 0.0, 0.0]
                    userdata.ball_confidence = 0.0
                    self._processed = True
                    return 'not_detected'
                    
            except Exception as e:
                Logger.logerr(f'{self.name}: Error processing vision data: {str(e)}')
                userdata.ball_detected = False
                userdata.ball_position = [0.0, 0.0, 0.0]
                userdata.ball_confidence = 0.0
                return 'failed'
        
        return None
    
    def on_enter(self, userdata):
        """Called when state is entered."""
        self._start_time = GetBallLocationState._node.get_clock().now()
        self._processed = False
        
        if not VISION_AVAILABLE or self._sub is None:
            Logger.logwarn(f'{self.name}: Vision messages not available - state will timeout')
        
        Logger.loginfo(f'{self.name}: Waiting for vision detections on topic: {self._topic}')
    
    def on_exit(self, userdata):
        """Called when state is exited."""
        pass
