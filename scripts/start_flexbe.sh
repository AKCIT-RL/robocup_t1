#!/bin/bash
###############################################################################
# RoboCup FlexBE System Startup Script
# 
# This script launches the complete FlexBE + BehaviorTree system for RoboCup
###############################################################################

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  RoboCup FlexBE System Startup${NC}"
echo -e "${GREEN}========================================${NC}"

# Source ROS setup
echo -e "${YELLOW}Sourcing ROS environment...${NC}"
source /opt/ros/humble/setup.bash
source /home/booster/booster_ws/install/setup.bash

# Parse arguments
LAUNCH_MODE=${1:-full}

case $LAUNCH_MODE in
  full)
    echo -e "${GREEN}Launching FULL system (BT Server + FlexBE + WebUI)${NC}"
    
    # Activate BT Server in background using Python script for reliability
    (sleep 5 && \
     echo -e "${YELLOW}Activating BT Server lifecycle node...${NC}" && \
     python3 -c "
import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
import time

rclpy.init()
node = Node('bt_server_activator')
client = node.create_client(ChangeState, '/bt_server/change_state')

if client.wait_for_service(timeout_sec=10.0):
    # Configure
    req = ChangeState.Request()
    req.transition = Transition()
    req.transition.id = Transition.TRANSITION_CONFIGURE
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    
    if future.result() and future.result().success:
        time.sleep(0.5)
        # Activate
        req = ChangeState.Request()
        req.transition = Transition()
        req.transition.id = Transition.TRANSITION_ACTIVATE
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        
        if future.result() and future.result().success:
            print('\033[0;32m✓ BT Server activated!\033[0m')
        else:
            print('\033[0;31m✗ Failed to activate bt_server!\033[0m')
    else:
        print('\033[0;31m✗ Failed to configure bt_server!\033[0m')
else:
    print('\033[0;31m✗ BT Server services not available!\033[0m')

node.destroy_node()
rclpy.shutdown()
" 2>&1) &
    
    ros2 launch robocup_bringup robocup_full.launch.py
    ;;
  
  flexbe)
    echo -e "${GREEN}Launching FlexBE system only (Onboard + WebUI)${NC}"
    echo -e "${YELLOW}Note: BT Server should be running separately${NC}"
    ros2 launch robocup_bringup flexbe_system.launch.py
    ;;
  
  bt)
    echo -e "${GREEN}Launching BT Server only${NC}"
    echo -e "${YELLOW}Note: FlexBE should be running separately${NC}"
    ros2 launch robocup_bringup brain_bt_server.launch.py
    ;;
  
  webui)
    echo -e "${GREEN}Launching FlexBE WebUI only${NC}"
    echo -e "${YELLOW}Accessing at: http://localhost:8000${NC}"
    ros2 run flexbe_webui webui_server
    ;;
  
  *)
    echo -e "${RED}Unknown launch mode: $LAUNCH_MODE${NC}"
    echo ""
    echo "Usage: $0 [mode]"
    echo ""
    echo "Available modes:"
    echo "  full    - Launch complete system (BT Server + FlexBE) [default]"
    echo "  flexbe  - Launch FlexBE system only (Onboard + WebUI)"
    echo "  bt      - Launch BT Server only"
    echo "  webui   - Launch WebUI only"
    echo ""
    exit 1
    ;;
esac
