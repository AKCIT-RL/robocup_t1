#!/bin/bash
# Start brain with BT web monitor

cd `dirname $0`
cd ..

echo "[STARTING ROBOCUP SYSTEM WITH BT MONITOR]"

# Start main system
./scripts/start.sh &

# Wait for brain_node to initialize and ZMQ ports to bind
echo "[WAITING FOR BRAIN_NODE TO START...]"
sleep 8

# Start BT web monitor
echo "[STARTING BT WEB MONITOR]"
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch bt_webmonitor bt_webmonitor.launch.py &

echo ""
echo "========================================="
echo "🌳 BT Web Monitor Available At:"
echo "   http://localhost:8080"
echo "========================================="
echo ""
echo "Press Ctrl+C to stop all services"

# Wait for user interrupt
wait
