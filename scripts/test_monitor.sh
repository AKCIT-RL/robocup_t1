#!/bin/bash
# Quick rebuild and restart for testing

cd ~/booster_ws

echo "🔨 Building bt_webmonitor..."
source /opt/ros/humble/setup.bash
colcon build --packages-select bt_webmonitor --symlink-install

if [ $? -ne 0 ]; then
    echo "❌ Build failed!"
    exit 1
fi

echo "✅ Build successful!"

# Force copy static files (since they might not update with symlink)
echo "📁 Copying static files..."
INSTALL_DIR="install/bt_webmonitor/share/bt_webmonitor/static"
if [ -d "$INSTALL_DIR" ]; then
    cp -f src/bt_webmonitor/bt_webmonitor/static/*.html "$INSTALL_DIR/" 2>/dev/null
    cp -f src/bt_webmonitor/bt_webmonitor/static/*.css "$INSTALL_DIR/" 2>/dev/null
    cp -f src/bt_webmonitor/bt_webmonitor/static/*.js "$INSTALL_DIR/" 2>/dev/null
    echo "✅ Static files updated!"
fi

echo ""
echo "🔄 Restarting services..."

# Kill existing monitor if running
pkill -f bt_webmonitor || true

# Source and launch
source install/setup.bash
sleep 2

echo "🚀 Launching BT Web Monitor..."
ros2 launch bt_webmonitor bt_webmonitor.launch.py

echo ""
echo "========================================="
echo "🌳 BT Web Monitor running!"
echo "   Open: http://$(hostname -I | awk '{print $1}'):8080"
echo "   Press Ctrl+Shift+R in browser to force refresh!"
echo "========================================="
