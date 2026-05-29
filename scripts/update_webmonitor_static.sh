#!/bin/bash
# Force update static files for bt_webmonitor

cd ~/booster_ws

echo "🔄 Copying static files to install directory..."

# Find install directory
INSTALL_DIR="install/bt_webmonitor/share/bt_webmonitor"

if [ ! -d "$INSTALL_DIR" ]; then
    echo "❌ Install directory not found: $INSTALL_DIR"
    exit 1
fi

# Copy static files
cp -v src/bt_webmonitor/bt_webmonitor/static/index.html "$INSTALL_DIR/static/"
cp -v src/bt_webmonitor/bt_webmonitor/static/styles.css "$INSTALL_DIR/static/"
cp -v src/bt_webmonitor/bt_webmonitor/static/bt_visualizer.js "$INSTALL_DIR/static/"

echo "✅ Static files updated!"
echo ""
echo "🔄 Now restart the webmonitor:"
echo "   pkill -f bt_webmonitor"
echo "   ros2 launch bt_webmonitor bt_webmonitor.launch.py"
