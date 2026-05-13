#!/bin/bash
# Script to activate BT Server lifecycle node

echo "Waiting for BT Server to start..."
sleep 3

echo "Configuring BT Server..."
ros2 lifecycle set /bt_server configure

echo "Activating BT Server..."
ros2 lifecycle set /bt_server activate

echo "BT Server activated and ready!"
ros2 lifecycle get /bt_server
