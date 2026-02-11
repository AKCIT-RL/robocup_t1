#!/bin/bash
echo "[START VISION]"
cd `dirname $0`
cd ..

source ./install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=./configs/fastdds.xml

ros2 launch vision launch.py
