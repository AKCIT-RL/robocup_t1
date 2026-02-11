#!/bin/bash
echo "[START SOUND]"

cd `dirname $0`
cd ..

source ./install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=./configs/fastdds.xml

ros2 run sound_play sound_play_node
