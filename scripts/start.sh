#!/bin/bash

cd `dirname $0`
cd ..

echo "[STOP EXISTING NODES (IF ANY), TO AVOID CONFILICT]"
./scripts/stop.sh

source ./install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=./configs/fastdds.xml

echo "[START ROBOCUP NODES]"
echo "[START VISION]"
# 如果是用的zed相机，https://booster.feishu.cn/wiki/XodtwX56AiCtZtkewo3cPgbrn8d#share-MDrvdyWa2o87qixU3TccE72NnNc 文档中下载安装包，可以自启动zed
# 注意: 如果是realsense相机，千万不要改，否则realsense摄像头会起不来，这个时候只能cd ~/Documents/recovery/ 重装daemon-perception恢复
# source ~/ThirdParty/zed-ros/install/setup.bash
# nohup ros2 launch zed_wrapper zed_camera.launch.py camera_model:="zed2i" > zed.log 2>&1 &
nohup ros2 launch vision launch.py save_data:=true > vision.log 2>&1 &
echo "[START BRAIN]"
nohup ros2 launch brain launch.py "$@"  > brain.log 2>&1 &
echo "[START GAME_CONTROLLER]"
nohup ros2 launch game_controller launch.py > game_controller.log 2>&1 &
echo "[START SOUND]"
nohup ros2 run sound_play sound_play_node > sound.log 2>&1 &
echo "[DONE]"
