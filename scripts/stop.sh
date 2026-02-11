#!/bin/bash
echo ["STOP VISION"]
killall -9 vision_node
echo ["STOP BRAIN"]
killall -9 brain_node
echo ["STOP SOUND"]
killall -9 sound_play_node
echo ["STOP GAMECONTROLLER"]
killall -9 game_controller
