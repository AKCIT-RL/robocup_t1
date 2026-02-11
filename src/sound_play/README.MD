#  播放几种机器人的音效

## 使用

```
colcon build
source install/setup.bash
ros2 topic pub -1 /play_sound std_msgs/String "exited" # options: alarm | exited | playful | ... see sounds dir for more
```


