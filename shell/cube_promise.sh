#!/bin/bash
# File              : cube.sh
# Author            : Taichi Sekikawa <sekikawa.taichi.vf@tut.jp>
# Date              : 2023 05/11
# Last Modified Date: 2023 05/11
# Last Modified By  : Taichi Sekikawa <sekikawa.taichi.vf@tut.jp>

screen -AdmS dc_blue ros2 launch cube_dc dc_promise_blue_launch.py
sleep 1
screen -AdmS dc_green ros2 launch cube_dc dc_promise_green_launch.py
sleep 1
screen -AdmS dc_yellow ros2 launch cube_dc dc_promise_yellow_launch.py
sleep 1
# screen -AdmS toio roslaunch pokebo_cube toio.launch
sleep 2
screen -AdmS main ros2 launch pokebo_cube cube_promise_launch.py
