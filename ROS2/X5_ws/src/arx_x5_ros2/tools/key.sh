#!/bin/bash
source ~/.bashrc

workspace=$(pwd)

gnome-terminal -t "key" -x bash -c "source ~/.bashrc;\
cd ${workspace};\
source ./install/setup.bash;ros2 run arx_x5_controller KeyBoard ;exec bash;"


