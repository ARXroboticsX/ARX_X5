#!/bin/bash

workspace=$(pwd)

source ~/.bashrc


gnome-terminal -t "can0" -x sudo bash -c "cd ${workspace}/arx_can; ./arx_can0.sh; exec bash;"
#sleep 0.1
#gnome-terminal -t "can1" -x sudo bash -c "cd ${workspace}/arx_can; ./arx_can1.sh; exec bash;"
#sleep 0.1
#gnome-terminal -t "can2" -x sudo bash -c "cd ${workspace}/arx_can; ./arx_can2.sh; exec bash;"
#sleep 0.1
#gnome-terminal -t "can3" -x sudo bash -c "cd ${workspace}/arx_can; ./arx_can3.sh; exec bash;"


