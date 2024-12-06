#!/bin/bash
workspace=$(pwd)

source ~/.bashrc



 gnome-terminal -t "master1" -x sudo bash -c "cd ${workspace}; sudo cp arx_can.rules /etc/udev/rules.d/;sleep 1;sudo chmod +x /etc/udev/rules.d/arx_can.rules;sleep 1;sudo udevadm control --reload-rules && sudo udevadm trigger; exec bash;"


