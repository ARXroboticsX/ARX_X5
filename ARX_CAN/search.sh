#!/bin/bash
source ~/.bashrc
gnome-terminal -t "can" -x bash -c "source ~/.bashrc;udevadm info -a -n /dev/ttyACM* | grep serial;exec bash;"