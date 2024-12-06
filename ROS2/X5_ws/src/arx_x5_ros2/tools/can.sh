#!/bin/bash
source ~/.bashrc



sudo ip link set can0 down
sudo pkill -9 slcand



#while [ 1 ]
#do
	sudo -S slcand -o -f -s8 /dev/arxcan0 can0
	sudo ifconfig can0 up
	# sudo -S slcand -o -f -s8 /dev/arxcan1 can1
	# sudo ifconfig can1 up
	# sudo -S slcand -o -f -s8 /dev/arxcan2 can2
	# sudo ifconfig can2 up
	# sudo -S slcand -o -f -s8 /dev/arxcan3 can3
	# sudo ifconfig can3 up
	# sudo -S slcand -o -f -s8 /dev/arxcan5 can5
	# sudo ifconfig can5 up




#	sleep 1
#done







