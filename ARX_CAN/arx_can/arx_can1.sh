#!/bin/bash

source ~/.bashrc

CAN_DEVICE="/dev/arxcan1"
CAN_INTERFACE="can1"


start_can() {
    echo "启动 slcand..."
    sudo slcand -o -f -s8 $CAN_DEVICE $CAN_INTERFACE
    if [ $? -ne 0 ]; then
        echo "slcand 启动失败"
        return 1
    fi
    echo "配置 $CAN_INTERFACE 接口..."


    sudo slcand -o -f -s8 $CAN_DEVICE $CAN_INTERFACE
    sudo ifconfig $CAN_INTERFACE up
    
    if [ $? -ne 0 ]; then
        echo "启动 $CAN_INTERFACE 接口失败：RTNETLINK answers: Operation not supported"
        return 1
    fi
    echo "$CAN_INTERFACE 启动成功"
    return 0
}

check_can() {

    if ip link show "$CAN_INTERFACE" > /dev/null 2>&1; then
  
        if ip link show "$CAN_INTERFACE" | grep -q "UP"; then
            return 0
        else
            return 1  
        fi
    else
        return 2  
    fi
}

while true; do

    if check_can; then

        echo "CAN 接口 $CAN_INTERFACE 正常工作"
    else

        echo "$CAN_INTERFACE 掉线，重启中..."
        
        sudo ip link set $CAN_INTERFACE down
        sudo pkill -9 slcand  
        sleep 1  

        if ! start_can; then
            echo "重启 CAN 接口失败，请检查硬件或驱动。"

        fi
    fi


done
