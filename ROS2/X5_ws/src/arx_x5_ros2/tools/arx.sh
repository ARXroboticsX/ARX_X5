#!/bin/bash

# 读取用户的 bashrc 配置
source ~/.bashrc

# 设备路径
CAN_DEVICE="/dev/arxcan0"
CAN_INTERFACE="can0"


# 启动 CAN 接口
start_can() {
    echo "启动 slcand..."
    sudo slcand -o -f -s8 $CAN_DEVICE $CAN_INTERFACE
    if [ $? -ne 0 ]; then
        echo "slcand 启动失败"
        return 1
    fi
    echo "配置 can0 接口..."
    # sudo ip link set $CAN_INTERFACE up type can bitrate $CAN_BITRATE

    # sudo -S slcand -o -f -s8 /dev/arxcan0 can0
    # sudo ifconfig can0 up

    sudo slcand -o -f -s8 $CAN_DEVICE $CAN_INTERFACE
    sudo ifconfig $CAN_INTERFACE up
    
    if [ $? -ne 0 ]; then
        echo "启动 can0 接口失败：RTNETLINK answers: Operation not supported"
        return 1
    fi
    echo "$CAN_INTERFACE 启动成功"
    return 0
}

# 检查 can0 是否处于活动状态
check_can() {
    # 检查接口是否存在
    if ip link show "$CAN_INTERFACE" > /dev/null 2>&1; then
        # 检查接口状态是否为 UP
        if ip link show "$CAN_INTERFACE" | grep -q "state UP"; then
            return 1  # 表示CAN接口已开启
        else
            return 0  # 表示CAN接口存在但未开启
        fi
    else
        return 2  # 表示CAN接口不存在
    fi
}


# 主循环，持续检测 CAN 接口状态
while true; do
    # 检查 can0 是否连接
    if check_can; then
        # 接口正常，继续工作
        echo "CAN 接口 $CAN_INTERFACE 正常工作"
    else
        # 接口掉线，重启中...
        echo "$CAN_INTERFACE 掉线，重启中..."
        
        # 停止并重启 slcand 和 can0
        sudo ip link set $CAN_INTERFACE down
        sudo pkill -9 slcand  # 强制杀掉 slcand 进程
        sleep 1  # 稍等一会儿，确保进程结束

        # 重新启动 slcand 和 can0 接口
        if ! start_can; then
            echo "重启 CAN 接口失败，请检查硬件或驱动。"
            # exit 1
        fi
    fi

    # 每 1 秒检测一次
    # sleep 1
done
