#!/bin/bash

echo "安装依赖"
sudo apt install ros-$ROS_DISTRO-serial-driver
sudo apt install ros-$ROS_DISTRO-asio-cmake-module
ls -l /dev/ttyACM*
# 创建 udev 规则文件
RULES_FILE="/etc/udev/rules.d/99-ttyACM.rules"

# 检查规则文件是否已存在
if [ -f "$RULES_FILE" ]; then
    echo "udev 规则文件已存在: $RULES_FILE"
else
    # 创建并写入规则文件
    echo "创建 udev 规则文件 $RULES_FILE"
    echo 'SUBSYSTEM=="tty", KERNEL=="ttyACM*", MODE="0666", GROUP="dialout"' | sudo tee "$RULES_FILE" > /dev/null
fi

# 重新加载 udev 规则
echo "重新加载 udev 规则..."
sudo udevadm control --reload-rules
sudo udevadm trigger

# 提示用户将自己添加到 dialout 组
echo "确保当前用户已被添加到 dialout 组..."
sudo usermod -aG dialout "$USER"

# 提示用户重新登录或使用 newgrp 切换组
echo "操作完成。请重新登录，或运行 'newgrp dialout' 使更改生效。"

# 提供验证信息
echo "您可以通过以下命令验证组成员信息："
echo "groups $USER"
