#pragma once

#include <ros/ros.h>
#include "arx_r5_src/interfaces/InterfacesThread.hpp"
#include "arx5_arm_msg/RobotCmd.h"
#include "arx5_arm_msg/RobotStatus.h"
#include <chrono>
#include <memory>
#include <termio.h>

namespace arx::r5
{
    class KeyBoardNode
    {
    public:
        KeyBoardNode(ros::NodeHandle nh);

        void Update(const ros::TimerEvent &);

        int ScanKeyBoard();

    private:
        std::shared_ptr<InterfacesThread> interfaces_ptr_;

        ros::Publisher joint_cmd_publisher_;
        ros::Timer timer_;

        arx5_arm_msg::RobotCmd message_;

        int key_[3] = {0};
    };
}
