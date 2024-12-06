#pragma once

#include <ros/ros.h>
#include "arx_x5_src/interfaces/InterfacesThread.hpp"
#include "arx_x5_msg/RobotCmd.h"
#include "arx_x5_msg/RobotStatus.h"
#include <chrono>
#include <memory>
#include <termio.h>

namespace arx::x5
{
    class KeyBoardNode
    {
    public:
        KeyBoardNode(ros::NodeHandle nh);

        void Update(const ros::TimerEvent &);

        int ScanKeyBoard();

    private:
        std::shared_ptr<InterfacesThread> x5_Interfaces_ptr_;

        ros::Publisher joint_cmd_publisher_;
        ros::Timer timer_;

        arx_x5_msg::RobotCmd message_;

        int key_[3] = {0};
    };
}
