#pragma once

#include <ros/ros.h>
#include "arx_x5_src/interfaces/InterfacesThread.hpp"
#include "arm_control/PosCmd.h"
#include <chrono>
#include <memory>

namespace arx::x5
{
    class X5Controller
    {
    public:
        X5Controller(ros::NodeHandle nh);

        void CmdCallback(const arm_control::PosCmd::ConstPtr& msg);
        void PubState(const ros::TimerEvent&);

    private:
        std::shared_ptr<InterfacesThread> x5_Interfaces_ptr_;

        ros::Publisher joint_state_publisher_;
        ros::Subscriber joint_state_subscriber_;
        ros::Timer timer_;
    };
}