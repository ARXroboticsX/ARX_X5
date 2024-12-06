#pragma once

#include <rclcpp/rclcpp.hpp>
#include "arx_x5_src/interfaces/InterfacesThread.hpp"
#include "arm_control/msg/pos_cmd.hpp"
#include <chrono>
#include <memory>

namespace arx::x5
{
    class X5Controller : public rclcpp::Node
    {
    public:
        X5Controller();

        void CmdCallback(const arm_control::msg::PosCmd::SharedPtr msg);
        void PubState();
        
    private:
        std::shared_ptr<InterfacesThread> l5_pro_Interfaces_ptr_;

        rclcpp::Publisher<arm_control::msg::PosCmd>::SharedPtr joint_state_publisher_;
        rclcpp::Subscription<arm_control::msg::PosCmd>::SharedPtr joint_state_subscriber_;
        rclcpp::TimerBase::SharedPtr timer_;
    };
}