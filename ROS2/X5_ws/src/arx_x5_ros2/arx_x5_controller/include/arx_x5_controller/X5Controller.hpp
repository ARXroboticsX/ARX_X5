#pragma once

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>

#include "arx_x5_src/interfaces/InterfacesThread.hpp"

#include "arx5_arm_msg/msg/robot_cmd.hpp"
#include "arx5_arm_msg/msg/robot_status.hpp"
#include "arm_control/msg/pos_cmd.hpp"

namespace arx::x5 {
    class X5Controller : public rclcpp::Node {
    public:
        X5Controller();

        void CmdCallback(const arx5_arm_msg::msg::RobotCmd::SharedPtr msg);

        void PubState();

        void VrCmdCallback(const arm_control::msg::PosCmd::SharedPtr msg);

        void VrPubState();

        void FollowCmdCallback(const arx5_arm_msg::msg::RobotStatus::SharedPtr msg);

    private:
        std::shared_ptr<InterfacesThread> interfaces_ptr_;

        // 通常 & remote从机模式下
        rclcpp::Publisher<arx5_arm_msg::msg::RobotStatus>::SharedPtr joint_state_publisher_;
        // vr模式下
        rclcpp::Publisher<arm_control::msg::PosCmd>::SharedPtr vr_joint_state_publisher_;

        // 通常模式下
        rclcpp::Subscription<arx5_arm_msg::msg::RobotCmd>::SharedPtr joint_state_subscriber_;
        // vr模式下
        rclcpp::Subscription<arm_control::msg::PosCmd>::SharedPtr vr_joint_state_subscriber_;
        // remote从机模式下
        rclcpp::Subscription<arx5_arm_msg::msg::RobotStatus>::SharedPtr follow_joint_state_subscriber_;

        rclcpp::TimerBase::SharedPtr timer_;
    };
}
