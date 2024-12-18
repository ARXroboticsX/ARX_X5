#pragma once

#include <ros/ros.h>
#include "arx_x5_src/interfaces/InterfacesThread.hpp"
#include "arx5_arm_msg/RobotCmd.h"
#include "arx5_arm_msg/RobotStatus.h"

#include "arm_control/PosCmd.h"
#include "arm_control/JointInfomation.h"
#include "arm_control/JointControl.h"

#include <chrono>
#include <memory>

namespace arx::x5
{
    class X5Controller
    {
    public:
        X5Controller(ros::NodeHandle nh);

        void CmdCallback(const arx5_arm_msg::RobotCmd::ConstPtr &msg);
        void PubState(const ros::TimerEvent &);

        void OldCmdCallback(const arm_control::JointControl::ConstPtr &msg);
        void VrCmdCallback(const arm_control::PosCmd::ConstPtr &msg);
        void FollowCmdCallback(const arx5_arm_msg::RobotStatus::ConstPtr &msg);

        void PubOldJointState(std::vector<double> joint_pos_vector,
                              std::vector<double> joint_velocities_vector,
                              std::vector<double> joint_current_vector);

        void PubOldEndState(std::vector<double> xyzrpy, double gripper);

    private:
        bool pub_old_joint_topic_ = false;
        bool pub_old_end_topic_ = false;
        std::shared_ptr<InterfacesThread> interfaces_ptr_;

        ros::Publisher joint_state_publisher_;
        ros::Subscriber joint_state_subscriber_;

        ros::Publisher old_joint_state_publisher_;
        ros::Publisher old_ee_pos_publisher_;

        ros::Timer timer_;
    };
}
