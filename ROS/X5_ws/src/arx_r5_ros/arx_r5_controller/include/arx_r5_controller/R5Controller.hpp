#pragma once

#include <ros/ros.h>
#include "arx_r5_src/interfaces/InterfacesThread.hpp"
#include "arx5_arm_msg/RobotCmd.h"
#include "arx5_arm_msg/RobotStatus.h"

#include "arm_control/PosCmd.h"
#include "arm_control/JointInfomation.h"
#include "arm_control/JointControl.h"

#include <chrono>
#include <memory>

namespace arx::r5
{
    class R5Controller
    {
    public:
        R5Controller(ros::NodeHandle nh);

        // sub
        void CmdCallbackV1(const arm_control::PosCmd::ConstPtr &msg);
        void CmdCallbackV2(const arx5_arm_msg::RobotCmd::ConstPtr &msg);

        void FollowCallbackV1(const arm_control::JointInfomation::ConstPtr &msg);
        void FollowCallbackV2(const arx5_arm_msg::RobotStatus::ConstPtr &msg);

        void JointControlCallbackV1(const arm_control::JointControl::ConstPtr &msg);

        // pub
        void PubState(const ros::TimerEvent &);
        // 发布old_joint_state_publisher_和old_ee_pos_publisher_信息
        void pubArmStatusV1(std::vector<double> joint_pos_vector,
                            std::vector<double> joint_velocities_vector,
                            std::vector<double> joint_current_vector,
                            std::vector<double> xyzrpy);

        void pubArmStatusV2(std::vector<double> joint_pos_vector,
                            std::vector<double> joint_velocities_vector,
                            std::vector<double> joint_current_vector,
                            std::vector<double> xyzrpy);

    private:

        bool pub_topic_v1_ = false;
        bool pub_topic_v2_ = false;
        std::shared_ptr<InterfacesThread> interfaces_ptr_;

        ros::Publisher joint_state_publisher_;
        ros::Subscriber joint_state_subscriber_;

        ros::Publisher joint_state_publisher_v1_;
        ros::Publisher ee_pos_publisher_v1_;

        ros::Timer timer_;
    };
}
