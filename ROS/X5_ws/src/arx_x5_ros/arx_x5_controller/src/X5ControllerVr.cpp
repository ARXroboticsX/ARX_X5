#include "arx_x5_controller/X5ControllerVr.hpp"

// using namespace std::chrono_literals;

namespace arx::x5
{
    X5Controller::X5Controller(ros::NodeHandle nh)
    {
        // 创建发布器
        joint_state_publisher_ = nh.advertise<arm_control::PosCmd>("/x5_status", 10);
        // 创建订阅器
        joint_state_subscriber_ = nh.subscribe<arm_control::PosCmd>(
            "/ARX_VR_L", 10, &X5Controller::CmdCallback, this);
        // 定时器，用于发布关节信息

        timer_ = nh.createTimer(ros::Duration(0.01), &X5Controller::PubState, this);
        x5_Interfaces_ptr_ = std::make_shared<InterfacesThread>("can0",0);
    }

    void X5Controller::CmdCallback(const arm_control::PosCmd::ConstPtr& msg)
    {
        double input[6] = {msg->x, msg->y, msg->z, msg->roll, msg->pitch, msg->yaw};
        Eigen::Isometry3d transform = solve::Xyzrpy2Isometry(input);

        x5_Interfaces_ptr_->setEndPose(transform);

        x5_Interfaces_ptr_->setArmStatus(4);

        x5_Interfaces_ptr_->setCatch(msg->gripper);
    }

    void X5Controller::PubState(const ros::TimerEvent&)
    {
        arm_control::PosCmd msg;
        // message.header.stamp = this->get_clock()->now();

        Eigen::Isometry3d transform = x5_Interfaces_ptr_->getEndPose();

        // 提取四元数和位移
        Eigen::Quaterniond quat(transform.rotation());
        Eigen::Vector3d translation = transform.translation();

        std::vector<double> xyzrpy = solve::Isometry2Xyzrpy(transform);

        // 填充vector

        msg.x = xyzrpy[0];
        msg.y = xyzrpy[1];
        msg.z = xyzrpy[2];
        msg.roll = xyzrpy[3];
        msg.pitch = xyzrpy[4];
        msg.yaw = xyzrpy[5];
        msg.quater_x = quat.x();
        msg.quater_y = quat.y();
        msg.quater_z = quat.z();
        msg.quater_w = quat.w();

        // 发布消息
        joint_state_publisher_.publish(msg);
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "x5_controller");
    ros::NodeHandle nh = ros::NodeHandle("~");
    arx::x5::X5Controller controller(nh);
    ros::spin();
    return 0;
}
