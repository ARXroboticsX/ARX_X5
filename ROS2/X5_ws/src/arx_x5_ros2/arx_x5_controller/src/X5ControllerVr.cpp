#include "arx_x5_controller/X5ControllerVr.hpp"

// using namespace std::chrono_literals;

namespace arx::x5
{
    X5Controller::X5Controller() : Node("l5_pro_controller_node")
    {

        // 创建发布器
        joint_state_publisher_ = this->create_publisher<arm_control::msg::PosCmd>("x5_status", 10);
        // 创建订阅器
        joint_state_subscriber_ = this->create_subscription<arm_control::msg::PosCmd>(
            "ARX_VR_L", 10, std::bind(&X5Controller::CmdCallback, this, std::placeholders::_1));
        // 定时器，用于发布关节信息

        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&X5Controller::PubState, this));

        l5_pro_Interfaces_ptr_ = std::make_shared<InterfacesThread>("can0",0);
    }

    void X5Controller::CmdCallback(const arm_control::msg::PosCmd::SharedPtr msg)
    {
        double input[6] = {msg->x, msg->y, msg->z, msg->roll, msg->pitch, msg->yaw};
        Eigen::Isometry3d transform = solve::Xyzrpy2Isometry(input);

        l5_pro_Interfaces_ptr_->setEndPose(transform);

        l5_pro_Interfaces_ptr_->setArmStatus(InterfacesThread::state::END_CONTROL);

        l5_pro_Interfaces_ptr_->setCatch(msg->gripper);
    }

    void X5Controller::PubState()
    {
        auto message = arm_control::msg::PosCmd();
        // message.header.stamp = this->get_clock()->now();

        Eigen::Isometry3d transform = l5_pro_Interfaces_ptr_->getEndPose();

        // 提取四元数和位移
        Eigen::Quaterniond quat(transform.rotation());
        Eigen::Vector3d translation = transform.translation();

        std::vector<double> xyzrpy = solve::Isometry2Xyzrpy(transform);

        // 填充vector

        message.x = xyzrpy[0];
        message.y = xyzrpy[1];
        message.z = xyzrpy[2];
        message.roll = xyzrpy[3];
        message.pitch = xyzrpy[4];
        message.yaw = xyzrpy[5];
        message.quater_x = quat.x();
        message.quater_y = quat.y();
        message.quater_z = quat.z();
        message.quater_w = quat.w();

        // 发布消息
        joint_state_publisher_->publish(message);
    }

}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<arx::x5::X5Controller>());
    rclcpp::shutdown();
    return 0;
}