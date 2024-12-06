#include "arx_x5_controller/X5Controller.hpp"

// using namespace std::chrono_literals;

namespace arx::x5
{
    X5Controller::X5Controller() : Node("l5_pro_controller_node")
    {

        // 创建发布器
        joint_state_publisher_ = this->create_publisher<arx_x5_msg::msg::RobotStatus>("x5_status", 1);
        // 创建订阅器
        joint_state_subscriber_ = this->create_subscription<arx_x5_msg::msg::RobotCmd>(
            "x5_cmd", 10, std::bind(&X5Controller::CmdCallback, this, std::placeholders::_1));
        // 定时器，用于发布关节信息

        timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&X5Controller::PubState, this));

        l5_pro_Interfaces_ptr_ = std::make_shared<InterfacesThread>("can0",0);
    }

    void X5Controller::CmdCallback(const arx_x5_msg::msg::RobotCmd::SharedPtr msg)
    {
        double end_pos[6] = {msg->end_pos[0], msg->end_pos[1], msg->end_pos[2], msg->end_pos[3], msg->end_pos[4], msg->end_pos[5]};

        Eigen::Isometry3d transform = solve::Xyzrpy2Isometry(end_pos);

        l5_pro_Interfaces_ptr_->setEndPose(transform);

        std::vector<double> joint_positions = {0, 0, 0, 0, 0, 0};

        for (int i = 0; i < 6; i++)
        {
            joint_positions[i] = msg->joint_pos[i];
        }

        l5_pro_Interfaces_ptr_->setJointPositions(joint_positions);

        l5_pro_Interfaces_ptr_->setArmStatus(msg->mode);

        l5_pro_Interfaces_ptr_->setCatch(msg->gripper);
    }

    void X5Controller::PubState()
    {

        auto message = arx_x5_msg::msg::RobotStatus();
        message.header.stamp = this->get_clock()->now();

        Eigen::Isometry3d transform = l5_pro_Interfaces_ptr_->getEndPose();

        // 创建长度为6的vector
        std::array<double, 6> result;

        std::vector<double> xyzrpy = {0, 0, 0, 0, 0, 0};
        xyzrpy = solve::Isometry2Xyzrpy(transform);

        // 填充vector
        result[0] = xyzrpy[0];
        result[1] = xyzrpy[1];
        result[2] = xyzrpy[2];
        result[3] = xyzrpy[3];
        result[4] = xyzrpy[4];
        result[5] = xyzrpy[5];

        message.end_pos = result;

        std::vector<double> joint_pos_vector = l5_pro_Interfaces_ptr_->getJointPositons();
        for (int i = 0; i <= 7; i++)
        {
            message.joint_pos[i] = joint_pos_vector[i];
        }

        std::vector<double> joint_velocities_vector = l5_pro_Interfaces_ptr_->getJointVelocities();
        for (int i = 0; i <= 7; i++)
        {
            message.joint_vel[i] = joint_velocities_vector[i];
        }

        std::vector<double> joint_current_vector = l5_pro_Interfaces_ptr_->getJointCurrent();
        for (int i = 0; i < 7; i++)
        {
            message.joint_cur[i] = joint_current_vector[i];
        }
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