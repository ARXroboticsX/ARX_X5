#include "arx_x5_controller/X5Controller.hpp"

// using namespace std::chrono_literals;

namespace arx::x5
{
    X5Controller::X5Controller(ros::NodeHandle nh)
    {
        ROS_INFO("机械臂开始初始化...");
        std::string arm_control_type = nh.param("arm_control_type", std::string("normal"));
        std::string pub_topic_name = nh.param("arm_pub_topic_name", std::string("arm_status"));
        std::string sub_topic_name = nh.param("arm_sub_topic_name", std::string("arm_cmd"));
        std::string pub_topic_name_joint_old = nh.param("arm_pub_topic_name_joint_old", std::string("arm_status_old"));
        std::string pub_topic_name_ee_old = nh.param("arm_pub_topic_name_ee_old", std::string("arm_status_old"));

        interfaces_ptr_ = std::make_shared<InterfacesThread>(nh.param("arm_can_id", std::string("can0")), nh.param("arm_end_type", 0));

        if (arm_control_type == "normal")
        {

            ROS_INFO("常规模式启动");
            // 创建发布器
            joint_state_publisher_ = nh.advertise<arx5_arm_msg::RobotStatus>(pub_topic_name, 10);
            // // 创建订阅器
            joint_state_subscriber_ = nh.subscribe<arx5_arm_msg::RobotCmd>(
                sub_topic_name, 10, &X5Controller::CmdCallback, this);
            // 定时器，用于发布关节信息

            timer_ = nh.createTimer(ros::Duration(0.01), &X5Controller::PubState, this);
        }
        else if (arm_control_type == "normal_old")
        {
            ROS_INFO("常规模式启动[old_topic模式]");
            old_ee_pos_publisher_ = nh.advertise<arm_control::PosCmd>(pub_topic_name_ee_old, 10);
            pub_old_end_topic_ = true;
            
            old_joint_state_publisher_ = nh.advertise<arm_control::JointInfomation>(pub_topic_name_joint_old, 10);
            pub_old_joint_topic_ = true;

            // 创建发布器
            joint_state_publisher_ = nh.advertise<arx5_arm_msg::RobotStatus>(pub_topic_name, 10);
            // // 创建订阅器
            joint_state_subscriber_ = nh.subscribe<arm_control::JointControl>(
                sub_topic_name, 10, &X5Controller::OldCmdCallback, this);
            // 定时器，用于发布关节信息

            timer_ = nh.createTimer(ros::Duration(0.01), &X5Controller::PubState, this);
        }
        else if (arm_control_type == "vr_slave")
        {
            ROS_INFO("vr遥操作模式启动");
            joint_state_publisher_ = nh.advertise<arm_control::PosCmd>(pub_topic_name, 10);
            joint_state_subscriber_ = nh.subscribe<arm_control::PosCmd>(
                sub_topic_name, 10, &X5Controller::VrCmdCallback, this);
            timer_ = nh.createTimer(ros::Duration(0.01), &X5Controller::PubState, this);
        }
        else if (arm_control_type == "vr_slave_old")
        {
            ROS_INFO("vr遥操作模式启动[old_topic兼容模式]");
            old_joint_state_publisher_ = nh.advertise<arm_control::JointInfomation>(pub_topic_name_joint_old, 10);
            pub_old_joint_topic_ = true;

            old_ee_pos_publisher_ = nh.advertise<arm_control::PosCmd>(pub_topic_name_ee_old, 10);
            pub_old_end_topic_ = true;

            joint_state_publisher_ = nh.advertise<arm_control::PosCmd>(pub_topic_name, 10);
            joint_state_subscriber_ = nh.subscribe<arm_control::PosCmd>(
                sub_topic_name, 10, &X5Controller::VrCmdCallback, this);
            timer_ = nh.createTimer(ros::Duration(0.01), &X5Controller::PubState, this);
        }
        else if (arm_control_type == "aloha_master")
        {
            ROS_INFO("aloha主机模式启动");
            joint_state_publisher_ = nh.advertise<arx5_arm_msg::RobotStatus>(pub_topic_name, 10);
            interfaces_ptr_->setArmStatus(InterfacesThread::state::G_COMPENSATION);
            timer_ = nh.createTimer(ros::Duration(0.01), &X5Controller::PubState, this);
        }
        else if (arm_control_type == "aloha_slave")
        {
            ROS_INFO("aloha从机模式启动");
            joint_state_publisher_ = nh.advertise<arx5_arm_msg::RobotStatus>(pub_topic_name, 10);
            joint_state_subscriber_ = nh.subscribe<arx5_arm_msg::RobotStatus>(
                sub_topic_name, 10, &X5Controller::FollowCmdCallback, this);
            timer_ = nh.createTimer(ros::Duration(0.01), &X5Controller::PubState, this);
        }

        else if (arm_control_type == "aloha_master_old")
        {
            ROS_INFO("aloha主机模式启动[old_topic兼容模式]");
            old_ee_pos_publisher_ = nh.advertise<arm_control::PosCmd>(pub_topic_name_ee_old, 10);
            pub_old_end_topic_ = true;

            joint_state_publisher_ = nh.advertise<arx5_arm_msg::RobotStatus>(pub_topic_name, 10);
            interfaces_ptr_->setArmStatus(InterfacesThread::state::G_COMPENSATION);
            timer_ = nh.createTimer(ros::Duration(0.01), &X5Controller::PubState, this);
        }
        else if (arm_control_type == "aloha_slave_old")
        {
            ROS_INFO("aloha从机模式启动[old_topic兼容模式]");
            old_ee_pos_publisher_ = nh.advertise<arm_control::PosCmd>(pub_topic_name_ee_old, 10);
            pub_old_end_topic_ = true;

            old_joint_state_publisher_ = nh.advertise<arm_control::JointInfomation>(pub_topic_name_joint_old, 10);
            pub_old_joint_topic_ = true;

            joint_state_publisher_ = nh.advertise<arx5_arm_msg::RobotStatus>(pub_topic_name, 10);
            joint_state_subscriber_ = nh.subscribe<arx5_arm_msg::RobotStatus>(
                sub_topic_name, 10, &X5Controller::FollowCmdCallback, this);
            timer_ = nh.createTimer(ros::Duration(0.01), &X5Controller::PubState, this);
        }
    }

    void X5Controller::CmdCallback(const arx5_arm_msg::RobotCmd::ConstPtr &msg)
    {
        double end_pos[6] = {msg->end_pos[0], msg->end_pos[1], msg->end_pos[2], msg->end_pos[3], msg->end_pos[4], msg->end_pos[5]};

        Eigen::Isometry3d transform = solve::Xyzrpy2Isometry(end_pos);

        interfaces_ptr_->setEndPose(transform);

        std::vector<double> joint_positions = {0, 0, 0, 0, 0, 0};

        for (int i = 0; i < 6; i++)
        {
            joint_positions[i] = msg->joint_pos[i];
        }

        interfaces_ptr_->setJointPositions(joint_positions);

        interfaces_ptr_->setArmStatus(msg->mode);

        interfaces_ptr_->setCatch(msg->gripper);
    }

    void X5Controller::PubState(const ros::TimerEvent &)
    {
        arx5_arm_msg::RobotStatus msg;
        msg.header.stamp = ros::Time::now();

        Eigen::Isometry3d transform = interfaces_ptr_->getEndPose();

        /*
        // 提取四元数和位移
        Eigen::Quaterniond quat(transform.rotation());
        Eigen::Vector3d translation = transform.translation();

        // 创建长度为7的vector

        */

        // 填充vector
        boost::array<double, 6> result;

        std::vector<double> xyzrpy = {0, 0, 0, 0, 0, 0};
        xyzrpy = solve::Isometry2Xyzrpy(transform);

        result[0] = xyzrpy[0];
        result[1] = xyzrpy[1];
        result[2] = xyzrpy[2];
        result[3] = xyzrpy[3];
        result[4] = xyzrpy[4];
        result[5] = xyzrpy[5];

        msg.end_pos = result;

        std::vector<double> joint_pos_vector = interfaces_ptr_->getJointPositons();
        for (int i = 0; i < 7; i++)
        {
            msg.joint_pos[i] = joint_pos_vector[i];
        }

        std::vector<double> joint_velocities_vector = interfaces_ptr_->getJointVelocities();
        for (int i = 0; i < 7; i++)
        {
            msg.joint_vel[i] = joint_velocities_vector[i];
        }

        std::vector<double> joint_current_vector = interfaces_ptr_->getJointCurrent();
        for (int i = 0; i < 7; i++)
        {
            msg.joint_cur[i] = joint_current_vector[i];
        }

        // 发布消息
        ROS_INFO("Publishing RobotStatus message");
        joint_state_publisher_.publish(msg);

        if (pub_old_joint_topic_)
        {
            PubOldJointState(joint_pos_vector, joint_velocities_vector, joint_current_vector);
        }

        if (pub_old_end_topic_)
        {
            PubOldEndState(xyzrpy,joint_pos_vector[6]);
        }
    }

    void X5Controller::OldCmdCallback(const arm_control::JointControl::ConstPtr &msg)
    {
        std::vector<double> joint_positions = {0, 0, 0, 0, 0, 0};

        for (int i = 0; i < 6; i++)
        {
            joint_positions[i] = msg->joint_pos[i];
        }

        interfaces_ptr_->setJointPositions(joint_positions);
        interfaces_ptr_->setArmStatus(InterfacesThread::state::POSITION_CONTROL);
        interfaces_ptr_->setCatch(msg->joint_pos[6]);
    }

    void X5Controller::VrCmdCallback(const arm_control::PosCmd::ConstPtr &msg)
    {
        double input[6] = {msg->x, msg->y, msg->z, msg->roll, msg->pitch, msg->yaw};
        Eigen::Isometry3d transform = solve::Xyzrpy2Isometry(input);
        interfaces_ptr_->setEndPose(transform);
        interfaces_ptr_->setArmStatus(4);
        interfaces_ptr_->setCatch(msg->gripper);
    }

    void X5Controller::FollowCmdCallback(const arx5_arm_msg::RobotStatus::ConstPtr &msg)
    {
        std::vector<double> target_joint_position(6, 0.0);

        for (int i = 0; i < 6; i++)
        {
            target_joint_position[i] = msg->joint_pos[i];
        }

        interfaces_ptr_->setJointPositions(target_joint_position);
        interfaces_ptr_->setArmStatus(InterfacesThread::state::POSITION_CONTROL);

        interfaces_ptr_->setCatch(msg->joint_pos[6] * 5);
    }

    void X5Controller::PubOldJointState(std::vector<double> joint_pos_vector,
                                        std::vector<double> joint_velocities_vector,
                                        std::vector<double> joint_current_vector)
    {
        arm_control::JointInfomation msg;

        for (int i = 0; i < 7; i++)
        {
            msg.joint_pos[i] = joint_pos_vector[i];
        }
        for (int i = 0; i < 7; i++)
        {
            msg.joint_vel[i] = joint_velocities_vector[i];
        }
        for (int i = 0; i < 7; i++)
        {
            msg.joint_cur[i] = joint_current_vector[i];
        }

        old_joint_state_publisher_.publish(msg);
    }

    void X5Controller::PubOldEndState(std::vector<double> xyzrpy,double gripper)
    {
        arm_control::PosCmd msg_pos_cmd;
        msg_pos_cmd.x = xyzrpy[0];
        msg_pos_cmd.y = xyzrpy[1];
        msg_pos_cmd.z = xyzrpy[2];
        msg_pos_cmd.roll = xyzrpy[3];
        msg_pos_cmd.pitch = xyzrpy[4];
        msg_pos_cmd.yaw = xyzrpy[5];
        
        msg_pos_cmd.gripper = gripper;

        old_ee_pos_publisher_.publish(msg_pos_cmd);
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "r5_controller");
    ros::NodeHandle nh = ros::NodeHandle("~");
    arx::x5::X5Controller controller(nh);
    ros::spin();
    return 0;
}