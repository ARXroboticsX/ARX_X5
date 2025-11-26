#include "arx_x5_controller/X5Controller.hpp"
#include <ros/package.h>

#include "ros/ros.h"
#include <chrono>
#include <thread>
// using namespace std::chrono_literals;

namespace arx::x5 {
X5Controller::X5Controller(ros::NodeHandle nh) {
  ROS_INFO("[%s] 机械臂开始初始化...", ros::this_node::getName().c_str());
  std::string arm_control_type =
      nh.param("arm_control_type", std::string("normal_v2"));

  std::string sub_topic_name =
      nh.param("sub_topic_name", std::string("/arm_cmd"));
  std::string pub_topic_ee_name_v1 =
      nh.param("pub_topic_ee_name_v1", std::string("/arm_status_ee"));
  std::string pub_topic_joint_name_v1 =
      nh.param("pub_topic_joint_name_v1", std::string("/arm_status_joint"));
  std::string pub_topic_name_v2 =
      nh.param("pub_topic_name_v2", std::string("/arm_status"));
  catch_control_mode_ =
      static_cast<CatchControlMode>(nh.param("catch_control_mode", 0));

  arm_end_type_ = nh.param("arm_end_type", 0);
  nh.getParam("go_home_position", go_home_positions);

  std::string package_path = ros::package::getPath("arx_x5_controller");
  std::string urdf_path;
  if (arm_end_type_ == 0)
    urdf_path = package_path + "/x5.urdf";
  else if (arm_end_type_ == 1)
    urdf_path = package_path + "/x5_master.urdf";
  else
    urdf_path = package_path + "/x5_2025.urdf";
  can_name_ = nh.param("arm_can_id", std::string("can0"));
  interfaces_ptr_ =
      std::make_shared<InterfacesThread>(urdf_path, can_name_, arm_end_type_);
  interfaces_ptr_->arx_x(150, 600, 10);
  interfaces_ptr_->setHomePositions(go_home_positions);
  if (arm_control_type == "normal_v1") {
    ROS_INFO("[%s] 常规模式启动[v1]", ros::this_node::getName().c_str());
    // sub
    joint_state_subscriber_ = nh.subscribe<arm_control::PosCmd>(
        sub_topic_name, 10, &X5Controller::CmdCallbackV1, this);

    // pub
    ee_pos_publisher_v1_ =
        nh.advertise<arm_control::PosCmd>(pub_topic_ee_name_v1, 10);
    joint_state_publisher_v1_ = nh.advertise<arm_control::JointInformation>(
        pub_topic_joint_name_v1, 10);
    pub_topic_v1_ = true;
    pub_topic_v2_ = false;
  } else if (arm_control_type == "normal_v2") {
    ROS_INFO("[%s] 常规模式启动[v2]", ros::this_node::getName().c_str());
    // sub
    joint_state_subscriber_ = nh.subscribe<arx5_arm_msg::RobotCmd>(
        sub_topic_name, 10, &X5Controller::CmdCallbackV2, this);
    // pub
    joint_state_publisher_ =
        nh.advertise<arx5_arm_msg::RobotStatus>(pub_topic_name_v2, 10);
    pub_topic_v1_ = false;
    pub_topic_v2_ = true;
  } else if (arm_control_type == "vr_slave_v1") {
    ROS_INFO("[%s] vr_slave启动[v1]", ros::this_node::getName().c_str());
    // sub
    joint_state_subscriber_ = nh.subscribe<arm_control::JointInformation>(
        sub_topic_name, 10, &X5Controller::FollowCallbackV1, this);

    // pub
    ee_pos_publisher_v1_ =
        nh.advertise<arm_control::PosCmd>(pub_topic_ee_name_v1, 10);
    joint_state_publisher_v1_ = nh.advertise<arm_control::JointInformation>(
        pub_topic_joint_name_v1, 10);
    pub_topic_v1_ = true;
    pub_topic_v2_ = false;
  } else if (arm_control_type == "remote_master_v1") {
    ROS_INFO("[%s] remote_master启动[v1]", ros::this_node::getName().c_str());
    interfaces_ptr_->setArmStatus(InterfacesThread::state::G_COMPENSATION);
    // pub
    ee_pos_publisher_v1_ =
        nh.advertise<arm_control::PosCmd>(pub_topic_ee_name_v1, 10);
    joint_state_publisher_v1_ = nh.advertise<arm_control::JointInformation>(
        pub_topic_joint_name_v1, 10);
    pub_topic_v1_ = true;
    pub_topic_v2_ = false;
  } else if (arm_control_type == "remote_slave_v1") {
    ROS_INFO("[%s] remote_slave启动[v1]", ros::this_node::getName().c_str());
    // sub
    joint_state_subscriber_ = nh.subscribe<arm_control::JointInformation>(
        sub_topic_name, 10, &X5Controller::FollowCallbackV1, this);

    // pub
    ee_pos_publisher_v1_ =
        nh.advertise<arm_control::PosCmd>(pub_topic_ee_name_v1, 10);
    joint_state_publisher_v1_ = nh.advertise<arm_control::JointInformation>(
        pub_topic_joint_name_v1, 10);
    pub_topic_v1_ = true;
    pub_topic_v2_ = false;
  } else if (arm_control_type == "joint_control_v1") {
    ROS_INFO("[%s] 常规模式启动[v1]", ros::this_node::getName().c_str());
    // sub
    joint_state_subscriber_ = nh.subscribe<arm_control::JointControl>(
        sub_topic_name, 10, &X5Controller::JointControlCallbackV1, this);

    // pub
    ee_pos_publisher_v1_ =
        nh.advertise<arm_control::PosCmd>(pub_topic_ee_name_v1, 10);
    joint_state_publisher_v1_ = nh.advertise<arm_control::JointInformation>(
        pub_topic_joint_name_v1, 10);
    pub_topic_v1_ = true;
    pub_topic_v2_ = false;
  }
  arx_joy_sub_ = nh.subscribe<std_msgs::Int32MultiArray>(
      "/arx_joy", 1, &X5Controller::arxJoyCB, this);

  timer_ = nh.createTimer(ros::Duration(0.01), &X5Controller::PubState, this);
}

X5Controller::~X5Controller() {
  interfaces_ptr_->setArmStatus(InterfacesThread::state::PROTECT);
}

void X5Controller::CmdCallbackV2(const arx5_arm_msg::RobotCmd::ConstPtr &msg) {
  double end_pos[6] = {msg->end_pos[0], msg->end_pos[1], msg->end_pos[2],
                       msg->end_pos[3], msg->end_pos[4], msg->end_pos[5]};

  Eigen::Isometry3d transform = solve::Xyzrpy2Isometry(end_pos);

  interfaces_ptr_->setEndPose(transform);

  std::vector<double> joint_positions = {0, 0, 0, 0, 0, 0};

  for (int i = 0; i < 6; i++) {
    joint_positions[i] = msg->joint_pos[i];
  }

  interfaces_ptr_->setJointPositions(joint_positions);

  interfaces_ptr_->setArmStatus(msg->mode);

  interfaces_ptr_->setCatch(msg->gripper);
}

void X5Controller::CmdCallbackV1(const arm_control::PosCmd::ConstPtr &msg) {
  double input[6] = {msg->x, msg->y, msg->z, msg->roll, msg->pitch, msg->yaw};
  Eigen::Isometry3d transform = solve::Xyzrpy2Isometry(input);
  interfaces_ptr_->setEndPose(transform);
  interfaces_ptr_->setArmStatus(InterfacesThread::state::END_CONTROL);
  double gripper = msg->gripper;
  if(arm_end_type_ == 2)
    gripper *= (-3.4 / 5);
  interfaces_ptr_->setCatch(gripper);
}

void X5Controller::FollowCallbackV2(
    const arx5_arm_msg::RobotStatus::ConstPtr &msg) {
  std::vector<double> target_joint_position(6, 0.0);

  for (int i = 0; i < 6; i++) {
    target_joint_position[i] = msg->joint_pos[i];
  }

  interfaces_ptr_->setJointPositions(target_joint_position);
  interfaces_ptr_->setArmStatus(InterfacesThread::state::POSITION_CONTROL);

  interfaces_ptr_->setCatch(msg->joint_pos[6]);
}

void X5Controller::FollowCallbackV1(
    const arm_control::JointInformation::ConstPtr &msg) {
  std::vector<double> target_joint_position(6, 0.0);

  for (int i = 0; i < 6; i++) {
    target_joint_position[i] = msg->joint_pos[i];
  }

  interfaces_ptr_->setJointPositions(target_joint_position);
  interfaces_ptr_->setArmStatus(InterfacesThread::state::POSITION_CONTROL);

  if (catch_control_mode_ == CatchControlMode::kPosition)
    interfaces_ptr_->setCatch(msg->joint_pos[6]);
  else
    interfaces_ptr_->setCatchTorque(msg->joint_cur[6]);
}

void X5Controller::JointControlCallbackV1(
    const arm_control::JointControl::ConstPtr &msg) {
  std::vector<double> target_joint_position(6, 0.0);

  for (int i = 0; i < 6; i++) {
    target_joint_position[i] = msg->joint_pos[i];
  }

  interfaces_ptr_->setJointPositions(target_joint_position);
  interfaces_ptr_->setArmStatus(InterfacesThread::state::POSITION_CONTROL);

  interfaces_ptr_->setCatch(msg->joint_pos[6]);
}

void X5Controller::arxJoyCB(const std_msgs::Int32MultiArray::ConstPtr &msg) {
  if (msg->data[0] == 1)
    interfaces_ptr_->setArmStatus(InterfacesThread::state::G_COMPENSATION);
  if (msg->data[1] == 1) {
    interfaces_ptr_->setArmStatus(InterfacesThread::state::GO_HOME);
  }
}

// Publisher
void X5Controller::PubState(const ros::TimerEvent &) {

  Eigen::Isometry3d transform = interfaces_ptr_->getEndPose();

  // 填充vector

  std::vector<double> xyzrpy = solve::Isometry2Xyzrpy(transform);

  std::vector<double> joint_pos_vector = interfaces_ptr_->getJointPositons();

  std::vector<double> joint_velocities_vector =
      interfaces_ptr_->getJointVelocities();

  std::vector<double> joint_current_vector = interfaces_ptr_->getJointCurrent();

  // 发布消息
  ROS_INFO("[%s] joint_pos:%f,%f,%f,%f,%f,%f,%f", ros::this_node::getName().c_str(), joint_pos_vector[0],
           joint_pos_vector[1], joint_pos_vector[2], joint_pos_vector[3],
           joint_pos_vector[4], joint_pos_vector[5], joint_pos_vector[6]);
  ROS_INFO("[%s] joint_vel:%f,%f,%f,%f,%f,%f,%f", ros::this_node::getName().c_str(), joint_velocities_vector[0],
           joint_velocities_vector[1], joint_velocities_vector[2],
           joint_velocities_vector[3], joint_velocities_vector[4],
           joint_velocities_vector[5], joint_velocities_vector[6]);
  ROS_INFO("[%s] joint_cur:%f,%f,%f,%f,%f,%f,%f", ros::this_node::getName().c_str(), joint_current_vector[0],
           joint_current_vector[1], joint_current_vector[2],
           joint_current_vector[3], joint_current_vector[4],
           joint_current_vector[5], joint_current_vector[6]);
  setlocale(LC_ALL, "");
  for (int code : interfaces_ptr_->getErrorCode()) {
    if (code == 1)
      ROS_ERROR("[%s] 关节力矩超限", ros::this_node::getName().c_str());
    else if (code == 2)
      ROS_ERROR("[%s] %s掉线", ros::this_node::getName().c_str(), can_name_.c_str());
    else if (code > 10) {
      int joint_id = code / 16;
      int error_code = code % 16;
      if (error_code == 15)
        ROS_ERROR("[%s] 电机离线！离线电机编号：%d", ros::this_node::getName().c_str(), joint_id);
      else
        ROS_ERROR("[%s] 电机返回故障，电机编号：%d,故障类型：%X", ros::this_node::getName().c_str(), joint_id,
                  error_code);
    }
  }

  if (pub_topic_v1_) {
    pubArmStatusV1(joint_pos_vector, joint_velocities_vector,
                   joint_current_vector, xyzrpy);
  }

  if (pub_topic_v2_) {
    pubArmStatusV2(joint_pos_vector, joint_velocities_vector,
                   joint_current_vector, xyzrpy);
  }
}

void X5Controller::pubArmStatusV1(std::vector<double> joint_pos_vector,
                                  std::vector<double> joint_velocities_vector,
                                  std::vector<double> joint_current_vector,
                                  std::vector<double> xyzrpy) {
  arm_control::JointInformation msg;

  for (int i = 0; i < 7; i++) {
    msg.joint_pos[i] = joint_pos_vector[i];
  }
  for (int i = 0; i < 7; i++) {
    msg.joint_vel[i] = joint_velocities_vector[i];
  }
  for (int i = 0; i < 7; i++) {
    msg.joint_cur[i] = joint_current_vector[i];
  }

  if (arm_end_type_ == 1) {
    msg.joint_pos[6] *= 5;
  }

  joint_state_publisher_v1_.publish(msg);

  arm_control::PosCmd msg_pos_cmd;

  msg_pos_cmd.x = xyzrpy[0];
  msg_pos_cmd.y = xyzrpy[1];
  msg_pos_cmd.z = xyzrpy[2];
  msg_pos_cmd.roll = xyzrpy[3];
  msg_pos_cmd.pitch = xyzrpy[4];
  msg_pos_cmd.yaw = xyzrpy[5];

  msg_pos_cmd.gripper = joint_pos_vector[6];

  ee_pos_publisher_v1_.publish(msg_pos_cmd);
}

void X5Controller::pubArmStatusV2(std::vector<double> joint_pos_vector,
                                  std::vector<double> joint_velocities_vector,
                                  std::vector<double> joint_current_vector,
                                  std::vector<double> xyzrpy) {
  arx5_arm_msg::RobotStatus msg;
  msg.header.stamp = ros::Time::now();

  boost::array<double, 6> result;
  for (int i = 0; i < 6; i++) {
    result[i] = xyzrpy[i];
  }

  msg.end_pos = result;

  for (int i = 0; i < 7; i++) {
    msg.joint_pos.push_back(joint_pos_vector[i]);
  }
  for (int i = 0; i < 7; i++) {
    msg.joint_vel.push_back(joint_velocities_vector[i]);
  }
  for (int i = 0; i < 7; i++) {
    msg.joint_cur.push_back(joint_current_vector[i]);
  }

  if (arm_end_type_ == 1) {
    msg.joint_pos[6] *= 5;
  }

  joint_state_publisher_.publish(msg);
}
} // namespace arx::x5

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "x5_controller");
  ros::NodeHandle nh = ros::NodeHandle("~");
  arx::x5::X5Controller controller(nh);
  ros::spin();
  return 0;
}
