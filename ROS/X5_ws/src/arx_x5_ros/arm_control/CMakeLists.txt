cmake_minimum_required(VERSION 3.8)
project(arm_control)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rosmsg
        rospy
        std_msgs
        message_generation
)

add_message_files(
        FILES
        PosCmd.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES demo_msg
   CATKIN_DEPENDS roscpp rosmsg rospy message_runtime
#  DEPENDS system_lib
)