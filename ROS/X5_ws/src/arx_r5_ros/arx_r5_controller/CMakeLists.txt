cmake_minimum_required(VERSION 3.8)
project(arx_r5_controller)

find_package(catkin REQUIRED COMPONENTS
        arm_control
        arx5_arm_msg
        roscpp
        std_msgs
        message_generation
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS arm_control arx5_arm_msg roscpp std_msgs
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

# 添加共享库路径
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)

# 添加可执行文件
add_executable(R5Controller src/R5Controller.cpp)
target_include_directories(R5Controller PUBLIC lib/arx_hardware_interface/include lib/arx_r5_src/include)
target_link_libraries(R5Controller ${catkin_LIBRARIES} ${CMAKE_CURRENT_SOURCE_DIR}/lib/arx_r5_src/libarx_r5_src.so ${CMAKE_CURRENT_SOURCE_DIR}/lib/arx_hardware_interface/libarx_hardware_interface.so)
add_dependencies(R5Controller  ${catkin_EXPORTED_TARGETS})

add_executable(R5KeyBoard src/KeyBoard.cpp)
target_include_directories(R5KeyBoard PUBLIC lib/arx_hardware_interface/include lib/arx_r5_src/include)
target_link_libraries(R5KeyBoard ${catkin_LIBRARIES} ${CMAKE_CURRENT_SOURCE_DIR}/lib/arx_r5_src/libarx_r5_src.so ${CMAKE_CURRENT_SOURCE_DIR}/lib/arx_hardware_interface/libarx_hardware_interface.so)
add_dependencies(R5KeyBoard  ${catkin_EXPORTED_TARGETS})

install(TARGETS
        R5Controller
        R5KeyBoard
        DESTINATION lib/${PROJECT_NAME})

install(FILES lib/arx_r5_src/libarx_r5_src.so lib/arx_hardware_interface/libarx_hardware_interface.so
        DESTINATION lib/
)
