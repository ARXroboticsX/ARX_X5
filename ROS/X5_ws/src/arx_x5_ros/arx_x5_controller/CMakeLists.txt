cmake_minimum_required(VERSION 3.8)
project(arx_x5_controller)

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
add_executable(X5Controller src/X5Controller.cpp)
target_include_directories(X5Controller PUBLIC lib/arx_hardware_interface/include lib/arx_x5_src/include)
target_link_libraries(X5Controller ${catkin_LIBRARIES} ${CMAKE_CURRENT_SOURCE_DIR}/lib/arx_x5_src/libarx_x5_src.so ${CMAKE_CURRENT_SOURCE_DIR}/)
add_dependencies(X5Controller  ${catkin_EXPORTED_TARGETS})

add_executable(KeyBoard src/KeyBoard.cpp)
target_include_directories(KeyBoard PUBLIC lib/arx_hardware_interface/include lib/arx_x5_src/include)
target_link_libraries(KeyBoard ${catkin_LIBRARIES} ${CMAKE_CURRENT_SOURCE_DIR}/lib/arx_x5_src/libarx_x5_src.so ${CMAKE_CURRENT_SOURCE_DIR}/)
add_dependencies(KeyBoard  ${catkin_EXPORTED_TARGETS})

install(TARGETS
        X5Controller
        DESTINATION lib/${PROJECT_NAME})

install(FILES lib/arx_x5_src/libarx_x5_src.so 
        DESTINATION lib/
)
