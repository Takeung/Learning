cmake_minimum_required(VERSION 3.22)

project(first_node)

find_package(rclcpp REQUIRED)

# add_executable - 生成first_node可执行文件
add_executable(first_node first_ros2_node.cpp)

# target_link_libraries - 为first_node(目标) 添加需要动态链接库，相同于指定g++编译器-l参数
# 下面的语句代替 -lrclcpp -lrcutils
target_link_libraries(first_node rclcpp::rclcpp)

