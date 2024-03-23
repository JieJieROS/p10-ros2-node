cmake_minimum_required(VERSION 3.5)
project(ddt2_assets)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ddt2_hw REQUIRED)
find_package(ddt2_msgs REQUIRED)
find_package(realtime_tools REQUIRED)


install(
        DIRECTORY
        one_actuator/config
        one_actuator/launch
        one_actuator/description/ros2_control
        one_actuator/description/urdf
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
