cmake_minimum_required(VERSION 3.5)
project(ddt2_hw)

set(FILE_NAME can_control_actuator)
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ddt2_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(yaml-cpp REQUIRED)
## COMPILE
add_library(
        ddt2_hw
        SHARED
        hardware/src/can_control_actuator.cpp
        hardware/src/common/tools/can_motor.cpp
        hardware/src/common/tools/lp_filter.cpp
        hardware/src/common/can_interface/can_bus.cpp
        hardware/src/common/can_interface/socketcan.cpp

)
target_include_directories(
        ddt2_hw
        PUBLIC
        hardware/include
)
ament_target_dependencies(
        ddt2_hw
        ddt2_msgs
        hardware_interface
        pluginlib
        rclcpp
        realtime_tools
        yaml-cpp
)

# Only change the .xml name
pluginlib_export_plugin_description_file(hardware_interface can_control_actuator.xml)

# INSTALL
install(
        TARGETS ddt2_hw
        DESTINATION lib
)
install(
        DIRECTORY hardware/include/
        DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_include_directories(hardware/include)
ament_export_libraries(ddt2_hw)
ament_export_dependencies(
        hardware_interface
        pluginlib
        rclcpp
        yaml-cpp
)
target_link_libraries(ddt2_hw yaml-cpp)

add_executable(low_pass_filter_tester
        hardware/src/common/tools/lp_filter.cpp
        test/test_lpfilter/test_lpfilter.cpp
)

target_link_libraries(low_pass_filter_tester ddt2_hw)

add_executable(test_yaml_cpp
        hardware/src/common/tools/lp_filter.cpp
        test/test_yaml_cpp/test_yaml_cpp.cpp
)

target_link_libraries(test_yaml_cpp ddt2_hw yaml-cpp)

ament_package()
