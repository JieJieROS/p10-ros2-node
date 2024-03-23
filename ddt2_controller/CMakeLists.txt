cmake_minimum_required(VERSION 3.16)
project(ddt2_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=conversion -Werror=unused-but-set-variable
          -Werror=return-type -Werror=shadow -Werror=format)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
        forward_command_controller
        pluginlib
        rclcpp
        control_toolbox
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(ddt2_controller SHARED
        src/ddt2_controller.cpp
)
target_compile_features(ddt2_controller PUBLIC cxx_std_17)
target_include_directories(ddt2_controller PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/ddt2_controller>
)
ament_target_dependencies(ddt2_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(ddt2_controller PRIVATE "ddt2_controller_BUILDING_DLL")

pluginlib_export_plugin_description_file(controller_interface ddt2_controller_plugins.xml)

#if(BUILD_TESTING)
#  find_package(ament_cmake_gmock REQUIRED)
#  find_package(controller_manager REQUIRED)
#  find_package(ros2_control_test_assets REQUIRED)
#
#  ament_add_gmock(test_load_joint_group_effort_controller
#          test/test_load_joint_group_effort_controller.cpp
#  )
#  target_link_libraries(test_load_joint_group_effort_controller
#          ddt2_controller
#  )
#  ament_target_dependencies(test_load_joint_group_effort_controller
#          controller_manager
#          ros2_control_test_assets
#  )
#
#  ament_add_gmock(test_joint_group_effort_controller
#          test/test_joint_group_effort_controller.cpp
#  )
#  target_link_libraries(test_joint_group_effort_controller
#          ddt2_controller
#  )
#endif()

install(
        DIRECTORY include/
        DESTINATION include/ddt2_controller
)
install(
        TARGETS ddt2_controller
        EXPORT export_ddt2_controller
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

ament_export_targets(export_ddt2_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
