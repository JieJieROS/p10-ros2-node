//
// Created by lsy on 24-3-22.
//

#pragma once

#include "visbility_control.h"
#include <control_toolbox/pid_ros.hpp>
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <memory>
#include <rclcpp/subscription.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <string>
#include <vector>
#include "ddt2_position_controller/socketcan.h"

namespace ddt2_position_controller {
    using CmdType = std_msgs::msg::Float64MultiArray;
    using CallbackReturn =
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class DDT2PositionController
            : public controller_interface::ControllerInterface {
    public:
        DDT2_CONTROLLER_CONTROLLER_PUBLIC
        DDT2PositionController();

        DDT2_CONTROLLER_CONTROLLER_PUBLIC
        controller_interface::return_type
        init(const std::string &controller_name) override;

        DDT2_CONTROLLER_CONTROLLER_PUBLIC
        controller_interface::InterfaceConfiguration
        command_interface_configuration() const override;

        DDT2_CONTROLLER_CONTROLLER_PUBLIC
        controller_interface::InterfaceConfiguration
        state_interface_configuration() const override;

        DDT2_CONTROLLER_CONTROLLER_PUBLIC
        CallbackReturn
        on_configure(const rclcpp_lifecycle::State &previous_state) override;

        DDT2_CONTROLLER_CONTROLLER_PUBLIC
        CallbackReturn
        on_activate(const rclcpp_lifecycle::State &previous_state) override;

        DDT2_CONTROLLER_CONTROLLER_PUBLIC
        CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        DDT2_CONTROLLER_CONTROLLER_PUBLIC
        controller_interface::return_type update() override;

        DDT2_CONTROLLER_CONTROLLER_PUBLIC
        void setLimitVelocity(const CmdType::SharedPtr msg);

    protected:
        std::vector<std::string> joint_names_;
        std::vector<std::string> command_interface_types_;
        std::vector<std::string> state_interface_types_;

        realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>> rt_command_ptr_;
        rclcpp::Subscription<CmdType>::SharedPtr joints_command_subscriber_, velocity_limits_subscriber_;

        std::string logger_name_;
        rclcpp::Time last_time_;
        std::vector<double> init_positions_;
        bool is_init_pos_ = false;
        can_interface::SocketCAN socket_can_;
    };

} // namespace ddt2_position_controller
