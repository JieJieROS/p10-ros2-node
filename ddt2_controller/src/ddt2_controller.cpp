//
// Created by lsy on 24-3-22.
//

#include "ddt2_controller/ddt2_controller.h"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

#include "hardware_interface/loaned_command_interface.hpp"

namespace ddt2_controller
{
using hardware_interface::LoanedCommandInterface;
using hardware_interface::LoanedStateInterface;
DDT2Controller::DDT2Controller()
        : controller_interface::ControllerInterface(),
          rt_command_ptr_(nullptr),
          joints_command_subscriber_(nullptr)
{
}

controller_interface::return_type DDT2Controller::init(const std::string & controller_name)
{
    auto ret = ControllerInterface::init(controller_name);
    if (ret != controller_interface::return_type::OK)
        return ret;
//    RCLCPP_INFO_STREAM(rclcpp::get_logger("DDT"),"!!!!!!!!!!");
    try
    {
        auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
        auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
        auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);
    }
    catch (const std::exception & e)
    {
        fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
        return controller_interface::return_type::ERROR;
    }

    return controller_interface::return_type::OK;
}

CallbackReturn DDT2Controller::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
    joint_names_ = node_->get_parameter("joints").as_string_array();

    if (joint_names_.empty())
    {
        RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
        return CallbackReturn::ERROR;
    }

    // Specialized, child controllers set interfaces before calling configure function.
    if (command_interface_types_.empty() && state_interface_types_.empty() && command_mode_.empty())
    {
        command_interface_types_ = node_->get_parameter("command_interfaces").as_string_array();
        state_interface_types_ = node_->get_parameter("state_interfaces").as_string_array();
        command_mode_ = node_->get_parameter("command_mode").as_string();
        RCLCPP_INFO_STREAM(get_node()->get_logger(),command_mode_);
    }
    if (command_interface_types_.empty() || state_interface_types_.empty())
    {
        RCLCPP_ERROR(get_node()->get_logger(), "'command_interface' of 'state_interface' parameter was empty");
        return CallbackReturn::ERROR;
    }
    if (command_mode_ != "position" && command_mode_ != "velocity" && command_mode_ != "effort")
    {
        RCLCPP_ERROR(get_node()->get_logger(), "'command_mode' parameter was error check the .yaml");
        return CallbackReturn::ERROR;
    }
    pids_.resize((int)joint_names_.size());
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        // prefix should be interpreted as parameters prefix
        pids_[i] = std::make_shared<control_toolbox::PidROS>(get_node(), "gains." + joint_names_[i]);
        if (!pids_[i]->initPid())
            return CallbackReturn::FAILURE;
    }
    joints_command_subscriber_ = get_node()->create_subscription<CmdType>(
            "~/commands", rclcpp::SystemDefaultsQoS(),
            [this](const CmdType::SharedPtr msg) { rt_command_ptr_.writeFromNonRT(msg); });

    RCLCPP_INFO(get_node()->get_logger(), "configure successful");
    return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration DDT2Controller::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration command_interfaces_config;
//  Only use one command type so use INDIVIDUAL
    command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto & joint : joint_names_)
    {
//      TODO now the controller is only suppose effort control
        command_interfaces_config.names.push_back(joint + "/" + command_interface_types_[0]);
    }
    return command_interfaces_config;
}

controller_interface::InterfaceConfiguration DDT2Controller::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    state_interfaces_config.names.reserve(joint_names_.size() * state_interface_types_.size());
    for (const auto & joint : joint_names_)
    {
        for (const auto & interface_type : state_interface_types_)
            state_interfaces_config.names.push_back(joint + "/" + interface_type);
    }
    return state_interfaces_config;
}

// Fill ordered_interfaces with references to the matching interfaces
// in the same order as in joint_names
template <typename T>
bool get_ordered_interfaces(
        std::vector<T> & unordered_interfaces, const std::vector<std::string> & joint_names,
        const std::string & interface_type, std::vector<std::reference_wrapper<T>> & ordered_interfaces)
{
    for (const auto & joint_name : joint_names)
    {
        for (auto & interface : unordered_interfaces)
        {
            if ((interface.get_name() == joint_name) && (interface.get_interface_name() == interface_type))
            {
                ordered_interfaces.emplace_back(std::ref(interface));
            }
        }
    }

    return joint_names.size() == ordered_interfaces.size() || joint_names.size()*3 == ordered_interfaces.size() ;
}

CallbackReturn DDT2Controller::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
{
    //  check if we have all resources defined in the "points" parameter
    //  also verify that we *only* have the resources defined in the "points" parameter
    std::vector<std::reference_wrapper<LoanedCommandInterface>> command_ordered_interfaces;
    std::vector<std::reference_wrapper<LoanedStateInterface>> state_ordered_interfaces;
    for (const auto & interface : command_interface_types_)
    {
        if (!get_ordered_interfaces(command_interfaces_,joint_names_, interface, command_ordered_interfaces) ||
                command_interfaces_.size() != command_ordered_interfaces.size())
        {
            RCLCPP_ERROR(
                    node_->get_logger(), "Expected %zu '%s' command interfaces, got %zu.", joint_names_.size(),
                    interface.c_str(), command_interfaces_.size());
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }
    }

    // reset command buffer if a command came through callback when controller was inactive
    rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);

    return CallbackReturn::SUCCESS;
}

CallbackReturn DDT2Controller::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
{
    // reset command buffer
    for (auto index = 0ul; index < command_interfaces_.size(); ++index) {
        if (command_mode_ == "effort" || command_mode_ == "velocity")
            command_interfaces_[index].set_value(0.);
    }
    first_pos_initialized_ = false;
    rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);

    return CallbackReturn::SUCCESS;
}


// Except for update() and config get, everything else is almost the same
controller_interface::return_type DDT2Controller::update()
{
    if (last_time_.seconds() == 0 && last_time_.nanoseconds() == 0)
        last_time_ = rclcpp::Clock().now();
    auto joint_commands = rt_command_ptr_.readFromRT();

    std::vector<double> original_desired, output_commands;

    // no command received yet
    if (!joint_commands || !(*joint_commands))
    {
        original_desired.resize(joint_names_.size());
        output_commands.resize(joint_names_.size());
        for (auto index = 0ul; index < command_interfaces_.size(); ++index)
        {
            if (command_mode_ == "effort"){output_commands[index] = 0.;}
            else if (command_mode_ == "position"){
                double cur_pos;
                for (const auto & state_interface : state_interfaces_)
                {
                    if (state_interface.get_full_name().find("position") != std::string::npos
                        && state_interface.get_full_name().find(command_interfaces_[index].get_name()) != std::string::npos)
                    {
                        if (!first_pos_initialized_)
                        {
                            first_pos_ = state_interface.get_value();
                            first_pos_initialized_ = true;
                        }
                        cur_pos = state_interface.get_value();
                    }
                }
                double pos_err = first_pos_ - cur_pos;
//                RCLCPP_INFO_STREAM(rclcpp::get_logger("ERROR"), pos_err);
                rclcpp::Time cur_time = rclcpp::Clock().now();
                output_commands[index] = pids_[index]->computeCommand(pos_err,cur_time-last_time_);
            }
            else if (command_mode_ == "velocity"){
                double cur_vel;
                for (const auto & state_interface : state_interfaces_)
                {
                    if (state_interface.get_full_name().find("velocity") != std::string::npos &&
                        state_interface.get_full_name().find(command_interfaces_[index].get_name()) != std::string::npos)
                        cur_vel = state_interface.get_value();
                }

                double vel_err = 0. - cur_vel;
                rclcpp::Time cur_time = rclcpp::Clock().now();
                output_commands[index] = pids_[index]->computeCommand(vel_err,cur_time-last_time_);
            }
            command_interfaces_[index].set_value(output_commands[index]);
        }
        return controller_interface::return_type::OK;
    }

    if ((*joint_commands)->data.size() != command_interfaces_.size())
    {
        RCLCPP_ERROR_THROTTLE(
                get_node()->get_logger(), *node_->get_clock(), 1000,
                "command size (%zu) does not match number of interfaces (%zu)",
                (*joint_commands)->data.size(), command_interfaces_.size());
        return controller_interface::return_type::ERROR;
    }

    original_desired.resize((*joint_commands)->data.size());
    output_commands.resize((*joint_commands)->data.size());
    for (auto index = 0ul; index < command_interfaces_.size(); ++index)
    {
        original_desired[index] = ((*joint_commands)->data[index]);
        if (command_mode_ == "effort"){output_commands[index] = original_desired[index];}
        else if (command_mode_ == "position"){
            double cur_pos;
            for (const auto & state_interface : state_interfaces_)
            {
                if (state_interface.get_full_name().find("position") != std::string::npos
                && state_interface.get_full_name().find(command_interfaces_[index].get_name()) != std::string::npos)
                    cur_pos = state_interface.get_value();
            }
            double pos_err = original_desired[index] - cur_pos;
            rclcpp::Time cur_time = rclcpp::Clock().now();
            output_commands[index] = pids_[index]->computeCommand(pos_err,cur_time-last_time_);
        }
        else if (command_mode_ == "velocity"){
            double cur_vel;
            for (const auto & state_interface : state_interfaces_)
            {
                if (state_interface.get_full_name().find("velocity") != std::string::npos &&
                        state_interface.get_full_name().find(command_interfaces_[index].get_name()) != std::string::npos)
                {
                    cur_vel = state_interface.get_value();
                    RCLCPP_INFO_STREAM(get_node()->get_logger(),cur_vel);
                }
            }

            double vel_err = original_desired[index] - cur_vel;
            rclcpp::Time cur_time = rclcpp::Clock().now();
            output_commands[index] = pids_[index]->computeCommand(vel_err,cur_time-last_time_);
        }
        command_interfaces_[index].set_value(output_commands[index]);
    }
    last_time_ = rclcpp::Clock().now();
    return controller_interface::return_type::OK;
}

}  // namespace ddt2_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
        ddt2_controller::DDT2Controller, controller_interface::ControllerInterface)
