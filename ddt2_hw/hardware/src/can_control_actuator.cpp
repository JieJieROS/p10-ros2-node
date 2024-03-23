#include <memory>
#include <vector>

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/base_interface.hpp"

#include "can_control_actuator/can_control_actuator.h"
#include "can_control_actuator/common/tools/can_motor.h"
//using hardware_interface::ActuatorInterface;
//using hardware_interface::CommandInterface;
//using hardware_interface::return_type;
//using hardware_interface::StateInterface;

namespace ddt2_hw
{
hardware_interface::return_type CanControlActuator::configure(const hardware_interface::HardwareInfo & info)
{
    if (configure_default(info) != hardware_interface::return_type::OK)
    {
        return hardware_interface::return_type::ERROR;
    }
    hw_start_sec_ = stod(info_.hardware_parameters["hw_start_duration_sec"]);
    hw_stop_sec_ = stod(info_.hardware_parameters["hw_stop_duration_sec"]);
    hw_slowdown_ = stod(info_.hardware_parameters["hw_slowdown"]);
    control_level_.resize(info_.joints.size(), integration_level_t::EFFORT);

    for (const hardware_interface::ComponentInfo & joint : info_.joints)
    {
        if (joint.command_interfaces.size() != 1)
        {
            RCLCPP_FATAL(
                    rclcpp::get_logger("CanControlActuator"),
                    "Actuator '%s' has %d command interfaces. 1 expected.", joint.name.c_str());
            return hardware_interface::return_type::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_EFFORT)
        {
            RCLCPP_FATAL(
                    rclcpp::get_logger("CanControlActuator"),
                    "Joint '%s' has %s command interface. Expected %s", joint.name.c_str(),
                    joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_EFFORT);
            return hardware_interface::return_type::ERROR;
        }

        if (joint.state_interfaces.size() != 3)
        {
            RCLCPP_FATAL(
                    rclcpp::get_logger("CanControlActuator"),
                    "Joint '%s'has %d state interfaces. 3 expected.", joint.name.c_str());
            return hardware_interface::return_type::ERROR;
        }

        if (!(joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
              joint.state_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
              joint.state_interfaces[0].name == hardware_interface::HW_IF_EFFORT))
        {
            RCLCPP_FATAL(
                    rclcpp::get_logger("CanControlActuator"),
                    "Joint '%s' has %s state interface. Expected %s, %s, or %s.", joint.name.c_str(),
                    joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_EFFORT,
                    hardware_interface::HW_IF_POSITION,hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::return_type::ERROR;
        }
    }

    status_ = hardware_interface::status::CONFIGURED;

//  Add can interface relative
//  TODO: Add methes to simply the path
    std::string act_coeffs_path = "/home/lsy/ddt2_ws/src/ddt2_hw/hardware/config/actuator_coefficient.yaml";
    std::string act_datas_path = "/home/lsy/ddt2_ws/src/ddt2_hw/hardware/config/actuator.yaml";
    std::string bus_info_path = "/home/lsy/ddt2_ws/src/ddt2_hw/hardware/config/actuator.yaml";
    can_motor_.init(act_coeffs_path,act_datas_path,bus_info_path);

    return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> CanControlActuator::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (auto & joint : info_.joints)
    {
        can_interface::ActData *actData = can_motor_.getActDataByName(joint.name + "_motor",can_motor_.bus_id2act_data_);
        state_interfaces.emplace_back(
                joint.name, hardware_interface::HW_IF_EFFORT, &(actData->effort));
        state_interfaces.emplace_back(
                joint.name, hardware_interface::HW_IF_POSITION, &(actData->pos));
        state_interfaces.emplace_back(
                joint.name, hardware_interface::HW_IF_VELOCITY, &(actData->vel));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> CanControlActuator::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (auto & joint : info_.joints)
    {
        can_interface::ActData *actData = can_motor_.getActDataByName(joint.name + "_motor",can_motor_.bus_id2act_data_);
        command_interfaces.emplace_back(
                joint.name, hardware_interface::HW_IF_EFFORT, &(actData->exe_effort));
    }
//    RCLCPP_INFO_STREAM(rclcpp::get_logger("!!!!!!!!!!!!!!!!!!!!!"),command_interfaces.begin()->get_full_name());
    return command_interfaces;
}

hardware_interface::return_type CanControlActuator::start()
{
    can_motor_.startMotor();
    rclcpp::Node node("CanControlActuator");
    auto pub = node.create_publisher<ddt2_msgs::msg::ActuatorState>("/actuator_states", 100);
    actuator_state_pub_ = std::make_shared<realtime_tools::RealtimePublisher<ddt2_msgs::msg::ActuatorState>>(pub);
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type CanControlActuator::stop()
{
    can_motor_.closeMotor();
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type CanControlActuator::read()
{
    // no-op, state is getting propagated within write.
    rclcpp::Time curren = rclcpp::Clock().now();
    for (auto bus : can_motor_.can_buses_)
        bus->read(curren);
//    for (auto& id2act_datas : can_motor_.bus_id2act_data_)
//        for (auto& act_data : id2act_datas.second)
//        {
//            try
//            {  // Duration will be out of dual 32-bit range while motor failure
//                act_data.second.halted = (curren - act_data.second.stamp).seconds() > 0.1 || act_data.second.temp > 99;
//            }
//            catch (std::runtime_error& ex)
//            {
//            }
//            if (act_data.second.halted)
//            {
//                act_data.second.seq = 0;
//                act_data.second.vel = 0;
//                act_data.second.effort = 0;
//            }
//        }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type CanControlActuator::write()
{
    for (auto& bus : can_motor_.can_buses_)
        bus->write();

    rclcpp::Time curren = rclcpp::Clock().now();
    publishActuatorState(curren);
    return hardware_interface::return_type::OK;
}

void CanControlActuator::publishActuatorState(const rclcpp::Time& time)
{
    //TODO change the nano to sec
    if (last_publish_time_ + rclcpp::Duration(1.0 / 100.0) < time)
    {
        if (actuator_state_pub_->trylock())
        {
            ddt2_msgs::msg::ActuatorState actuator_state;
            for (const auto& id2act_datas : can_motor_.bus_id2act_data_)
                for (const auto& act_data : id2act_datas.second)
                {
                    actuator_state.stamp.push_back(act_data.second.stamp);
                    actuator_state.name.push_back(act_data.second.name);
                    actuator_state.type.push_back(act_data.second.type);
                    actuator_state.bus.push_back(id2act_datas.first);
                    actuator_state.id.push_back(act_data.first);
                    actuator_state.halted.push_back(act_data.second.halted);
                    actuator_state.need_calibration.push_back(act_data.second.need_calibration);
                    actuator_state.calibrated.push_back(act_data.second.calibrated);
                    actuator_state.calibration_reading.push_back(act_data.second.calibration_reading);
                    actuator_state.position_raw.push_back(act_data.second.q_raw);
                    actuator_state.velocity_raw.push_back(act_data.second.qd_raw);
                    actuator_state.temperature.push_back(act_data.second.temp);
                    actuator_state.circle.push_back(act_data.second.q_circle);
                    actuator_state.last_position_raw.push_back(act_data.second.q_last);
                    actuator_state.frequency.push_back(act_data.second.frequency);
                    actuator_state.position.push_back(act_data.second.pos);
                    actuator_state.velocity.push_back(act_data.second.vel);
                    actuator_state.effort.push_back(act_data.second.effort);
                    actuator_state.commanded_effort.push_back(act_data.second.cmd_effort);
                    actuator_state.executed_effort.push_back(act_data.second.exe_effort);
                }
            actuator_state_pub_->msg_ = actuator_state;
            actuator_state_pub_->unlockAndPublish();
            last_publish_time_ = time;
        }
    }
}


}  // namespace can_control_actuator

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(ddt2_hw::CanControlActuator, hardware_interface::SystemInterface)