#pragma once

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/macros.hpp"
#include "realtime_tools/realtime_publisher.h"
#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"

#include "can_control_actuator/visbility_control.h"
#include "can_control_actuator/common/tools/can_motor.h"

#include "ddt2_msgs/msg/actuator_state.hpp"

namespace ddt2_hw
{
class CanControlActuator : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
    DDT2_HW_PUBLIC
    hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;

    DDT2_HW_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    DDT2_HW_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    DDT2_HW_PUBLIC
    hardware_interface::return_type start() override;

    DDT2_HW_PUBLIC
    hardware_interface::return_type stop() override;

    DDT2_HW_PUBLIC
    std::string get_actuator_name() const  { return actuator_name_; }

    DDT2_HW_PUBLIC
    std::string get_actuator_type() const { return actuator_type_; }

    DDT2_HW_PUBLIC
    hardware_interface::return_type read() override;

    DDT2_HW_PUBLIC
    hardware_interface::return_type write() override;

    DDT2_HW_PUBLIC
    void publishActuatorState(const rclcpp::Time& time);
private:
    double hw_start_sec_;
    double hw_stop_sec_;
    double hw_slowdown_;

    std::string actuator_name_;
    std::string actuator_type_;
    can_interface::CanMotor can_motor_;

    enum class integration_level_t : std::uint8_t
    {
        UNDEFINED = 0,
        POSITION = 1,
        VELOCITY = 2,
        ACCELERATION = 3,
        EFFORT = 4
    };

    // Active control mode for each actuator
    std::vector<integration_level_t> control_level_;

    rclcpp::Time last_publish_time_;
    std::shared_ptr<realtime_tools::RealtimePublisher<ddt2_msgs::msg::ActuatorState>> actuator_state_pub_;
};

}  // namespace can_control_actuator
