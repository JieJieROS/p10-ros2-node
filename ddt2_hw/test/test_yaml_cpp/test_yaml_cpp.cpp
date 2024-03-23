//
// Created by lsy on 24-3-21.
//
#include "can_control_actuator/common/tools/can_motor.h"

int main()
{
    can_interface::CanMotor canMotor;
    canMotor.parseActCoeffs("/home/lsy/ddt2_ws/src/ddt2_hw/hardware/test/test_yaml_cpp/test_yaml.yaml");
    canMotor.parseActData("/home/lsy/ddt2_ws/src/ddt2_hw/hardware/test/test_yaml_cpp/test_bus_act_yaml.yaml");
    canMotor.initCanBus("/home/lsy/ddt2_ws/src/ddt2_hw/hardware/test/test_yaml_cpp/test_bus_act_yaml.yaml");
    canMotor.startMotor();
    can_interface::ActData testFunction;
    testFunction = *canMotor.getActDataByName("joint1_motor", canMotor.bus_id2act_data_);
    std::cout << testFunction.type << std::endl;
//    while (1)
//        canMotor.testMotor();
}