#include "can_control_actuator/common/tools/can_motor.h"

namespace can_interface
{
bool CanMotor::init( std::string act_coeffs_yaml,  std::string act_datas_yaml, std::string bus_yaml)
{
    bool success = true;
    success &= parseActCoeffs(act_coeffs_yaml);
    success &= parseActData(act_datas_yaml);
    success &= initCanBus(bus_yaml);
    return success;
};
bool CanMotor::parseActCoeffs(std::string act_coeffs_yaml) {
    std::ifstream fin(act_coeffs_yaml);
    if (!fin) {
        std::cerr << "Failed to open YAML file." << std::endl;
        return -1;
    }
    YAML::Node config = YAML::LoadFile(act_coeffs_yaml);
    YAML::Node act_coeffs = config["actuator_coefficient"];

    for (auto it = act_coeffs.begin(); it != act_coeffs.end(); ++it) {
        std::string type = it->first.as<std::string>();
        YAML::Node params = it->second;

        ActCoeff coeff;
        coeff.act2pos = params["act2pos"] ? params["act2pos"].as<double>() : 0.0;
        coeff.act2vel = params["act2vel"] ? params["act2vel"].as<double>() : 0.0;
        coeff.act2effort = params["act2effort"] ? params["act2effort"].as<double>() : 0.0;
        coeff.effort2act = params["effort2act"] ? params["effort2act"].as<double>() : 0.0;
        coeff.max_out = params["max_out"] ? params["max_out"].as<double>() : 0.0;
        coeff.kp2act = params["kp2act"] ? params["kp2act"].as<double>() : 0.0;
        coeff.kd2act = params["kd2act"] ? params["kd2act"].as<double>() : 0.0;
        coeff.pos2act = params["pos2act"] ? params["pos2act"].as<double>() : 0.0;
        coeff.vel2act = params["vel2act"] ? params["vel2act"].as<double>() : 0.0;
        coeff.pos_offset = params["pos_offset"] ? params["pos_offset"].as<double>() : 0.0;
        coeff.vel_offset = params["vel_offset"] ? params["vel_offset"].as<double>() : 0.0;
        coeff.effort_offset = params["effort_offset"] ? params["effort_offset"].as<double>() : 0.0;


        if (type2act_coeffs_.find(type) == type2act_coeffs_.end()) {
            type2act_coeffs_.insert(std::make_pair(type, coeff));
            RCLCPP_INFO_STREAM(logger_, type2act_coeffs_.begin()->first);
        } else
            RCLCPP_INFO_STREAM(logger_, "Repeat actuator coefficient of type: " << type);
    }
    if (true) {
        for (const auto &pair: type2act_coeffs_) {
            std::cout << "Actuator Type: " << pair.first << std::endl;
            std::cout << "act2pos: " << pair.second.act2pos << std::endl;
            std::cout << "act2vel: " << pair.second.act2vel << std::endl;
            std::cout << "act2effort: " << pair.second.act2effort << std::endl;
            std::cout << "effort2act: " << pair.second.effort2act << std::endl;
            std::cout << "max_out: " << pair.second.max_out << std::endl;
            std::cout << "kp2act: " << pair.second.kp2act << std::endl;
            std::cout << "kd2act: " << pair.second.kd2act << std::endl;
            std::cout << "pos2act: " << pair.second.pos2act << std::endl;
            std::cout << "vel2act: " << pair.second.vel2act << std::endl;
            std::cout << "pos_offset: " << pair.second.pos_offset << std::endl;
            std::cout << "vel_offset: " << pair.second.vel_offset << std::endl;
            std::cout << "effort_offset: " << pair.second.effort_offset << std::endl;
            std::cout << std::endl;
        }
    }
    return true;
}

    bool CanMotor::parseActData(std::string act_datas_yaml) {
        std::ifstream fin(act_datas_yaml);
        if (!fin) {
            std::cerr << "Failed to open YAML file." << std::endl;
            return -1;
        }
        YAML::Node config = YAML::LoadFile(act_datas_yaml);
        YAML::Node act_datas = config["actuators"];

        for (auto it = act_datas.begin(); it != act_datas.end(); ++it) {
            RCLCPP_INFO_STREAM(logger_,it->first);
            const std::string &actuator_name = it->first.as<std::string>();
            const YAML::Node &actuator_data = it->second;

            if (!actuator_data["bus"] || !actuator_data["type"] || !actuator_data["id"]) {
                std::cerr << "Actuator " << actuator_name << " has incomplete data." << std::endl;
                continue;
            }

            std::string bus = actuator_data["bus"].as<std::string>();
            std::string type = actuator_data["type"].as<std::string>();
            int id = actuator_data["id"].as<int>();

            // Check for duplicate actuators
            if (bus_id2act_data_.find(bus) != bus_id2act_data_.end() &&
                bus_id2act_data_[bus].find(id) != bus_id2act_data_[bus].end()) {
                std::cerr << "Duplicate actuator on bus " << bus << " and ID " << id << std::endl;
                return false;
            }
            else
            {
                bus_id2act_data_[bus].insert(std::make_pair(id, can_interface::ActData{ .name = it->first.as<std::string>(),
                        .type = type,
                        .stamp = rclcpp::Clock().now(),
                        .seq = 0,
                        .halted = false,
                        .q_raw = 0,
                        .qd_raw = 0,
                        .temp = 0,
                        .q_circle = 0,
                        .q_last = 0,
                        .frequency = 0,
                        .pos = 0,
                        .vel = 0,
                        .effort = 0,
                        .cmd_pos = 0,
                        .cmd_vel = 0,
                        .cmd_effort = 0,
                        .exe_effort = 0,}));
            }
        }
        return true;
    }


    bool CanMotor::initCanBus(std::string bus_yaml) {
        // CAN Bus
        std::ifstream fin(bus_yaml);
        if (!fin) {
            std::cerr << "Failed to open YAML file." << std::endl;
            return -1;
        }
        int thread_priority = 95;
        YAML::Node config = YAML::LoadFile(bus_yaml);
        try {
            if (!config["bus"]) {
                RCLCPP_WARN(logger_, "No bus specified");
                return true;  // Consider returning false here if no bus specified means failure
            }
            YAML::Node bus_datas = config["bus"];
            const auto &buses = bus_datas;
            for (const auto &bus: buses) {
                std::string bus_name = bus.as<std::string>();
                if (bus_name.find("can") != std::string::npos) {
                    can_buses_.push_back(new can_interface::CanBus(bus_name, can_interface::CanDataPtr{
                        .type2act_coeffs_ = &type2act_coeffs_, .id2act_data_ = &bus_id2act_data_[bus_name]},
                                                                   thread_priority));
                } else {
                    RCLCPP_ERROR_STREAM(logger_, "Unknown bus: " << bus_name);
                }
            }
        } catch (const YAML::Exception &e) {
            RCLCPP_FATAL_STREAM(logger_, "YAML parsing error: " << e.what());
            return false;
        }
        return true;
    }

    void CanMotor::startMotor() {
        for (auto can_bus: can_buses_) {
            can_bus->start();
        }
    }

    void CanMotor::closeMotor() {
        for (auto can_bus: can_buses_) {
            can_bus->close();
        }
    }

    void CanMotor::testMotor() {
        for (auto can_bus: can_buses_) {
            can_bus->test();
        }
    }

    ActData* CanMotor::getActDataByName(const std::string& name, std::unordered_map<std::string, std::unordered_map<int, ActData>>& bus_id2act_data) {
        for ( auto& entry : bus_id2act_data) {
             auto& innerMap = entry.second;
            for ( auto& innerEntry : innerMap) {
                if (innerEntry.second.name == name) {
                    return &(innerEntry.second);
                }
            }
        }
        return nullptr; // Return nullptr if name is not found
    }
}  // namespace can_interface
