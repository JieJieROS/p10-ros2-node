/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

//
// Created by qiayuan on 5/16/21.
//
#pragma once

#include "can_control_actuator/common/can_interface/can_bus.h"
#include "yaml-cpp/yaml.h"
#include "fstream"

namespace can_interface
{
    class CanMotor
    {
    public:
        CanMotor():logger_(rclcpp::get_logger("CanMotor"))
        {
        }
        bool init( std::string act_coeffs_yaml,  std::string act_datas_yaml, std::string bus_yaml);

        bool parseActCoeffs( std::string act_coeffs_yaml);

        bool parseActData(std::string act_datas_yaml);

        bool initCanBus(std::string bus_yaml);

        void startMotor();
        void closeMotor();
        void testMotor();

        ActData* getActDataByName(const std::string& name, std::unordered_map<std::string, std::unordered_map<int, ActData>>& bus_id2act_data);
    public:
        rclcpp::Logger logger_;
        // can interface
        std::vector<CanBus*> can_buses_{};
        // motor param
        std::unordered_map<std::string, ActCoeff> type2act_coeffs_{};
        std::unordered_map<std::string, std::unordered_map<int, ActData>> bus_id2act_data_{};
    };

}  // namespace can_interface