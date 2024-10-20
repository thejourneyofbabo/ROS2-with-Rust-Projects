/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      vehicle_simulator_params.hpp
 * @brief     simulate vehicle
 * 
 * @date      2023-08-07 created by Yuseung Na (yuseungna@hanyang.ac.kr)
 */

#ifndef __VEHICLE_SIMULATOR_PARAMS_HPP__
#define __VEHICLE_SIMULATOR_PARAMS_HPP__
#pragma once

// STD Header
#include <string>
#include <cmath>

typedef struct {
    std::string vehicle_namespace{""};

    double init_x{0.0};
    double init_y{0.0};
    double init_yaw{0.0};
    double init_vel{0.0};
} VehicleSimulatorParams;

#endif // __VEHICLE_SIMULATOR_PARAMS_HPP__