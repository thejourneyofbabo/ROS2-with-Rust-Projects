/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      speed_limiter_algorithm.hpp
 * @brief     provide limit speed to autonomous driving algorithm
 *            this package is only used in practice "Longitudinal Control"
 * 
 * @date      2024-10-14 created by Seongjae Jeong (sjeong99@hanyang.ac.kr)
 */

#ifndef __SPEED_LIMITER_ALGORITHM_HPP__
#define __SPEED_LIMITER_ALGORITHM_HPP__
#pragma once

// STD Header
#include <string>
#include <math.h>

// Interface Header
#include "interface_lane.hpp"
#include "interface_vehicle.hpp"

using namespace interface;

class SpeedLimiterAlgorithm {
    public:
        SpeedLimiterAlgorithm();
        ~SpeedLimiterAlgorithm();
        
    public:
        double SetSpeedLimits(const VehicleState& ego);

        std::string GetSpeedLimitInfo(const VehicleState& ego, 
                                      const double& time_reach,
                                      const double& time_use);
        

    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Functions
        
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variables

        // Outputs
        
        // Speed limits
        int target_limit_speed_index_ = 0;
        double target_limit_speed_ = 0.0;
        std::vector<double> limit_speed_vector_ms_;

        // Parameter
};


#endif // __SPEED_LIMITER_ALGORITHM_HPP__