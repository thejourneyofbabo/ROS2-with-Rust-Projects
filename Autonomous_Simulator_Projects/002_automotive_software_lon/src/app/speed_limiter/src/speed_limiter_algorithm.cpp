/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      speed_limiter_algorithm.cpp
 * @brief     provide limit speed to autonomous driving algorithm
 *            this package is only used in practice "Longitudinal Control"
 * 
 * @date      2024-10-14 created by Seongjae Jeong (sjeong99@hanyang.ac.kr)
 */

#include "speed_limiter_algorithm.hpp"

SpeedLimiterAlgorithm::SpeedLimiterAlgorithm() {
    limit_speed_vector_ms_.push_back(30.0 / 3.6);
    limit_speed_vector_ms_.push_back(60.0 / 3.6);
    limit_speed_vector_ms_.push_back(90.0 / 3.6);
    limit_speed_vector_ms_.push_back(70.0 / 3.6);
    limit_speed_vector_ms_.push_back(50.0 / 3.6);
    limit_speed_vector_ms_.push_back(30.0 / 3.6);
    limit_speed_vector_ms_.push_back(100.0 / 3.6);
    limit_speed_vector_ms_.push_back(50.0 / 3.6);
    limit_speed_vector_ms_.push_back(100.0 / 3.6);
    

    target_limit_speed_ = limit_speed_vector_ms_.front();
    target_limit_speed_index_ = 0;
}

SpeedLimiterAlgorithm::~SpeedLimiterAlgorithm() {}

double SpeedLimiterAlgorithm::SetSpeedLimits(const VehicleState& ego) {
    double speed_difference = fabs(target_limit_speed_ - ego.velocity);

    if(speed_difference < 1.0 / 3.6) {
        target_limit_speed_index_ += 1;
        
        if(target_limit_speed_index_ >= limit_speed_vector_ms_.size()) {
            target_limit_speed_index_ = 0;
        }
    }

    target_limit_speed_ = limit_speed_vector_ms_[target_limit_speed_index_];

    return target_limit_speed_;
}

std::string SpeedLimiterAlgorithm::GetSpeedLimitInfo(const VehicleState& ego, 
                                                     const double& time_reach,
                                                     const double& time_use) {

    std::ostringstream speedlimit_info;

    speedlimit_info         
        << "\n\n"
        << "/////////////// [Speed Limit Result] ///////////////"
        << "\n"
        << "   Limit Speed:\t\t" << target_limit_speed_ << " m/s"
        << "\n"
        << "   Vehicle Speed:\t" << ego.velocity << " m/s"
        << "\n"
        << "   Time consumed to reach target speed:\t" << time_use << " s"
        << "\n"
        << "   Previous time consumed:\t" << time_reach << " s"
        << "\n"
        << "///////////////////////////////////////////////////";

    return speedlimit_info.str();
}