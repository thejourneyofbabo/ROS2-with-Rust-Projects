/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      evaluation_algorithm.cpp
 * @brief     autonomous driving algorithm evaluation tool
 * 
 * @date      2023-08-07 created by Yuseung Na (yuseungna@hanyang.ac.kr)
 */

#include "evaluation_algorithm.hpp"

EvaluationAlgorithm::EvaluationAlgorithm(const EvaluationParams& param) {
    param_ = param;
    
    param_limit_speed_ms_ = param_.speed_limit/3.6;
    param_master_target_speed_ms_ = 20.0;
}

EvaluationAlgorithm::~EvaluationAlgorithm() {}

void EvaluationAlgorithm::SetSpeedLimits() {
    param_master_target_speed_signs_.push_back(TargetSpeedPoint(
        param_.goal_x, param_.goal_y, 0.0)
    );
}

void EvaluationAlgorithm::LoadLaneData() {
    SLanes ref_csv_lane;
    ref_csv_lane.ImportLaneCsvFile(param_.ref_csv_path);

    o_ref_lane_.frame_id = "world";
    o_ref_lane_.point.clear();
    o_ref_lane_.id = param_.lane_id;

    for (auto i_lane = 0; i_lane < ref_csv_lane.m_vecLanes.size(); i_lane++) {
        if (param_.lane_id == std::to_string(ref_csv_lane.m_vecLanes[i_lane].m_nLaneID)) {
            for (auto i_point = 0;
                    i_point < ref_csv_lane.m_vecLanes[i_lane].m_vecLanePoint.size();
                    i_point++) {
                
                interface::Point2D point;
                point.x = ref_csv_lane.m_vecLanes[i_lane].m_vecLanePoint[i_point].m_dPtX_m;
                point.y = ref_csv_lane.m_vecLanes[i_lane].m_vecLanePoint[i_point].m_dPtY_m;

                o_ref_lane_.point.push_back(point);
            }
        }
    }
}

double EvaluationAlgorithm::SetLimitSpeedSigns(const VehicleState& ego) {    
    for (auto i = 0; i < param_limit_speed_signs_.size(); i++) {
        double dx = ego.x - param_limit_speed_signs_[i].x;
        double dy = ego.y - param_limit_speed_signs_[i].y;

        double distance_sq = dx * dx + dy * dy;
        if (distance_sq <= 5.0 * 5.0) {
            param_limit_speed_ms_ = param_limit_speed_signs_[i].targetSpeed;
        }
    }

    return param_limit_speed_ms_;
}

double EvaluationAlgorithm::SetMasterSpeedSigns(const VehicleState& master) {
    for (auto i = 0; i < param_master_target_speed_signs_.size(); i++) {
        double dx = master.x - param_master_target_speed_signs_[i].x;
        double dy = master.y - param_master_target_speed_signs_[i].y;

        double distance_sq = dx * dx + dy * dy;
        if (distance_sq <= 5.0 * 5.0) {
            param_master_target_speed_ms_ = param_master_target_speed_signs_[i].targetSpeed;
        }
    }

    return param_master_target_speed_ms_;
}

void EvaluationAlgorithm::CalcErrors(const VehicleState& ego, 
                                     const double& speed_gain, const double& driving_time,
                                     const bool& master_vehicle_collision, const double& dt) {
    CalcOverSpeedPenalty(ego, dt, speed_gain);
    CalcCrossTrackError(ego, dt);
    CheckFailure(master_vehicle_collision, driving_time);
}

void EvaluationAlgorithm::CalcOverSpeedPenalty(const VehicleState& ego, const double& dt, const double& gain) {
    if (param_limit_speed_ms_ > 0.1 && ego.velocity > param_limit_speed_ms_) {
        eval_speed_penalty_ += gain * dt *
                                (ego.velocity - param_limit_speed_ms_) / param_limit_speed_ms_;
        
    }
}

void EvaluationAlgorithm::CalcCrossTrackError(const VehicleState& ego, const double& dt) {
    double min1_distance_sq = std::numeric_limits<double>::max();
    double min2_distance_sq = std::numeric_limits<double>::max();
    interface::Point2D min1_point;
    interface::Point2D min2_point;
    for (auto i = 0; i < o_ref_lane_.point.size(); i++) {
        double dx = ego.x - o_ref_lane_.point[i].x;
        double dy = ego.y - o_ref_lane_.point[i].y;
        double distance_sq = dx * dx + dy * dy;

        if (distance_sq < min1_distance_sq) {
            min2_distance_sq = min1_distance_sq;
            min2_point = min1_point;

            min1_distance_sq = distance_sq;
            min1_point = o_ref_lane_.point[i];
        } else if (distance_sq < min2_distance_sq) {
            min2_distance_sq = distance_sq;
            min2_point = o_ref_lane_.point[i];
        }
    }

    double a = (min2_point.y - min1_point.y) / (min2_point.x - min1_point.x);
    double b = -1.0;
    double c = -1.0 * a * min2_point.x + min2_point.y;

    eval_curr_cte_ = (a * ego.x + b * ego.y + c) / (pow(a * a + b * b, 0.5));

    if (fabs(eval_curr_cte_) > eval_max_cte_) {
        eval_max_cte_ = fabs(eval_curr_cte_);
    }

    eval_LAD_cte_ += dt * fabs(eval_curr_cte_);
}

void EvaluationAlgorithm::CheckFailure(const bool& master_vehicle_collision,
                                       const double& driving_time) {
    // 1. Lane Departure
    if (fabs(eval_curr_cte_) > param_.eval_lane_departure){
        eval_is_lane_departure = true;
    }
    // 2. Collision
    if (master_vehicle_collision == true){
        eval_is_collision_ = true;
    }

    // 3. Time Limit
    if (driving_time > param_.eval_time_limit){
        eval_is_retire_ = true;
    }
}

bool EvaluationAlgorithm::IsFinished(const VehicleState& ego, const double& gain) {
    double dx = ego.x - param_.goal_x;
    double dy = ego.y - param_.goal_y;
    double distance_sq = dx * dx + dy * dy;

    if (distance_sq <= gain * gain) {
        return true;
    }
    else {
        return false;
    }
}

std::string EvaluationAlgorithm::GetEvaluationInfo(const VehicleState& ego, 
                                                   const double& driving_time) {

    std::ostringstream evaluation_info;

    evaluation_info         
        << "\n\n"
        << "///////////////////////////////////////////////////"
        << "\n"
        << "/////////////// [Evaluation Result] ///////////////"
        << "\n"
        << "1. Limit Speed:\t\t" << param_limit_speed_ms_ << " m/s"
        << "\n"
        << "   Vehicle Speed:\t" << ego.velocity << " m/s"
        << "\n"
        << "   ------------------------------------------------"
        << "\n"
        << "   Over Speed Penalty:\t" << eval_speed_penalty_ << " sec"
        << "\n"
        << "\n"
        << "==================================================="
        << "\n"
        << "2. Cross Track Error:\t\t" << eval_curr_cte_ << " m"
        << "\n"
        << "   Lane Departure Threshold:\t" << param_.eval_lane_departure << " m"
        << "\n"
        // << "   LAD Cross Track Error:\t" << eval_LAD_cte_ << " m"
        // << "\n"
        << "   Max Cross Track Error:\t" << eval_max_cte_ << " m"
        << "\n"
        << "\n"
        << "==================================================="
        // << "\n"
        // << "3. Spacing Error: " << eval_curr_spacing_error_ << " m"
        // << "\n"
        // << "   LAD Spacing Error: " << eval_LAD_spacing_error_ << " m"
        // << "\n"
        // << "   Collision: " << (eval_is_collision_ ? "TRUE" : "FALSE") << "\n"
        // << "\n"
        // << "==================================================="
        << "\n"
        << "3. Driving Time:\t" << driving_time << " sec"
        << "\n"
        << "   Time Limit:\t\t" << param_.eval_time_limit << " sec"
        << "\n"
        << "\n"
        << "==================================================="
        << "\n"
        << "4. Failure:"
        << "\n"
        << "   Time Limit:\t\t" << (eval_is_retire_ ? "FAIL" : "SUCCESS") << "\n"
        << "   Lane Departure:\t" << (eval_is_lane_departure ? "FAIL" : "SUCCESS") << "\n"
        << "   Collision:\t\t" << (eval_is_collision_ ? "FAIL" : "SUCCESS") << "\n"
        << "\n"
        << "\n"
        << "------------------- [Result] ----------------------"
        << "\n"
        << "Time Score:\t" << (driving_time + eval_speed_penalty_)
        << "\n" 
        << "Failure:\t" << ((eval_is_retire_
                           + eval_is_lane_departure
                           + eval_is_collision_) ? "FAIL" : "SUCCESS")
        << "\n"                            
        << "///////////////////////////////////////////////////";

    return evaluation_info.str();
}