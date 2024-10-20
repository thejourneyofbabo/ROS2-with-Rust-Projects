/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      evaluation_algorithm.cpp
 * @brief     autonomous driving algorithm evaluation tool
 * 
 * @date      2023-08-07 created by Yuseung Na (yuseungna@hanyang.ac.kr)
 */

#ifndef __EVALUATION_ALGORITHM_HPP__
#define __EVALUATION_ALGORITHM_HPP__
#pragma once

// STD Header
#include <string>
#include <math.h>

// Interface Header
#include "interface_lane.hpp"
#include "interface_vehicle.hpp"

using namespace interface;

typedef struct {
    double goal_x;
    double goal_y;

    std::string ref_csv_path;
    std::string lane_id;

    double eval_time_limit;
    double speed_limit;
    double eval_lane_departure;

} EvaluationParams;

class TargetSpeedPoint {
    public:
        double x = 0.0;
        double y = 0.0;
        double targetSpeed = 0.0;

    public:
        TargetSpeedPoint() {}
        TargetSpeedPoint(double x, double y, double targetSpeed) {
            this->x = x;
            this->y = y;
            this->targetSpeed = targetSpeed/3.6;
        }
        ~TargetSpeedPoint() {}
};

class EvaluationAlgorithm {
    public:
        EvaluationAlgorithm(const EvaluationParams& params);
        ~EvaluationAlgorithm();
        
    public:
        void SetSpeedLimits();
        void LoadLaneData();

        double SetLimitSpeedSigns(const VehicleState& ego);
        double SetMasterSpeedSigns(const VehicleState& master);

        void CalcErrors(const VehicleState& ego,
                        const double& speed_gain, const double& driving_time,
                        const bool& master_vehicle_collision, const double& dt);

        std::string GetEvaluationInfo(const VehicleState& ego, 
                                      const double& driving_time);
        
        bool IsFinished(const VehicleState& ego, const double& gain);

        inline double GetSpeedPenalty() { return eval_speed_penalty_; }
        inline double GetCrossTrackError() { return eval_curr_cte_; }
        inline double GetLADCrossTrackError() { return eval_LAD_cte_; }
        inline double GetMaxCrossTrackError() { return eval_max_cte_; }
        inline bool IsCollision() { return eval_is_collision_; }
        inline bool IsLaneDeparture() { return eval_is_lane_departure; }
        inline bool IsRetire() { return eval_is_retire_; }

    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Functions
        void CalcOverSpeedPenalty(const VehicleState& ego, const double& dt, const double& gain);
        void CalcCrossTrackError(const VehicleState& ego, const double& dt);
        void CheckFailure(const bool& master_vehicle_collision, const double& driving_time);
        
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variables

        // Outputs
        interface::Lane o_ref_lane_;
        
        // Speed limits
        double param_limit_speed_ms_;
        std::vector<TargetSpeedPoint> param_limit_speed_signs_;

        // Master vehicle
        double param_master_target_speed_ms_;
        std::vector<TargetSpeedPoint> param_master_target_speed_signs_;
        
        // Evaluation results
        double eval_speed_penalty_ = 0.0;

        double eval_curr_cte_ = 0.0;
        double eval_LAD_cte_ = 0.0;
        double eval_max_cte_ = 0.0;

        double eval_curr_spacing_error_ = 0.0;
        double eval_LAD_spacing_error_ = 0.0;

        bool eval_is_lane_departure = false;
        bool eval_is_collision_ = false;
        bool eval_is_retire_ = false;

        EvaluationParams param_;
};


#endif // __EVALUATION_ALGORITHM_HPP__