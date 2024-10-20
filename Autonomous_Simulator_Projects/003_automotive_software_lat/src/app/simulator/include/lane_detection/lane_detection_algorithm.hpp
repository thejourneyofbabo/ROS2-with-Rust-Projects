/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      lane_detection_algorithm.hpp
 * @brief     lane detection algorithm
 * 
 * @date      2023-08-07 created by Yuseung Na (yuseungna@hanyang.ac.kr)
 */

#ifndef __LANE_DETECTION_ALGORITHM_HPP__
#define __LANE_DETECTION_ALGORITHM_HPP__
#pragma once

// STD Header
#include <cmath>
#include <random>

// Interface Header
#include "interface_vehicle.hpp"
#include "interface_lane.hpp"

using namespace interface;

typedef struct {
    std::string vehicle_namespace;
    std::string ref_csv_path;

    double m_ROIFront_param{20.0};
    double m_ROIRear_param{10.0};
    double m_ROILeft_param{3.0};
    double m_ROIRight_param{3.0};

} LaneDetectionParams;

class LaneDetectionAlgorithm {
    public:
        LaneDetectionAlgorithm(const LaneDetectionParams& param);
        ~LaneDetectionAlgorithm();
    
    public:
        interface::Lanes LoadLanesData();
        interface::Lanes ExtractROI(const VehicleState& vehicle_state);

    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Functions
        interface::Point2D TfFromWorldToBody(const interface::Point2D& point_world, 
                                             const VehicleState& vehicle_state);
        
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variables
        
        // Outputs
        interface::Lanes o_ref_lanes_;
        interface::Lanes o_roi_lanes_;
        
        LaneDetectionParams param_;
};

#endif // __LANE_DETECTION_ALGORITHM_HPP__