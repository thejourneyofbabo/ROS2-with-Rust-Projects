/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      lane_detection_algorithm.cpp
 * @brief     lane detection algorithm
 * 
 * @date      2023-08-07 created by Yuseung Na (yuseungna@hanyang.ac.kr)
 */

#include "lane_detection/lane_detection_algorithm.hpp"

LaneDetectionAlgorithm::LaneDetectionAlgorithm(const LaneDetectionParams& param) {
    param_ = param;
}

LaneDetectionAlgorithm::~LaneDetectionAlgorithm() {}

interface::Lanes LaneDetectionAlgorithm::LoadLanesData() {
    SLanes ref_csv_lane;
    ref_csv_lane.ImportLaneCsvFile(param_.ref_csv_path);

    o_ref_lanes_.frame_id = "world";    
    o_ref_lanes_.lane.clear();

    for (auto i_lane = 0; i_lane < ref_csv_lane.m_vecLanes.size(); i_lane++) {
        interface::Lane lane;
        lane.frame_id = "world";
        lane.id = std::to_string(ref_csv_lane.m_vecLanes[i_lane].m_nLaneID);

        for (auto i_point = 0; i_point < ref_csv_lane.m_vecLanes[i_lane].m_vecLanePoint.size(); i_point++) {
            interface::Point2D point;
            point.x = ref_csv_lane.m_vecLanes[i_lane].m_vecLanePoint[i_point].m_dPtX_m;
            point.y = ref_csv_lane.m_vecLanes[i_lane].m_vecLanePoint[i_point].m_dPtY_m;
            lane.point.push_back(point);
        }
        o_ref_lanes_.lane.push_back(lane);
    }

    return o_ref_lanes_;
}

interface::Lanes LaneDetectionAlgorithm::ExtractROI(const VehicleState& vehicle_state) {
    o_roi_lanes_.frame_id = param_.vehicle_namespace + "/body";
    o_roi_lanes_.id = o_ref_lanes_.id;
    o_roi_lanes_.lane.clear();

    for (const auto& ref_lane : o_ref_lanes_.lane) {
        interface::Lane lane;
        lane.frame_id = param_.vehicle_namespace + "/body";
        lane.id = ref_lane.id;

        int down_size = (ref_lane.point.size() + 3) / 4;

        for (int ref_point = 0; ref_point < down_size; ref_point++) {
            interface::Point2D ref_point_world;
            ref_point_world.x = ref_lane.point[ref_point * 4].x;
            ref_point_world.y = ref_lane.point[ref_point * 4].y;

            interface::Point2D ref_point_body
                = TfFromWorldToBody(ref_point_world, vehicle_state);
            
            if ((ref_point_body.x <= param_.m_ROIFront_param) &&
                (ref_point_body.x >= -1 * param_.m_ROIRear_param) &&
                (ref_point_body.y <= param_.m_ROILeft_param) &&
                (ref_point_body.y >= -1 * param_.m_ROIRight_param)) {
                lane.point.push_back(ref_point_body);
            }
        }

        if (lane.point.size() >= 2) {
            o_roi_lanes_.lane.push_back(lane);
        }
    }

    return o_roi_lanes_;
}

interface::Point2D LaneDetectionAlgorithm::TfFromWorldToBody(const interface::Point2D& point_world,
                                                             const VehicleState& vehicle_state) {
    interface::Point2D point_body;

    double x_translate = point_world.x - vehicle_state.x;
    double y_translate = point_world.y - vehicle_state.y;

    double x_rotate =  x_translate * cos(vehicle_state.yaw) + y_translate * sin(vehicle_state.yaw);
    double y_rotate = -x_translate * sin(vehicle_state.yaw) + y_translate * cos(vehicle_state.yaw);

    point_body.x = x_rotate;
    point_body.y = y_rotate;

    return point_body;
}