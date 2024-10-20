/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      lane_detection_node.hpp
 * @brief     simulate lane detection
 * 
 * @date      2018-11-16 created by Eunsan Jo (eunsan.mountain@gmail.com)
 *            2023-08-07 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : adapt new template
 *            2023-08-20 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : change to ROS2
 */

#ifndef __LANE_DETECTION_NODE_HPP__
#define __LANE_DETECTION_NODE_HPP__
#pragma once

// STD Header
#include <cstdlib>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>
#include <string>
#include <cmath>
#include <chrono>

// ROS Header
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/message_info.hpp>

// ROS Message Header
#include <ad_msgs/msg/lane_point_data_array.hpp>
#include <ad_msgs/msg/polyfit_lane_data_array.hpp>
#include <ad_msgs/msg/vehicle_output.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

// Algorithm Header
#include "lane_detection/lane_detection_algorithm.hpp"
#include <eigen3/Eigen/Dense>

// Parameter Header

class LaneDetection : public rclcpp::Node {
    public:
        LaneDetection(const std::string& node_name, const double& loop_rate,
                      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        ~LaneDetection();

        void Init(const rclcpp::Time& current_time);
        void Run(const rclcpp::Time& current_time);
        void Publish(const rclcpp::Time& current_time);
    
    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Functions

        // Callback functions 
        inline void CallbackVehicleState(const ad_msgs::msg::VehicleOutput::SharedPtr msg) {
            mutex_vehicle_state_.lock();
            i_vehicle_state_.id = msg->id;
            i_vehicle_state_.x = msg->x;
            i_vehicle_state_.y = msg->y;
            i_vehicle_state_.yaw = msg->yaw;
            i_vehicle_state_.velocity = msg->velocity;
            i_vehicle_state_.length = msg->length;
            i_vehicle_state_.width = msg->width;

            param_is_simulator_on_ = true;
            mutex_vehicle_state_.unlock();
        }

        // Algorithm functions
        void PolyfitLane(const interface::Lanes& roi_lanes);
        void UpdateLanePointData(const interface::Lanes& roi_lanes);
    
    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variables
        
        // Publisher
        rclcpp::Publisher<ad_msgs::msg::LanePointDataArray>::SharedPtr p_csv_lanes_;
        rclcpp::Publisher<ad_msgs::msg::LanePointDataArray>::SharedPtr p_roi_lanes_;
        rclcpp::Publisher<ad_msgs::msg::PolyfitLaneDataArray>::SharedPtr p_poly_lanes_;
        
        // Subscriber
        rclcpp::Subscription<ad_msgs::msg::VehicleOutput>::SharedPtr s_vehicle_state_;

        // Timer
        rclcpp::TimerBase::SharedPtr t_run_node_;

        // Inputs
        interface::VehicleState i_vehicle_state_;
        interface::Lanes i_csv_lanes_;

        // Mutex
        std::mutex mutex_vehicle_state_;
        
        // Outputs
        ad_msgs::msg::LanePointDataArray o_csv_lanes_;
        ad_msgs::msg::LanePointDataArray o_roi_lanes_;
        ad_msgs::msg::PolyfitLaneDataArray o_poly_lanes_;
        
        // Algorithm
        std::unique_ptr<LaneDetectionAlgorithm> ptr_lane_detection_algorithm_;

        // Configuration parameters   
        LaneDetectionParams param_;
        
        double time_prev_csv_lanes_;
        bool param_is_simulator_on_ = false;

};

#endif // __LANE_DETECTION_NODE_HPP__