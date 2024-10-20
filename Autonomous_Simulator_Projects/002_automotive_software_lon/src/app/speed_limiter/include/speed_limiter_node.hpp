/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      speed_limiter_node.hpp
 * @brief     provide limit speed to autonomous driving algorithm
 *            this package is only used in practice "Longitudinal Control"
 * 
 * @date      2024-10-14 created by Seongjae Jeong (sjeong99@hanyang.ac.kr)
 */

#ifndef __SPEED_LIMITER_HPP__
#define __SPEED_LIMITER_HPP__
#pragma once

// STD Header
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
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <ad_msgs/msg/vehicle_output.hpp>
#include <ad_msgs/msg/lane_point_data_array.hpp>
#include <rviz_2d_overlay_msgs/msg/overlay_text.hpp>

// Algorithm Header
#include "speed_limiter_algorithm.hpp"

// Parameter Header

class SpeedLimiter : public rclcpp::Node {
    public:
        SpeedLimiter(const std::string& node_name, const double& loop_rate,
                   const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        ~SpeedLimiter();

        void Init(const rclcpp::Time& current_time);
        void Run(const rclcpp::Time& current_time);
        void Publish(const rclcpp::Time& current_time);

    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Functions

        // Callback functions
        inline void CallbackEgoVehicleState(const ad_msgs::msg::VehicleOutput::SharedPtr msg) {
            mutex_ego_vehicle_state_.lock();
            i_ego_vehicle_state_.id = msg->id;
            i_ego_vehicle_state_.x = msg->x;
            i_ego_vehicle_state_.y = msg->y;
            i_ego_vehicle_state_.yaw = msg->yaw;
            i_ego_vehicle_state_.velocity = msg->velocity;
            i_ego_vehicle_state_.length = msg->length;
            i_ego_vehicle_state_.width = msg->width;

            param_is_simulator_on_ = true;
            mutex_ego_vehicle_state_.unlock();
        }

        // Algorithm functions
        void UpdateSpeedLimiterResult(const interface::VehicleState& ego_vehicle_state,
                                      const double& current_time);
        
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variables
        
        // Publisher
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr p_limit_speed_;
        rclcpp::Publisher<rviz_2d_overlay_msgs::msg::OverlayText>::SharedPtr p_text_speed_limit_result_;

        // Subscriber
        rclcpp::Subscription<ad_msgs::msg::VehicleOutput>::SharedPtr s_ego_vehicle_state_;

        // Timer
        rclcpp::TimerBase::SharedPtr t_run_node_;

        // Inputs
        VehicleState i_ego_vehicle_state_;

        // Mutex
        std::mutex mutex_ego_vehicle_state_;

        // Outputs
        std_msgs::msg::Float32 o_limit_speed_;
        rviz_2d_overlay_msgs::msg::OverlayText o_text_speed_limit_result_;

        // Algorithm
        std::unique_ptr<SpeedLimiterAlgorithm> ptr_speed_limit_algorithm_;

        // Configuration parameters

        // Time
        double time_start_ = 0.0;
        double time_prev_ = 0.0;
        double time_use_ = 0.0;
        double time_reach_ = 0.0;

        // Simulator        
        double prev_target_limit_speed_ = 0.0;
        bool param_is_simulator_on_ = false;
}; 

#endif // __SPEED_LIMITER_HPP__