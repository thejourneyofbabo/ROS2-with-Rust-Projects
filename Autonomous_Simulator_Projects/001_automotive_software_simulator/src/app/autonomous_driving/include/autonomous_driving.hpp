/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 * 
 * @file      autonomous_driving.hpp
 * @brief     autonomous driving algorithm 
 * 
 * @date      2018-11-20 created by Kichun Jo (kichunjo@hanyang.ac.kr)
 *            2023-08-07 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : adapt new template
 *            2023-08-20 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : change to ROS2
 */

#ifndef __AUTONOMOUS_DRIVING_HPP__
#define __AUTONOMOUS_DRIVING_HPP__
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
#include <ad_msgs/msg/polyfit_lane_data_array.hpp>
#include <ad_msgs/msg/vehicle_input.hpp>
#include <ad_msgs/msg/vehicle_output.hpp>
#include <std_msgs/msg/float32.hpp>

// Algorithm Header

// Parameter Header

class AutonomousDriving : public rclcpp::Node {
    public:
        AutonomousDriving(const std::string& node_name, const double& loop_rate,
                          const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        ~AutonomousDriving();

        void Init(const rclcpp::Time& current_time);
        void Run(const rclcpp::Time& current_time);
        void Publish(const rclcpp::Time& current_time);
        void UpdateParameter();

    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Functions     
        
        // Callback functions   
        inline void CallbackManualInput(const ad_msgs::msg::VehicleInput::SharedPtr msg) {            
            mutex_manual_input_.lock();
            if(param_use_manual_inputs_ == true) {
                o_vehicle_command_.accel = msg->accel;
                o_vehicle_command_.brake = msg->brake;
                o_vehicle_command_.steering = msg->steering;
            }
            mutex_manual_input_.unlock();
        }
        inline void CallbackVehicleState(const ad_msgs::msg::VehicleOutput::SharedPtr msg) {            
            mutex_vehicle_state_.lock();
            i_vehicle_state_ = *msg;
            mutex_vehicle_state_.unlock();
        }
        inline void CallbackLimitSpeed(const std_msgs::msg::Float32::SharedPtr msg) {            
            mutex_limit_speed_.lock();
            i_limit_speed_ = msg->data;
            mutex_limit_speed_.unlock();
        }
        inline void CallbackPolyLanes(const ad_msgs::msg::PolyfitLaneDataArray::SharedPtr msg) {            
            mutex_poly_lanes_.lock();
            i_poly_lanes_ = *msg;
            mutex_poly_lanes_.unlock();
        }
        
        // Algorithm functions
        ad_msgs::msg::PolyfitLaneData FindDrivingWay(const ad_msgs::msg::VehicleOutput& vehicle_state,
                                                     const ad_msgs::msg::PolyfitLaneDataArray& poly_lanes);
        void LateralControl(const ad_msgs::msg::VehicleOutput& vehicle_state,
                            const ad_msgs::msg::PolyfitLaneData& driving_way);                           
        void LongitudinalControl(const ad_msgs::msg::VehicleOutput& vehicle_state,
                                 const ad_msgs::msg::PolyfitLaneData& driving_way, 
                                 const double& limit_speed);               
        
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variables

        // Publisher
        rclcpp::Publisher<ad_msgs::msg::VehicleInput>::SharedPtr p_vehicle_command_;
        rclcpp::Publisher<ad_msgs::msg::PolyfitLaneData>::SharedPtr p_driving_way_;

        // Subscriber
        rclcpp::Subscription<ad_msgs::msg::VehicleInput>::SharedPtr s_manual_input_;
        rclcpp::Subscription<ad_msgs::msg::VehicleOutput>::SharedPtr s_vehicle_state_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr s_limit_speed_;
        rclcpp::Subscription<ad_msgs::msg::PolyfitLaneDataArray>::SharedPtr s_poly_lanes_;

        // Timer
        rclcpp::TimerBase::SharedPtr t_run_node_;
        
        // Inputs
        ad_msgs::msg::VehicleOutput i_vehicle_state_;
        double i_limit_speed_ = 0.0;
        ad_msgs::msg::PolyfitLaneDataArray i_poly_lanes_;

        // Mutex
        std::mutex mutex_manual_input_;
        std::mutex mutex_vehicle_state_;
        std::mutex mutex_limit_speed_;
        std::mutex mutex_poly_lanes_;        

        // Outputs
        ad_msgs::msg::VehicleInput o_vehicle_command_;
        ad_msgs::msg::PolyfitLaneData o_driving_way_;

        // Configuration parameters        
        std::string param_vehicle_namespace_;
        bool param_use_manual_inputs_ = false;
        const double param_wheel_base_ = 1.302 + 1.398; // L_f + L_r
        const double param_max_lateral_accel_ = 6200.0 / 1319.91; // Fyf_max / Mass

        // Tuning parameters
        double param_pp_kd_ = 1.0;
        double param_pp_kv_ = 0.0;
        double param_pp_kc_ = 0.0;
        double param_pid_kp_ = 0.0;
        double param_pid_ki_ = 0.0;
        double param_pid_kd_ = 0.0;
        double param_brake_ratio_ = 1.0;

        // Algorhtm variables
        double speed_error_integral_ = 0.0;
        double speed_error_prev_     = 0.0;

};

#endif // __AUTONOMOUS_DRIVING_HPP__