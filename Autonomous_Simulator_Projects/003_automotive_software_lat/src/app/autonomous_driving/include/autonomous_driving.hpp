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
#include <ad_msgs/msg/lane_point_data_array.hpp>
#include <ad_msgs/msg/lane_point_data.hpp>
#include <ad_msgs/msg/vehicle_input.hpp>
#include <ad_msgs/msg/vehicle_output.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

// Algorithm Header
#include <eigen3/Eigen/Dense>

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
        inline void CallbackLanePointsArray(const ad_msgs::msg::LanePointDataArray::SharedPtr msg) {            
            mutex_lane_points_.lock();
            i_lane_points_array_ = *msg;
            mutex_lane_points_.unlock();
        }
        
        // Algorithm functions
        /* You can write you own functions */

        
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variables

        // Publisher
        rclcpp::Publisher<ad_msgs::msg::VehicleInput>::SharedPtr p_vehicle_command_;
        rclcpp::Publisher<ad_msgs::msg::PolyfitLaneData>::SharedPtr p_driving_way_;
        rclcpp::Publisher<ad_msgs::msg::PolyfitLaneDataArray>::SharedPtr p_poly_lanes_;

        // Subscriber
        rclcpp::Subscription<ad_msgs::msg::VehicleInput>::SharedPtr s_manual_input_;
        rclcpp::Subscription<ad_msgs::msg::VehicleOutput>::SharedPtr s_vehicle_state_;
        rclcpp::Subscription<ad_msgs::msg::LanePointDataArray>::SharedPtr s_lane_points_array_;

        // Timer
        rclcpp::TimerBase::SharedPtr t_run_node_;
        
        // Inputs
        ad_msgs::msg::VehicleOutput i_vehicle_state_;
        ad_msgs::msg::LanePointDataArray i_lane_points_array_;

        // Mutex
        std::mutex mutex_manual_input_;
        std::mutex mutex_vehicle_state_;
        std::mutex mutex_lane_points_;        

        // Outputs
        ad_msgs::msg::VehicleInput o_vehicle_command_;
        ad_msgs::msg::PolyfitLaneData o_driving_way_;
        ad_msgs::msg::PolyfitLaneDataArray o_poly_lanes_;
        
        double time_prev_csv_lanes_;
        bool param_is_simulator_on_ = false;

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

        double param_m_ROIFront_param = 20.0;
        double param_m_ROIRear_param = 10.0;
        double param_m_ROILeft_param = 3.0;
        double param_m_ROIRight_param = 3.0;
        std::string param_ref_csv_path;

        // Algorhtm variables
        /* You can write you own variables */

};

#endif // __AUTONOMOUS_DRIVING_HPP__