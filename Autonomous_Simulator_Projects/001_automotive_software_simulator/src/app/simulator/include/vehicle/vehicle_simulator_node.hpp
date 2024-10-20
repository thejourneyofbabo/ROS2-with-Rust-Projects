/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      vehicle_simulator_node.hpp
 * @brief     simulate vehicle
 * 
 * @date      2018-11-16 created by Eunsan Jo (eunsan.mountain@gmail.com)
 *            2023-08-07 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : adapt new template
 *            2023-08-20 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : change to ROS2
 */

#ifndef __VEHICLE_SIMULATOR_HPP__
#define __VEHICLE_SIMULATOR_HPP__
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
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

// ROS Message Header
#include <ad_msgs/msg/vehicle_input.hpp>
#include <ad_msgs/msg/vehicle_output.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>

// Algorithm Header

// Parameter Header
#include "vehicle/vehicle_simulator_params.hpp"

#include "interface_vehicle.hpp"

using namespace interface;

class Vehicle : public rclcpp::Node {
    public:
        Vehicle(const std::string& node_name, const double& loop_rate,
                const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        ~Vehicle();

        void Init(const rclcpp::Time& current_time);
        void Run(const rclcpp::Time& current_time);
        void Publish(const rclcpp::Time& current_time);

    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Functions

        // Callback functions
        inline void CallbackVehicleCommand(const ad_msgs::msg::VehicleInput::SharedPtr msg) {
            mutex_vehicle_command_.lock();
            i_vehicle_command_.accel = std::max(0.0, std::min(1.0, msg->accel));
            i_vehicle_command_.brake = std::max(0.0, std::min(1.0, msg->brake));
            i_vehicle_command_.steering = std::max(-vehicle_param::max_steer, std::min(vehicle_param::max_steer, msg->steering));     
            b_is_vehicle_command_ = true;
            mutex_vehicle_command_.unlock();
        }

        // Algorithm functions
        interface::VehicleState SimVehicleLongitudinalModel(const interface::VehicleState& prev_vehicle_state,
                                                            const interface::VehicleCommand& vehicle_command);
        interface::VehicleState SimVehicleLateralModel(const interface::VehicleState& prev_vehicle_state,
                                                       const interface::VehicleCommand& vehicle_command);
        void BroadcastVehicleTF(const rclcpp::Time& current_time,
                                const interface::VehicleState& vehicle_state);

    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variables

        // Subscriber
        rclcpp::Subscription<ad_msgs::msg::VehicleInput>::SharedPtr s_vehicle_command_;

        // Input
        interface::VehicleCommand i_vehicle_command_;

        // Mutex
        std::mutex mutex_vehicle_command_;

        // Publisher
        rclcpp::Publisher<ad_msgs::msg::VehicleOutput>::SharedPtr p_vehicle_state_;

        // Output
        ad_msgs::msg::VehicleOutput o_vehicle_state_;
        
        // Tf
        tf2_ros::TransformBroadcaster tf2_broadcaster_;

        // Timer
        rclcpp::TimerBase::SharedPtr t_run_node_;
        
        // Algorithm

        // Util and Configuration
        VehicleSimulatorParams param_;
        
        // Flags
        bool b_is_vehicle_command_ = false;
        
        // Global Variables        
        double time_dt_ = 0.0;
        double time_prev_ = 0.0;        
        interface::VehicleState prev_vehicle_state_;
};

#endif // __VEHICLE_SIMULATOR_HPP__
