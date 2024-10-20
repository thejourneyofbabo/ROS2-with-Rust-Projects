/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      display_node.hpp
 * @brief     display topics using markers
 * 
 * @date      2018-11-16 created by Eunsan Jo (eunsan.mountain@gmail.com)
 *            2023-08-07 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : adapt new template
 *            2023-08-20 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : change to ROS2
 */

#ifndef __DISPLAY_NODE_HPP__
#define __DISPLAY_NODE_HPP__
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
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

// ROS Message Header
#include <ad_msgs/msg/vehicle_output.hpp>
#include <ad_msgs/msg/lane_point_data_array.hpp>
#include <ad_msgs/msg/polyfit_lane_data_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/float32.hpp>

// Algorithm Header

// Parameter Header

class Display : public rclcpp::Node {
    public:
        Display(const std::string& node_name, const double& loop_rate,
                const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        ~Display();

        void Init(const rclcpp::Time& current_time);
        void Run(const rclcpp::Time& current_time);

    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Functions

        // Callback functions
        inline void CallbackVehicleState(const ad_msgs::msg::VehicleOutput::SharedPtr msg) {
            i_vehicle_state_ = *msg;
            param_is_vehicle_ = true;
        }
        inline void CallbackROILanes(const ad_msgs::msg::LanePointDataArray::SharedPtr msg) {
            i_roi_lanes_ = *msg;
        }
        inline void CallbackPolyLanes(const ad_msgs::msg::PolyfitLaneDataArray::SharedPtr msg) {
            i_poly_lanes_ = *msg;
        }
        inline void CallbackDrivingWay(const ad_msgs::msg::PolyfitLaneData::SharedPtr msg) {
            i_driving_way_ = *msg;
        }
        inline void CallbackCsvLanes(const ad_msgs::msg::LanePointDataArray::SharedPtr msg) {            
            i_csv_lanes_ = *msg;
        }        

        // Algorithm functions
        void DisplayVehicle(const ad_msgs::msg::VehicleOutput& vehicle_state,
                            const rclcpp::Time& current_time);
        void DisplayROILanes(const ad_msgs::msg::LanePointDataArray& roi_lanes,
                             const rclcpp::Time& current_time);
        void DisplayPolyLanes(const ad_msgs::msg::PolyfitLaneDataArray& poly_lanes,
                              const rclcpp::Time& current_time,
                              const double& interval, const double& ROILength);
        void DisplayDrivingWay(const ad_msgs::msg::PolyfitLaneData& driving_way,
                               const rclcpp::Time& current_time,
                               const double& interval, const double& ROILength);
        void DisplayCsvLanes(const ad_msgs::msg::LanePointDataArray& csv_lanes,
                             const rclcpp::Time& current_time);
        
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variables
        
        // Publisher
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr p_vehicle_marker_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr p_roi_lanes_marker_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr p_poly_lanes_marker_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr p_driving_way_marker_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr p_csv_lanes_marker_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr p_ego_vehicle_velocity_;

        // Subscriber
        rclcpp::Subscription<ad_msgs::msg::VehicleOutput>::SharedPtr s_vehicle_state_;
        rclcpp::Subscription<ad_msgs::msg::LanePointDataArray>::SharedPtr s_roi_lanes_;
        rclcpp::Subscription<ad_msgs::msg::PolyfitLaneDataArray>::SharedPtr s_poly_lanes_;
        rclcpp::Subscription<ad_msgs::msg::PolyfitLaneData>::SharedPtr s_driving_way_;
        rclcpp::Subscription<ad_msgs::msg::LanePointDataArray>::SharedPtr s_csv_lanes_;

        // Timer
        rclcpp::TimerBase::SharedPtr t_run_node_;

        // Inputs
        ad_msgs::msg::VehicleOutput i_vehicle_state_;
        ad_msgs::msg::LanePointDataArray i_roi_lanes_;
        ad_msgs::msg::PolyfitLaneDataArray i_poly_lanes_;
        ad_msgs::msg::PolyfitLaneData i_driving_way_;
        ad_msgs::msg::LanePointDataArray i_csv_lanes_;

        // Mutex
        std::mutex mutex_vehicle_state_;
        std::mutex mutex_roi_lanes_;
        std::mutex mutex_poly_lanes_;
        std::mutex mutex_driving_way_;
        std::mutex mutex_csv_lanes_;

        // Outputs
        std_msgs::msg::Float32 o_ego_vehicle_velocity_;

        // Configuration parameters   
        std::string param_vehicle_name_space = "";
        std::string param_mesh_dir_ = "";

        // Time        
        double time_vehicle_marker_ = 0.0;
        double time_lanes_marker_ = 0.0;
        double time_csv_lanes_marker_ = 0.0;

        // Simulator       
        bool param_is_vehicle_ = false; 
};

#endif // __DISPLAY_NODE_HPP__
