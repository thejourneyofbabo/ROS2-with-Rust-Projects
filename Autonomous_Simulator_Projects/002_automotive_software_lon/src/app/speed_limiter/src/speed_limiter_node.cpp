/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      speed_limiter_algorithm.cpp
 * @brief     provide limit speed to autonomous driving algorithm
 *            this package is only used in practice "Longitudinal Control"
 * 
 * @date      2024-10-14 created by Seongjae Jeong (sjeong99@hanyang.ac.kr)
 */

#include "speed_limiter_node.hpp"

SpeedLimiter::SpeedLimiter(const std::string& node_name, const double& loop_rate,
                       const rclcpp::NodeOptions& options)
    : Node(node_name, options) {
    
    RCLCPP_WARN(this->get_logger(), "Initialize node...");
    
    // QoS init
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    // Parameters    

    // Subscribers
    s_ego_vehicle_state_ = this->create_subscription<ad_msgs::msg::VehicleOutput>(
        "/ego/vehicle_state", qos_profile, std::bind(&SpeedLimiter::CallbackEgoVehicleState, this, std::placeholders::_1));

    // Publishers
    p_limit_speed_ = this->create_publisher<std_msgs::msg::Float32>(
        "/limit_speed", qos_profile);
    p_text_speed_limit_result_ = this->create_publisher<rviz_2d_overlay_msgs::msg::OverlayText>(
        "/text_speed_limit_result", qos_profile);

    // Initialize
    Init(this->now());

    // Timer init
    t_run_node_ = this->create_wall_timer(
        std::chrono::milliseconds((int64_t)(1000 / loop_rate)),
        [this]() { this->Run(this->now()); });        
}

SpeedLimiter::~SpeedLimiter() {}

void SpeedLimiter::Init(const rclcpp::Time& current_time) {   

    // Algorithm
    ptr_speed_limit_algorithm_ = std::make_unique<SpeedLimiterAlgorithm>();

    // Update Limit speed vector


    time_prev_ = current_time.seconds();
    time_start_ = current_time.seconds();
}

void SpeedLimiter::Run(const rclcpp::Time& current_time) {
    double curr_time = current_time.seconds(); 

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get subscribe variables
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    interface::VehicleState ego_vehicle_state; {
        mutex_ego_vehicle_state_.lock();
        ego_vehicle_state = i_ego_vehicle_state_;
        mutex_ego_vehicle_state_.unlock();
    }

    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Algorithm
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //  
    if (param_is_simulator_on_ == false) {
        time_start_ = curr_time;
        ego_vehicle_state.velocity = 0.0;
    }
        
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Update output
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    UpdateSpeedLimiterResult(ego_vehicle_state, curr_time);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Publish output
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    Publish(current_time);
}

void SpeedLimiter::UpdateSpeedLimiterResult(const interface::VehicleState& ego_vehicle_state,
                                            const double& current_time) {
    o_limit_speed_.data = ptr_speed_limit_algorithm_->SetSpeedLimits(ego_vehicle_state);

    if (fabs(prev_target_limit_speed_ - o_limit_speed_.data) > 5.0/3.6) {
        time_reach_ = current_time - time_start_;
        time_start_ = current_time;
    }
    time_use_ = current_time - time_start_;

    std::string speedlimit_info 
        = ptr_speed_limit_algorithm_->GetSpeedLimitInfo(ego_vehicle_state, time_reach_, time_use_);
    // RCLCPP_INFO_STREAM(this->get_logger(), speedlimit_info);
    
    rviz_2d_overlay_msgs::msg::OverlayText speedlimittext;
    speedlimittext.bg_color.r = 0.0f;
    speedlimittext.bg_color.g = 0.0f;
    speedlimittext.bg_color.b = 0.0f;
    speedlimittext.bg_color.a = 0.0f;

    speedlimittext.fg_color.r = 0.9f;
    speedlimittext.fg_color.g = 0.9f;
    speedlimittext.fg_color.b = 0.9f;
    speedlimittext.fg_color.a = 0.7f;
    
    speedlimittext.line_width = 1;
    speedlimittext.text_size = 9.0;
    speedlimittext.text = speedlimit_info;

    o_text_speed_limit_result_ = speedlimittext;

    prev_target_limit_speed_ = o_limit_speed_.data;
}

void SpeedLimiter::Publish(const rclcpp::Time& current_time) {
    p_limit_speed_->publish(o_limit_speed_);
    p_text_speed_limit_result_->publish(o_text_speed_limit_result_);
}

int main(int argc, char **argv) {
    std::string node_name = "speed_limiter";
    double loop_rate      = 50.0;

    // Initialize node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpeedLimiter>(node_name, loop_rate));
    rclcpp::shutdown();

    return 0;
}
