/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved.
 *            Subject to limited distribution and restricted disclosure only.
 *
 * @file      autonomous_driving.cpp
 * @brief     autonomous driving algorithm
 *
 * @date      2018-11-20 created by Kichun Jo (kichunjo@hanyang.ac.kr)
 *            2023-08-07 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : adapt new template
 *            2023-08-20 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : change to ROS2
 */

#include "autonomous_driving.hpp"
#include <std_msgs/msg/float64.hpp>

AutonomousDriving::AutonomousDriving(const std::string &node_name, const double &loop_rate,
                                     const rclcpp::NodeOptions &options)
    : Node(node_name, options),
      // START REUSABLE SECTION
      target_speeds_{30, 60, 90, 70, 50, 30, 100, 50, 100},
      current_speed_index_(0),
      last_speed_change_time_(this->now()),
      pid_last_time(this->now())
      // END REUSABLE SECTION
{
    RCLCPP_WARN(this->get_logger(), "Initialize node...");

    // QoS init
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    // Parameters
    this->declare_parameter("autonomous_driving/ns", "");
    if (!this->get_parameter("autonomous_driving/ns", param_vehicle_namespace_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get vehicle_namespace");
        param_vehicle_namespace_ = "";
    }
    else {
        RCLCPP_INFO(this->get_logger(), "param_vehicle_namespace_: %s", param_vehicle_namespace_.c_str());
    }
    this->declare_parameter("autonomous_driving/use_manual_inputs", false);
    if (!this->get_parameter("autonomous_driving/use_manual_inputs", param_use_manual_inputs_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get use_manual_inputs");
        param_use_manual_inputs_ = false;
    }
    else {
        RCLCPP_INFO(this->get_logger(), "param_use_manual_inputs_: %d", param_use_manual_inputs_);
    }
    this->declare_parameter("autonomous_driving/pure_pursuit_kd", 1.0);
    if (!this->get_parameter("autonomous_driving/pure_pursuit_kd", param_pp_kd_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get pure_pursuit_kd");
        param_pp_kd_ = 1.0;
    }
    else {
        RCLCPP_INFO(this->get_logger(), "param_pp_kd_: %f", param_pp_kd_);
    }
    this->declare_parameter("autonomous_driving/pure_pursuit_kv", 0.0);
    if (!this->get_parameter("autonomous_driving/pure_pursuit_kv", param_pp_kv_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get pure_pursuit_kv");
        param_pp_kv_ = 0.0;
    }
    this->declare_parameter("autonomous_driving/pure_pursuit_kc", 0.0);
    if (!this->get_parameter("autonomous_driving/pure_pursuit_kc", param_pp_kc_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get pure_pursuit_kc");
        param_pp_kc_ = 0.0;
    }
    else {
        RCLCPP_INFO(this->get_logger(), "param_pp_kv_: %f", param_pp_kv_);
    }
    this->declare_parameter("autonomous_driving/pid_kp", 0.0);
    if (!this->get_parameter("autonomous_driving/pid_kp", param_pid_kp_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get pid_kp");
        param_pid_kp_ = 0.0;
    }
    else {
        RCLCPP_INFO(this->get_logger(), "param_pid_kp_: %f", param_pid_kp_);
    }
    this->declare_parameter("autonomous_driving/pid_ki", 0.0);
    if (!this->get_parameter("autonomous_driving/pid_ki", param_pid_ki_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get pid_ki");
        param_pid_ki_ = 0.0;
    }
    else {
        RCLCPP_INFO(this->get_logger(), "param_pid_ki_: %f", param_pid_ki_);
    }
    this->declare_parameter("autonomous_driving/pid_kd", 0.0);
    if (!this->get_parameter("autonomous_driving/pid_kd", param_pid_kd_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get pid_kd");
        param_pid_kd_ = 0.0;
    }
    else {
        RCLCPP_INFO(this->get_logger(), "param_pid_kd_: %f", param_pid_kd_);
    }
    this->declare_parameter("autonomous_driving/brake_ratio", 1.0);
    if (!this->get_parameter("autonomous_driving/brake_ratio", param_brake_ratio_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get brake_ratio");
        param_brake_ratio_ = 0.0;
    }
    else {
        RCLCPP_INFO(this->get_logger(), "param_brake_ratio_: %f", param_brake_ratio_);
    }

    // Subscribers
    s_manual_input_ = this->create_subscription<ad_msgs::msg::VehicleInput>(
        "/manual_input", qos_profile, std::bind(&AutonomousDriving::CallbackManualInput, this, std::placeholders::_1));
    s_vehicle_state_ = this->create_subscription<ad_msgs::msg::VehicleOutput>(
        "vehicle_state", qos_profile, std::bind(&AutonomousDriving::CallbackVehicleState, this, std::placeholders::_1));
    s_limit_speed_ = this->create_subscription<std_msgs::msg::Float32>(
        "/limit_speed", qos_profile, std::bind(&AutonomousDriving::CallbackLimitSpeed, this, std::placeholders::_1));
    s_lane_points_ = this->create_subscription<ad_msgs::msg::LanePointData>(
        "/lane_points", qos_profile, std::bind(&AutonomousDriving::CallbackLanePoints, this, std::placeholders::_1));

    // Publishers
    p_vehicle_command_ = this->create_publisher<ad_msgs::msg::VehicleInput>(
        "vehicle_command", qos_profile);
    p_driving_way_ = this->create_publisher<ad_msgs::msg::PolyfitLaneData>(
        "driving_way", qos_profile);
    // START REUSABLE SECTION
    p_limit_speed_data_ = this->create_publisher<std_msgs::msg::Float64>("limit_speed_data", 10);
    p_ego_velocity_ = this->create_publisher<std_msgs::msg::Float64>("ego/vehicle_state/velocity", 10);
    // END REUSABLE SECTION

    // Initialize
    Init(this->now());

    // Timer init
    t_run_node_ = this->create_wall_timer(
        std::chrono::milliseconds((int64_t)(1000 / loop_rate)),
        [this]()
        { this->Run(this->now()); });
}

AutonomousDriving::~AutonomousDriving() {}

void AutonomousDriving::Init(const rclcpp::Time &current_time) {
    // You can remove this function if it's not used
}

void AutonomousDriving::UpdateParameter() {
    this->get_parameter("autonomous_driving/pure_pursuit_kd", param_pp_kd_);
    this->get_parameter("autonomous_driving/pure_pursuit_kv", param_pp_kv_);
    this->get_parameter("autonomous_driving/pure_pursuit_kc", param_pp_kc_);
    this->get_parameter("autonomous_driving/pid_kp", param_pid_kp_);
    this->get_parameter("autonomous_driving/pid_ki", param_pid_ki_);
    this->get_parameter("autonomous_driving/pid_kd", param_pid_kd_);
    this->get_parameter("autonomous_driving/brake_ratio", param_brake_ratio_);
}

void AutonomousDriving::Run(const rclcpp::Time &current_time) {
    UpdateParameter();

    // Get subscribe variables
    mutex_vehicle_state_.lock();
    ad_msgs::msg::VehicleOutput vehicle_state = i_vehicle_state_;
    mutex_vehicle_state_.unlock();

    mutex_limit_speed_.lock();
    double target_speed = i_limit_speed_;
    mutex_limit_speed_.unlock();

    mutex_lane_points_.lock();
    ad_msgs::msg::LanePointData lane_points = i_lane_points_;
    mutex_lane_points_.unlock();

    // Algorithm
    ad_msgs::msg::VehicleInput vehicle_command;
    ad_msgs::msg::PolyfitLaneData driving_way;

    if (!param_use_manual_inputs_) {
        // START REUSABLE SECTION
        PID(current_time, target_speed, vehicle_state.velocity);

        if (computed_input >= 0.0) {
            vehicle_command.accel = computed_input;
            vehicle_command.brake = 0.0;
        } else {
            vehicle_command.accel = 0.0;
            vehicle_command.brake = -computed_input;
        }

        // Publish target speed
        auto target_speed_msg = std_msgs::msg::Float64();
        target_speed_msg.data = target_speed;
        p_limit_speed_data_->publish(target_speed_msg);

        // Publish current vehicle speed
        auto current_speed_msg = std_msgs::msg::Float64();
        current_speed_msg.data = vehicle_state.velocity;
        p_ego_velocity_->publish(current_speed_msg);

        RCLCPP_INFO(this->get_logger(), "Target speed index: %zu, Target speed: %f, Current speed: %f", 
                    current_speed_index_, target_speed, vehicle_state.velocity);
        // END REUSABLE SECTION
    }

    // Update output
    o_driving_way_ = driving_way;
    o_vehicle_command_ = vehicle_command;

    // Publish output
    Publish(current_time);
}

void AutonomousDriving::Publish(const rclcpp::Time &current_time) {
    p_vehicle_command_->publish(o_vehicle_command_);
    p_driving_way_->publish(o_driving_way_);
}

// START REUSABLE SECTION
void AutonomousDriving::PID(const rclcpp::Time& current_time, double target_value, double current_value) {
    double dt = (current_time - pid_last_time).seconds();

    if (dt <= 0.0) {
        dt = 0.01;
    }

    e = target_value - current_value;

    int_e += (dt * (e + e_prev) / 2.0);

    double dev_e = (e - e_prev) / dt;

    computed_input = param_pid_kp_ * e + param_pid_ki_ * int_e + param_pid_kd_ * dev_e;

    e_prev = e;
    pid_last_time = current_time;
}
// END REUSABLE SECTION

void AutonomousDriving::CallbackManualInput(const ad_msgs::msg::VehicleInput::SharedPtr msg) {            
    mutex_manual_input_.lock();
    if(param_use_manual_inputs_ == true) {
        o_vehicle_command_.accel = msg->accel;
        o_vehicle_command_.brake = msg->brake;
        o_vehicle_command_.steering = msg->steering;
    }
    mutex_manual_input_.unlock();
}

void AutonomousDriving::CallbackVehicleState(const ad_msgs::msg::VehicleOutput::SharedPtr msg) {            
    mutex_vehicle_state_.lock();
    i_vehicle_state_ = *msg;
    mutex_vehicle_state_.unlock();
}

void AutonomousDriving::CallbackLimitSpeed(const std_msgs::msg::Float32::SharedPtr msg) {            
    mutex_limit_speed_.lock();
    
    // START REUSABLE SECTION
    // Update target speed sequence
    if ((this->now() - last_speed_change_time_).seconds() >= 10.0) {  // Change speed every 10 seconds
        current_speed_index_ = (current_speed_index_ + 1) % target_speeds_.size();
        last_speed_change_time_ = this->now();
        i_limit_speed_ = target_speeds_[current_speed_index_];
        RCLCPP_INFO(this->get_logger(), "Changing target speed to: %f", i_limit_speed_);
    } else {
        // Use the received limit speed only if we're not changing to the next target speed
        i_limit_speed_ = msg->data;
    }
    // END REUSABLE SECTION
    
    mutex_limit_speed_.unlock();
}

void AutonomousDriving::CallbackLanePoints(const ad_msgs::msg::LanePointData::SharedPtr msg) {            
    mutex_lane_points_.lock();
    i_lane_points_ = *msg;
    mutex_lane_points_.unlock();
}

int main(int argc, char **argv) {
    std::string node_name = "autonomous_driving";
    double loop_rate = 100.0;

    // Initialize node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutonomousDriving>(node_name, loop_rate));
    rclcpp::shutdown();
    return 0;
}
