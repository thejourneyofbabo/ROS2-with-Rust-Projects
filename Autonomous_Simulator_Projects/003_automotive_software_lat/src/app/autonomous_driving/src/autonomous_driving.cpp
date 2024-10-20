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

#include "autonomous_driving.hpp"

AutonomousDriving::AutonomousDriving(const std::string &node_name, const double &loop_rate,
                                     const rclcpp::NodeOptions &options)
    : Node(node_name, options) {

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
        param_use_manual_inputs_ = true;
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

    this->declare_parameter("autonomous_driving/ROIFront", 20.0);
    if (!this->get_parameter("autonomous_driving/ROIFront", param_m_ROIFront_param)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get ROIFront");
        param_m_ROIFront_param = 20.0;
    }
    else {
        RCLCPP_INFO(this->get_logger(), "ROIFront: %f", param_m_ROIFront_param);
    }
    this->declare_parameter("autonomous_driving/ROIRear", 10.0);
    if (!this->get_parameter("autonomous_driving/ROIRear", param_m_ROIRear_param)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get ROIRear");
        param_m_ROIRear_param = 10.0;
    }
    else {
        RCLCPP_INFO(this->get_logger(), "ROIRear: %f", param_m_ROIRear_param);
    }
    this->declare_parameter("autonomous_driving/ROILeft", 3.0);
    if (!this->get_parameter("autonomous_driving/ROILeft", param_m_ROILeft_param)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get ROILeft");
        param_m_ROILeft_param = 3.0;
    }
    else {
        RCLCPP_INFO(this->get_logger(), "ROILeft: %f", param_m_ROILeft_param);
    }
    this->declare_parameter("autonomous_driving/ROIRight", 3.0);
    if (!this->get_parameter("autonomous_driving/ROIRight", param_m_ROIRight_param)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get ROIRight");
        param_m_ROIRight_param = 3.0;
    }
    else {
        RCLCPP_INFO(this->get_logger(), "ROIRight: %f", param_m_ROIRight_param);
    }  

    // Subscribers
    s_manual_input_ = this->create_subscription<ad_msgs::msg::VehicleInput>(
        "/manual_input", qos_profile, std::bind(&AutonomousDriving::CallbackManualInput, this, std::placeholders::_1));
    s_vehicle_state_ = this->create_subscription<ad_msgs::msg::VehicleOutput>(
        "vehicle_state", qos_profile, std::bind(&AutonomousDriving::CallbackVehicleState, this, std::placeholders::_1));
    s_lane_points_array_ = this->create_subscription<ad_msgs::msg::LanePointDataArray>(
        "lane_points_array", qos_profile, std::bind(&AutonomousDriving::CallbackLanePointsArray, this, std::placeholders::_1));

    // Publishers
    p_vehicle_command_ = this->create_publisher<ad_msgs::msg::VehicleInput>(
        "vehicle_command", qos_profile);
    p_driving_way_ = this->create_publisher<ad_msgs::msg::PolyfitLaneData>(
        "driving_way", qos_profile);
    p_poly_lanes_ = this->create_publisher<ad_msgs::msg::PolyfitLaneDataArray>(
        "poly_lanes", qos_profile);

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
}

void AutonomousDriving::UpdateParameter() {
}

void AutonomousDriving::Run(const rclcpp::Time &current_time) {
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Update parameters
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    UpdateParameter();

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get subscribe variables
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    mutex_vehicle_state_.lock();
    ad_msgs::msg::VehicleOutput vehicle_state = i_vehicle_state_;
    mutex_vehicle_state_.unlock();

    mutex_lane_points_.lock();
    ad_msgs::msg::LanePointDataArray lane_points_array = i_lane_points_array_;
    mutex_lane_points_.unlock();

    /* Max speed */
    double limit_speed = 50 / 3.6; // [mps] =50kph

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Algorithm
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //

    /* Output variables */
    ad_msgs::msg::PolyfitLaneDataArray poly_lanes;
    poly_lanes.frame_id = param_vehicle_namespace_ + "/body";

    ad_msgs::msg::PolyfitLaneData driving_way;
    driving_way.frame_id = param_vehicle_namespace_ + "/body";
    
    ad_msgs::msg::VehicleInput vehicle_command;

    if (param_use_manual_inputs_ == false) {
        /*
            @input
                vehicle_state       : current vehicle state
                lane_points_array   : array that contains detected points of each lane
                limit_speed         : limit speed not to occur slip
            @output
                poly_lanes          : left and right lane generated by using curve fitting
                                      each lane is represented as form of cubic equation (a3*x^3 + a2*x^2 + a1*x + a0)
                driving_way         : centerline or customized lane which vehicle should follow
                                      driving_way is represented as form of cubic equaion (a3*x^3 + a2*x^2 + a1*x + a0)
                vehicle_command     : vehicle command (accel, brake, steering)
                                      accel and brake is between 0~1, steering [rad] is automatically restricted
            @brief
                Generate the centerline which vehicle will follow and calculate the vehicle command
        */

        // 1. Find left and right line using lane_point_array with pseudo inverse

        // 2. Generate centerline using left and right lane from 1.

        // 3. Calculate the accel or brake
        vehicle_command.accel = 0.0;
        vehicle_command.brake = 0.0;

        /* We will provide simple speed control code. You can use your own code */
        // if(vehicle_state.velocity > limit_speed) {
        //     vehicle_command.accel = 0.0;
        //     vehicle_command.brake = 1.0;
        // }
        // else {
        //     vehicle_command.accel = 1.0;
        //     vehicle_command.brake = 0.0;
        // }

        // 4. Calculate the steering angle [rad]
        vehicle_command.steering = 0.0;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Update output
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    o_driving_way_ = driving_way;
    o_poly_lanes_ = poly_lanes;
    o_vehicle_command_ = vehicle_command;
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Publish output
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    Publish(current_time);
}



void AutonomousDriving::Publish(const rclcpp::Time &current_time) {
    p_vehicle_command_->publish(o_vehicle_command_);
    p_driving_way_->publish(o_driving_way_);
    p_poly_lanes_->publish(o_poly_lanes_);
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
