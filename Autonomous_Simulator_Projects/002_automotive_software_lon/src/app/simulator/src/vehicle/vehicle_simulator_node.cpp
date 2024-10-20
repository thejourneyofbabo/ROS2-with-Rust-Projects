/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      vehicle_simulator_node.cpp
 * @brief     simulate vehicle
 * 
 * @date      2018-11-16 created by Eunsan Jo (eunsan.mountain@gmail.com)
 *            2023-08-07 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : adapt new template
 *            2023-08-20 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : change to ROS2
 */

#include "vehicle/vehicle_simulator_node.hpp"

Vehicle::Vehicle(const std::string& node_name, const double& loop_rate,
                 const rclcpp::NodeOptions& options)
    : Node(node_name, options), tf2_broadcaster_(this) {
    
    // Initialize
    RCLCPP_WARN_STREAM(this->get_logger(), "Initialize node (Period: " << loop_rate << " Hz)");
    
    // QoS
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    // Parameter init
    this->declare_parameter("vehicle/ns", "");
    this->declare_parameter("vehicle/init_x", 0.0);
    this->declare_parameter("vehicle/init_y", 0.0);
    this->declare_parameter("vehicle/init_yaw", 0.0);

    this->get_parameter("vehicle/ns", param_.vehicle_namespace);
    this->get_parameter("vehicle/init_x", param_.init_x);
    this->get_parameter("vehicle/init_y", param_.init_y);
    this->get_parameter("vehicle/init_yaw", param_.init_yaw);

    RCLCPP_INFO(this->get_logger(), "vehicle_namespace: %s", param_.vehicle_namespace.c_str());
    RCLCPP_INFO(this->get_logger(), "init_x: %f", param_.init_x);
    RCLCPP_INFO(this->get_logger(), "init_y: %f", param_.init_y);
    RCLCPP_INFO(this->get_logger(), "init_yaw: %f", param_.init_yaw);

    // Subscriber init
    s_vehicle_command_ = this->create_subscription<ad_msgs::msg::VehicleInput> (
        "vehicle_command", qos_profile, std::bind(&Vehicle::CallbackVehicleCommand, this, std::placeholders::_1));

    // Publisher init
    p_vehicle_state_ = this->create_publisher<ad_msgs::msg::VehicleOutput> (
        "vehicle_state", qos_profile);

    // Initialize
    Init(this->now());

    // Timer init
    t_run_node_ = this->create_wall_timer(
        std::chrono::milliseconds((int64_t)(1000 / loop_rate)),
        [this]() { this->Run(this->now()); });

}

Vehicle::~Vehicle() {}

void Vehicle::Init(const rclcpp::Time& current_time) {    
    prev_vehicle_state_.x        = param_.init_x;
    prev_vehicle_state_.y        = param_.init_y;
    prev_vehicle_state_.yaw      = param_.init_yaw;
    prev_vehicle_state_.velocity = param_.init_vel;

    time_prev_ = current_time.seconds();
}

void Vehicle::Run(const rclcpp::Time& current_time) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000, "Running ...");
    double curr_simulation_time = current_time.seconds();
    time_dt_ = curr_simulation_time - time_prev_;
    if (time_dt_ <= 0.0) {
        return;
    }
    time_prev_ = curr_simulation_time;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get subscribe variables
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    mutex_vehicle_command_.lock();
    interface::VehicleCommand vehicle_command = i_vehicle_command_;
    mutex_vehicle_command_.unlock();

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Algorithm
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //  

    // Simulate Vehicle Model
    interface::VehicleState updated_vehicle_state = SimVehicleLongitudinalModel(prev_vehicle_state_, vehicle_command);
    updated_vehicle_state = SimVehicleLateralModel(updated_vehicle_state, vehicle_command);

    // Broadcast vehicle's body TF
    BroadcastVehicleTF(current_time, updated_vehicle_state);
    

    prev_vehicle_state_ = updated_vehicle_state;
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Update output
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    o_vehicle_state_.id         = param_.vehicle_namespace;
    o_vehicle_state_.x          = updated_vehicle_state.x;
    o_vehicle_state_.y          = updated_vehicle_state.y;
    o_vehicle_state_.yaw        = updated_vehicle_state.yaw;
    o_vehicle_state_.velocity   = updated_vehicle_state.velocity;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Publish output
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    Publish(current_time);
}

void Vehicle::Publish(const rclcpp::Time& current_time) {
    p_vehicle_state_->publish(o_vehicle_state_);
}

/**
 * brief:   Longutudinal model simulation using vehicle_command
 * input:   prev_vehicle_state, vehicle_command (accel(0~1) and brake(0~1))
 * output:  updated_vehicle_state
 */
interface::VehicleState Vehicle::SimVehicleLongitudinalModel(const interface::VehicleState& prev_vehicle_state,
                                                             const interface::VehicleCommand& vehicle_command) {
    interface::VehicleState updated_vehicle_state = prev_vehicle_state;                                                            

    if (updated_vehicle_state.velocity < 0.0001 && vehicle_command.brake >= 0.0001) {
        updated_vehicle_state.velocity = 0.0;
        return updated_vehicle_state;
    }
    if (updated_vehicle_state.velocity < 0.0001 && vehicle_command.accel <= 0.0001) {
        updated_vehicle_state.velocity = 0.0;
        return updated_vehicle_state;
    }

    ////////////////////// TODO //////////////////////
    // torque_motor = accel_const * pedal_accel
    // torque_brake = brake_const * pedal_brake
    double torque_motor = vehicle_param::accel_const * vehicle_command.accel; // Nm
    double torque_brake = vehicle_param::brake_const * vehicle_command.brake; // Nm

    // J_m_eq = I_m + I_t + I_w * gear_ratio^2 + Mass * r_eff^2 * gear_ratio^2
    double j_m_eq = vehicle_param::I_m + vehicle_param::I_t + vehicle_param::I_w * vehicle_param::gear_ratio * vehicle_param::gear_ratio +
                    vehicle_param::Mass * vehicle_param::r_eff * vehicle_param::r_eff * vehicle_param::gear_ratio * vehicle_param::gear_ratio;

    // R_xf + R_xr = k_R * (F_zf + F_zr)
    double rolling_resistance = vehicle_param::k_R * vehicle_param::Mass * 9.81 * cos(vehicle_param::slope);
    if (updated_vehicle_state.velocity < 0.0) {
        rolling_resistance *= -1.0;
    }

    // F_aero = 1/2 * rho * Cd * A * v^2
    double aero_drag =
        0.5 * vehicle_param::rho * vehicle_param::Cd * vehicle_param::Af * updated_vehicle_state.velocity * updated_vehicle_state.velocity;

    // T_load = gear_ratio * r_eff * (R_xf + R_xr + F_aero + F_g * sin(slope))
    double torque_load = vehicle_param::gear_ratio * vehicle_param::r_eff *
            (rolling_resistance + aero_drag + vehicle_param::Mass * 9.81 * sin(vehicle_param::slope));

    // w_motor = (T_motor - T_load - T_brake) / J_m_eq
    double omega_motor =
        (torque_motor - torque_load - vehicle_param::gear_ratio * torque_brake) / j_m_eq;

    // v = r_eff * w_motor * gear_ratio
    updated_vehicle_state.velocity += time_dt_ * (vehicle_param::r_eff * (omega_motor * vehicle_param::gear_ratio));    
    //////////////////////////////////////////////////

    return updated_vehicle_state;
}

/**
 * brief:   Lateral dynamic model simulation using vehicle command and vehicle state
 * input:   prev_vehicle_state, vehicle_command (steering angle)
 * output:  updated_vehicle_state
 */
interface::VehicleState Vehicle::SimVehicleLateralModel(const interface::VehicleState& prev_vehicle_state,
                                                        const interface::VehicleCommand& vehicle_command) {
    
    interface::VehicleState updated_vehicle_state = prev_vehicle_state;

    double delta_x = 0.0;
    double delta_y = 0.0;
    double delta_slip_angle = 0.0;
    double delta_yaw = 0.0;
    double delta_yaw_rate = 0.0;

    double Fyf = 0.0;
    double Fyr = 0.0;

    // Dynamic model
    if (fabs(updated_vehicle_state.velocity) >= 5.0) {
        ////////////////////// TODO //////////////////////

        // delta_x = velocity * cos(yaw + slip_angle) * dt
        // delta_y = velocity * sin(yaw + slip_angle) * dt
        delta_x = updated_vehicle_state.velocity * cos(updated_vehicle_state.yaw + updated_vehicle_state.slip_angle) * time_dt_;
        delta_y = updated_vehicle_state.velocity * sin(updated_vehicle_state.yaw + updated_vehicle_state.slip_angle) * time_dt_;

        // alpha_f = steering_angle - slip_angle - (L_f * yaw_rate) / (velocity)
        double alpha_f = vehicle_command.steering - updated_vehicle_state.slip_angle -
                        (vehicle_param::L_f * updated_vehicle_state.yaw_rate) / (updated_vehicle_state.velocity);        
        // Set max front steer angle
        alpha_f = std::max(std::min(alpha_f, vehicle_param::max_alpha), -1.0 * vehicle_param::max_alpha);
        
        // Fyf = 2 * Cf * alpha_f
        Fyf = (2 * vehicle_param::Cf) * alpha_f;
        // Set max Fyf
        Fyf = std::max(std::min(Fyf, vehicle_param::Fyf_max), -1.0 * vehicle_param::Fyf_max);

        // alpha_r = -slip_angle + (L_r * yaw_rate) / (velocity)
        double alpha_r = -updated_vehicle_state.slip_angle + (vehicle_param::L_r * updated_vehicle_state.yaw_rate) 
                                                    / (updated_vehicle_state.velocity);
        // Set max rear steer angle
        alpha_r = std::max(std::min(alpha_r, vehicle_param::max_alpha), -1.0 * vehicle_param::max_alpha);

        // Fyr = 2 * Cr * alpha_r
        Fyr = (2 * vehicle_param::Cr) * alpha_r;
        // Set max Fyr
        Fyr = std::max(std::min(Fyr, vehicle_param::Fyr_max), -1.0 * vehicle_param::Fyr_max);

        // delta_slip_angle = (Fyf / (Mass * velocity) + Fyr / (Mass * velocity) - yaw_rate) * dt
        delta_slip_angle = (Fyf / (vehicle_param::Mass * updated_vehicle_state.velocity) +
                            Fyr / (vehicle_param::Mass * updated_vehicle_state.velocity) - updated_vehicle_state.yaw_rate)
                            * time_dt_;

        // delta_yaw = yaw_rate * dt
        delta_yaw = updated_vehicle_state.yaw_rate * time_dt_;

        // delta_yaw_rate = ((2 * L_f * Cf) / (Inertia) * (steering_angle - slip_angle - (L_f * yaw_rate) / (velocity)) -
        //                   (2 * L_r * Cr) / (Inertia) * (-slip_angle + (L_r * yaw_rate) / (velocity))) * dt
        delta_yaw_rate =
            ((2 * vehicle_param::L_f * vehicle_param::Cf) / (vehicle_param::Inertia) * 
                (vehicle_command.steering - updated_vehicle_state.slip_angle - (vehicle_param::L_f * updated_vehicle_state.yaw_rate) / (updated_vehicle_state.velocity)) 
            - (2 * vehicle_param::L_r * vehicle_param::Cr) / (vehicle_param::Inertia) * 
                (-updated_vehicle_state.slip_angle + (vehicle_param::L_r * updated_vehicle_state.yaw_rate) / (updated_vehicle_state.velocity)))
            * time_dt_;        

        //////////////////////////////////////////////////

        updated_vehicle_state.x += delta_x;
        updated_vehicle_state.y += delta_y;
        updated_vehicle_state.slip_angle += delta_slip_angle;
        updated_vehicle_state.yaw += delta_yaw;
        updated_vehicle_state.yaw_rate += delta_yaw_rate;
    }
    // Kinematic model
    else {
        ////////////////////// TODO //////////////////////

        // slip_angle = atan2((L_r * tan(steering_angle)), (L_f + L_r))
        updated_vehicle_state.slip_angle = atan2((vehicle_param::L_r * tan(vehicle_command.steering)), (vehicle_param::L_f + vehicle_param::L_r));

        // delta_x = velocity * cos(yaw + slip_angle) * dt
        // delta_y = velocity * sin(yaw + slip_angle) * dt
        delta_x = updated_vehicle_state.velocity * cos(updated_vehicle_state.yaw + updated_vehicle_state.slip_angle) * time_dt_;
        delta_y = updated_vehicle_state.velocity * sin(updated_vehicle_state.yaw + updated_vehicle_state.slip_angle) * time_dt_;

        // yaw_rate = ((velocity * cos(slip_angle)) / (L_f + L_r)) * (tan(steering_angle))
        updated_vehicle_state.yaw_rate = ((updated_vehicle_state.velocity * cos(updated_vehicle_state.slip_angle)) 
                                    / (vehicle_param::L_f + vehicle_param::L_r)) * (tan(vehicle_command.steering));
                                    
        // delta_yaw = yaw_rate * dt
        delta_yaw = updated_vehicle_state.yaw_rate * time_dt_;

        //////////////////////////////////////////////////
        updated_vehicle_state.x += delta_x;
        updated_vehicle_state.y += delta_y;
        updated_vehicle_state.yaw += delta_yaw;
    }

    // Heading normalization
    double yaw_degree = fmod(updated_vehicle_state.yaw*180/M_PI,360);
    if (yaw_degree < 0) {
        yaw_degree += 360; 
    }
    updated_vehicle_state.yaw = yaw_degree*M_PI/180.0;
    
    // RCLCPP_INFO_STREAM(this->get_logger(), 
    //                 "\n\n"<< "time_dt_ "<< time_dt_ <<"\n" 
    //                 <<"x " << updated_vehicle_state.x <<"\n"
    //                 <<"y " << updated_vehicle_state.y <<"\n"
    //                 <<"yaw " << updated_vehicle_state.yaw <<"\n"
    //                 <<"velocity "<<updated_vehicle_state.velocity <<"\n"
    //                 <<"yaw_rate "<<updated_vehicle_state.yaw_rate <<"\n"
    //                 <<"slip_angle "<< updated_vehicle_state.slip_angle <<"\n"
    //                 <<"command_steering " << vehicle_command.steering<< "\n"
    //                 <<"delta_yaw_rate    " <<delta_yaw_rate << "\n" );

    return updated_vehicle_state;
}

void Vehicle::BroadcastVehicleTF(const rclcpp::Time& current_time,
                                 const interface::VehicleState& vehicle_state) {
    tf2::Quaternion q;
    
    q.setRPY(0, 0, vehicle_state.yaw);
    q.normalize();
    geometry_msgs::msg::Pose pose;

    pose.position.x = vehicle_state.x;
    pose.position.y = vehicle_state.y;
    pose.orientation.x = q.getX();
    pose.orientation.y = q.getY();
    pose.orientation.z = q.getZ();
    pose.orientation.w = q.getW();

    tf2::Transform vehicle_transform;

    vehicle_transform.setOrigin(tf2::Vector3(vehicle_state.x, vehicle_state.y, 0.0));
    vehicle_transform.setRotation(q);

    // broadcasting the vehicle's body coordinate system
    // The parent coordinate is world, child coordinate is body.
    geometry_msgs::msg::TransformStamped transformStamped;  // Using the ROS 2 message type

    transformStamped.header.stamp = current_time;
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = param_.vehicle_namespace + "/body";
    transformStamped.transform.translation.x = vehicle_transform.getOrigin().x();
    transformStamped.transform.translation.y = vehicle_transform.getOrigin().y();
    transformStamped.transform.translation.z = vehicle_transform.getOrigin().z();
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    tf2_broadcaster_.sendTransform(transformStamped);
}

int main(int argc, char **argv) {
    std::string node_name = "vehicle";
    double loop_rate      = 100.0;

    // Initialize node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Vehicle>(node_name, loop_rate));
    rclcpp::shutdown();

    return 0;
}
