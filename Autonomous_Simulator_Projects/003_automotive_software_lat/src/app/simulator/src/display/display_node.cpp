/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      display_node.cpp
 * @brief     display topics using markers
 * 
 * @date      2018-11-16 created by Eunsan Jo (eunsan.mountain@gmail.com)
 *            2023-08-07 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : adapt new template
 *            2023-08-20 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : change to ROS2
 */

#include "display/display_node.hpp"

Display::Display(const std::string& node_name, const double& loop_rate,
                 const rclcpp::NodeOptions& options)
    : Node(node_name, options) {
    
    RCLCPP_WARN(this->get_logger(), "Initialize node...");
    
    // QoS init
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    // Parameters        
    this->declare_parameter("display/ns", "");
    if (!this->get_parameter("display/ns", param_vehicle_name_space)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get vehicle_namespace");
    }
    else {
        RCLCPP_INFO(this->get_logger(), "vehicle_namespace: %s", param_vehicle_name_space.c_str());
    }    
    
    std::string dir(getenv("PWD"));
    std::string mesh_path("/resources/meshes");
    param_mesh_dir_ = dir + mesh_path;

    // Subscribers
    s_vehicle_state_ = this->create_subscription<ad_msgs::msg::VehicleOutput> (
        "vehicle_state", qos_profile, std::bind(&Display::CallbackVehicleState, this, std::placeholders::_1));
    s_csv_lanes_ = this->create_subscription<ad_msgs::msg::LanePointDataArray> (
        "csv_lanes", qos_profile, std::bind(&Display::CallbackCsvLanes, this, std::placeholders::_1));
    s_poly_lanes_ = this->create_subscription<ad_msgs::msg::PolyfitLaneDataArray> (
        "poly_lanes", qos_profile, std::bind(&Display::CallbackPolyLanes, this, std::placeholders::_1));
    s_driving_way_ = this->create_subscription<ad_msgs::msg::PolyfitLaneData> (
        "driving_way", qos_profile, std::bind(&Display::CallbackDrivingWay, this, std::placeholders::_1));
    s_roi_lanes_ = this->create_subscription<ad_msgs::msg::LanePointDataArray> (
        "ROI_lanes", qos_profile, std::bind(&Display::CallbackROILanes, this, std::placeholders::_1));
    
    // Publishers
    p_vehicle_marker_ = this->create_publisher<visualization_msgs::msg::Marker> (
        "vehicle_marker", qos_profile);
    p_roi_lanes_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray> (
        "ROI_lanes_marker", qos_profile);
    p_poly_lanes_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray> (
        "polyfit_lanes_marker", qos_profile);
    p_driving_way_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray> (
        "driving_way_marker", qos_profile);
    p_csv_lanes_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray> (
        "csv_lanes_marker", qos_profile);
    p_ego_vehicle_velocity_ = this->create_publisher<std_msgs::msg::Float32> (
        "/ego_vehicle_velocity", qos_profile);

    // Initialize
    Init(this->now());

    // Timer init
    t_run_node_ = this->create_wall_timer(
        std::chrono::milliseconds((int64_t)(1000 / loop_rate)),
        [this]() { this->Run(this->now()); }); 
}

Display::~Display() {}

void Display::Init(const rclcpp::Time& current_time) {
}

void Display::Run(const rclcpp::Time& current_time) {
    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get subscribe variables
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //   
    mutex_vehicle_state_.lock();
    ad_msgs::msg::VehicleOutput vehicle_state     = i_vehicle_state_;
    mutex_vehicle_state_.unlock();

    mutex_roi_lanes_.lock();
    ad_msgs::msg::LanePointDataArray roi_lanes    = i_roi_lanes_;
    mutex_roi_lanes_.unlock();

    mutex_poly_lanes_.lock();
    ad_msgs::msg::PolyfitLaneDataArray poly_lanes = i_poly_lanes_;
    mutex_poly_lanes_.unlock();

    mutex_driving_way_.lock();
    ad_msgs::msg::PolyfitLaneData driving_way     = i_driving_way_;
    mutex_driving_way_.unlock();

    mutex_csv_lanes_.lock();
    ad_msgs::msg::LanePointDataArray csv_lanes    = i_csv_lanes_;
    mutex_csv_lanes_.unlock();
    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Algorithm
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //  
    
    // if ((current_time.seconds() - time_vehicle_marker_) > 0.1) {
    //     time_vehicle_marker_ = current_time.seconds();
        DisplayVehicle(vehicle_state, current_time);
    // }
    if ((current_time.seconds() - time_lanes_marker_) > 0.2) {
        time_lanes_marker_ = current_time.seconds();
        DisplayROILanes(roi_lanes, current_time);
        DisplayPolyLanes(poly_lanes, current_time, 0.1, 30.0);
        DisplayDrivingWay(driving_way, current_time, 0.1, 30.0);
    }
    if ((current_time.seconds() - time_csv_lanes_marker_) > 1.0) {
        time_csv_lanes_marker_ = current_time.seconds();
        DisplayCsvLanes(csv_lanes, current_time);
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Update output
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    o_ego_vehicle_velocity_.data = vehicle_state.velocity;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Publish output
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //    
    p_ego_vehicle_velocity_->publish(o_ego_vehicle_velocity_);
}

void Display::DisplayVehicle(const ad_msgs::msg::VehicleOutput& vehicle_state,
                             const rclcpp::Time& current_time) {

    if (param_is_vehicle_ == true) {
        tf2::Quaternion q_temp;
        tf2::Matrix3x3 m(q_temp);
        q_temp.setRPY(0.0 / 180.0 * M_PI, 0, 180.0 / 180.0 * M_PI);
        tf2::Quaternion q(q_temp.getX(), q_temp.getY(), q_temp.getZ(), q_temp.getW());

        visualization_msgs::msg::Marker vehicle_marker;
        vehicle_marker.header.frame_id = param_vehicle_name_space + "/body";
        vehicle_marker.header.stamp = current_time;
        vehicle_marker.ns = vehicle_state.id;
        vehicle_marker.id = 0;

        // Set the marker type
        vehicle_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        vehicle_marker.mesh_resource =             
            "file://" + param_mesh_dir_ + "/Ioniq5.stl";
        vehicle_marker.mesh_use_embedded_materials = true;

        vehicle_marker.pose.position.x = 1.5;
        vehicle_marker.pose.position.y = 0.0;
        vehicle_marker.pose.position.z = 0.0;
        vehicle_marker.pose.orientation.x = q.getX();
        vehicle_marker.pose.orientation.y = q.getY();
        vehicle_marker.pose.orientation.z = q.getZ();
        vehicle_marker.pose.orientation.w = q.getW();

        // Set the scale of the marker
        vehicle_marker.scale.x = 1.0;
        vehicle_marker.scale.y = 1.0;
        vehicle_marker.scale.z = 1.0;

        vehicle_marker.color.r = 1.0;
        vehicle_marker.color.g = 1.0;
        vehicle_marker.color.b = 1.0;
        vehicle_marker.color.a = 1.0;

        vehicle_marker.lifetime = rclcpp::Duration(0, int64_t(0.1*1e9));

        p_vehicle_marker_->publish(vehicle_marker);
    }
}

void Display::DisplayROILanes(const ad_msgs::msg::LanePointDataArray& roi_lanes,
                              const rclcpp::Time& current_time) {
    visualization_msgs::msg::MarkerArray markerArray;
    int id = 0;

    for (auto& lane : roi_lanes.lane) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = lane.frame_id;
        marker.header.stamp = current_time;

        marker.ns = lane.id;
        marker.id = id++;

        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
        marker.scale.x = 0.25;
        marker.lifetime = rclcpp::Duration(0, int64_t(0.2*1e9));

        geometry_msgs::msg::Point prevPoint;
        bool first = true;

        for (auto& point : lane.point) {
            geometry_msgs::msg::Point currPoint;
            currPoint.x = point.x;
            currPoint.y = point.y;
            currPoint.z = 0.0;

            if (first == true) {
                first = false;
            } 
            else {
                double dx = currPoint.x - prevPoint.x;
                double dy = currPoint.y - prevPoint.y;
                if ((dx * dx + dy * dy) <= 2.0 * 2.0) {
                    marker.points.push_back(prevPoint);
                    marker.points.push_back(currPoint);
                } 
                else {
                    markerArray.markers.push_back(marker);
                    marker.points.clear();
                    marker.id = id++;
                }
            }
            prevPoint = currPoint;
        }
        markerArray.markers.push_back(marker);
    }
    p_roi_lanes_marker_->publish(markerArray);
}

void Display::DisplayPolyLanes(const ad_msgs::msg::PolyfitLaneDataArray& poly_lanes,
                               const rclcpp::Time& current_time,
                               const double& interval, const double& ROILength) {

    visualization_msgs::msg::MarkerArray markerArray;

    for (auto& lane : poly_lanes.polyfitlanes) {
        double x = 0.0;
        double y = lane.a0;

        double distance_square = x * x + y * y;
        int id = 0;

        while (distance_square < ROILength * ROILength) {
            double a0 = lane.a0;
            double a1 = lane.a1;
            double a2 = lane.a2;
            double a3 = lane.a3;

            y = a0 + a1 * x + a2 * x * x + a3 * x * x * x;
            distance_square = x * x + y * y;

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = lane.frame_id;
            marker.header.stamp = current_time;

            marker.ns = lane.id;
            marker.id = id++;

            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.pose.position.x = x;
            marker.pose.position.y = y;
            marker.pose.position.z = 0.0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.1;
            marker.pose.orientation.w = 1.0;
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.lifetime = rclcpp::Duration(0, int64_t(0.2*1e9));

            markerArray.markers.push_back(marker);
            x += interval;
        }
    }
    p_poly_lanes_marker_->publish(markerArray);
}

void Display::DisplayDrivingWay(const ad_msgs::msg::PolyfitLaneData& driving_way,
                                const rclcpp::Time& current_time,
                                const double& interval, const double& ROILength) {

    double a0 = driving_way.a0;
    double a1 = driving_way.a1;
    double a2 = driving_way.a2;
    double a3 = driving_way.a3;

    double x = 0.0;
    double y = a0;

    double distance_square = x * x + y * y;
    int id = 0;

    visualization_msgs::msg::MarkerArray markerArray;
    while (distance_square < ROILength * ROILength) {

        y = a0 + a1 * x + a2 * x * x + a3 * x * x * x;
        distance_square = x * x + y * y;

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = driving_way.frame_id;
        marker.header.stamp = current_time;

        marker.ns = driving_way.id;
        marker.id = id++;

        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.1;
        marker.pose.orientation.w = 1.0;
        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.lifetime = rclcpp::Duration(0, int64_t(0.2*1e9));

        markerArray.markers.push_back(marker);
        x += interval;
    }
    p_driving_way_marker_->publish(markerArray);
}

void Display::DisplayCsvLanes(const ad_msgs::msg::LanePointDataArray& csv_lanes,
                              const rclcpp::Time& current_time) {

    visualization_msgs::msg::MarkerArray markerArray;

    int id = 0;
    double marker_rgb[] = {0.9f,0.9f,0.9f,0.7f};

    for (auto& lane : csv_lanes.lane) {
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = lane.frame_id;
        marker.header.stamp = current_time;
        marker.ns = lane.id;
        marker.id = id++;
        
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.color.r = marker_rgb[0];
        marker.color.g = marker_rgb[1];
        marker.color.b = marker_rgb[2];
        marker.color.a = 0.7;
        marker.scale.x = 0.2;
        marker.lifetime = rclcpp::Duration(0, 0);

        geometry_msgs::msg::Point prevPoint;
        bool first = true;

        for (auto& point : lane.point) {
            geometry_msgs::msg::Point currPoint;
            currPoint.x = point.x;
            currPoint.y = point.y;
            currPoint.z = 0.0;

            std_msgs::msg::ColorRGBA color;
            color.r = 0.9f;
            color.g = 0.9f;
            color.b = 0.9f;
            color.a = 0.7;

            if (first == true) {
                first = false;
            } 
            else {
                double dx = currPoint.x - prevPoint.x;
                double dy = currPoint.y - prevPoint.y;
                if ((dx * dx + dy * dy) <= 2.0 * 2.0) {
                    marker.points.push_back(prevPoint);
                    marker.points.push_back(currPoint);
                    marker.colors.push_back(color);
                    marker.colors.push_back(color);
                } 
                else {
                    markerArray.markers.push_back(marker);
                    marker.points.clear();
                    marker.colors.clear();
                    marker.id = id++;
                }
            }
            prevPoint = currPoint;
        }
        markerArray.markers.push_back(marker);
    }
    p_csv_lanes_marker_->publish(markerArray);
}

int main(int argc, char **argv) {
    std::string node_name = "display";
    double loop_rate      = 100.0;

    // Initialize node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Display>(node_name, loop_rate));
    rclcpp::shutdown();

    return 0;
}
