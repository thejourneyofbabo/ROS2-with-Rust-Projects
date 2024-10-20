#include <chrono>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtle_interfaces/msg/color_rgb.hpp"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;

class HMI : public rclcpp::Node
{
public:
  HMI() : Node("hmi")
  {
    pose_subscription_ = this->create_subscription<turtlesim::msg::Pose>(
      "/turtle1/pose", 10, std::bind(&HMI::pose_callback, this, std::placeholders::_1));
    
    color_subscription_ = this->create_subscription<turtle_interfaces::msg::ColorRGB>(
      "/turtle_color", 10, std::bind(&HMI::color_callback, this, std::placeholders::_1));
    
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/turtle_marker", 10);
    
    timer_ = this->create_wall_timer(16.67ms, std::bind(&HMI::timer_callback, this));
  }

private:
  void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
  {
    current_pose_ = *msg;
  }

  void color_callback(const turtle_interfaces::msg::ColorRGB::SharedPtr msg)
  {
    current_color_ = *msg;
  }

  void timer_callback()
  {
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = current_pose_.x;
    marker.pose.position.y = current_pose_.y;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = current_color_.r;
    marker.color.g = current_color_.g;
    marker.color.b = current_color_.b;
    marker.color.a = 1.0;

    marker_publisher_->publish(marker);
  }

  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscription_;
  rclcpp::Subscription<turtle_interfaces::msg::ColorRGB>::SharedPtr color_subscription_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  turtlesim::msg::Pose current_pose_;
  turtle_interfaces::msg::ColorRGB current_color_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HMI>());
  rclcpp::shutdown();
  return 0;
}
