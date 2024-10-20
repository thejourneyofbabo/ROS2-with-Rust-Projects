#include <chrono>
#include <functional>
#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtle_interfaces/msg/color_rgb.hpp"

using namespace std::chrono_literals;

class ColorChanger : public rclcpp::Node
{
public:
  ColorChanger() : Node("color_changer")
  {
    pose_subscription_ = this->create_subscription<turtlesim::msg::Pose>(
      "/turtle1/pose", 10, std::bind(&ColorChanger::pose_callback, this, std::placeholders::_1));
    
    color_publisher_ = this->create_publisher<turtle_interfaces::msg::ColorRGB>("/turtle_color", 10);
    
    param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "/turtlesim");
    
    timer_ = this->create_wall_timer(100ms, std::bind(&ColorChanger::timer_callback, this));
  }

private:
  void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
  {
    current_theta_ = msg->theta;
  }

  void timer_callback()
  {
    auto color_msg = turtle_interfaces::msg::ColorRGB();
    std::vector<rclcpp::Parameter> params;

    if (current_theta_ >= 0 && current_theta_ < M_PI_2) {
      params = {
        rclcpp::Parameter("background_r", 255),
        rclcpp::Parameter("background_g", 255),
        rclcpp::Parameter("background_b", 255)
      };
      color_msg.r = 1.0;
      color_msg.g = 0.0;
      color_msg.b = 0.0;
    } else if (current_theta_ >= M_PI_2 && current_theta_ < M_PI) {
      params = {
        rclcpp::Parameter("background_r", 255),
        rclcpp::Parameter("background_g", 0),
        rclcpp::Parameter("background_b", 0)
      };
      color_msg.r = 1.0;
      color_msg.g = 0.0;
      color_msg.b = 0.0;
    } else if (current_theta_ >= -M_PI && current_theta_ < -M_PI_2) {
      params = {
        rclcpp::Parameter("background_r", 0),
        rclcpp::Parameter("background_g", 255),
        rclcpp::Parameter("background_b", 0)
      };
      color_msg.r = 1.0;
      color_msg.g = 0.0;
      color_msg.b = 0.0;
    } else {
      params = {
        rclcpp::Parameter("background_r", 0),
        rclcpp::Parameter("background_g", 0),
        rclcpp::Parameter("background_b", 255)
      };
      color_msg.r = 0.0;
      color_msg.g = 0.0;
      color_msg.b = 1.0;
    }

    param_client_->set_parameters(params);
    color_publisher_->publish(color_msg);
  }

  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscription_;
  rclcpp::Publisher<turtle_interfaces::msg::ColorRGB>::SharedPtr color_publisher_;
  std::shared_ptr<rclcpp::AsyncParametersClient> param_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  double current_theta_ = 0.0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ColorChanger>());
  rclcpp::shutdown();
  return 0;
}
