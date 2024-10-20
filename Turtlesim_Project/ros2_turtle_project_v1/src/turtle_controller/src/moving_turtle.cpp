#include <chrono>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class MovingTurtle : public rclcpp::Node
{
public:
  MovingTurtle() : Node("moving_turtle"), count_(0)
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    timer_ = this->create_wall_timer(
      10ms, std::bind(&MovingTurtle::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = geometry_msgs::msg::Twist();
    
    if (count_ < 100) {
      message.linear.x = 2.0;
      message.angular.z = 0.0;
    } else if (count_ < 200) {
      message.linear.x = 0.0;
      message.angular.z = 1.0;
    } else {
      count_ = 0;
    }

    publisher_->publish(message);
    count_++;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MovingTurtle>());
  rclcpp::shutdown();
  return 0;
}
