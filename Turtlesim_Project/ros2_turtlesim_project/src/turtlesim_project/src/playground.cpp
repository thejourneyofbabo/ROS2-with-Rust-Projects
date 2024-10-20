#include <chrono>
#include <memory>
#include <rclcpp/rclcp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class MovingTurtle : public rclcpp::Node
{
  public:
    MovingTurtle() : Node("moving_turtle"), loop_count_(0)
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&MovingTurtle::timer_callback, this));
  }

  private:
    void timer_callback()
    {
      auto message = geometry_msgs::msg::Twist();

      if (loop_count_ < 100) {
        message.linear.x = 2.0;
        message.angular.z = 0.0;
      } else if (loop_count_ < 200) {
        message.linear.x = 0.0;
        message.angular.z = 1.0;
      } else {
        loop_count_ = 0;
      }

      publisher_->publish(message);
      loop_count_++;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int loop_count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MovingTurtle>());
  rclcpp::shutdown();
  return 0;
}

