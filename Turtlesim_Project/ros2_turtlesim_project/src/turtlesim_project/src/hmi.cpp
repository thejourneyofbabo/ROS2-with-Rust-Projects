#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>
#include <custom_interfaces/msg/turtle_color.hpp>
#include <custom_interfaces/msg/sphere.hpp>
#include <custom_interfaces/msg/turtle_state.hpp>
#include <visualization_msgs/msg/marker.hpp>

class HMI : public rclcpp::Node
{
public:
  HMI() : Node("hmi")
  {
    pose_subscription_ = this->create_subscription<turtlesim::msg::Pose>(
      "/turtle1/pose", 10, std::bind(&HMI::pose_callback, this, std::placeholders::_1));
    
    color_subscription_ = this->create_subscription<custom_interfaces::msg::TurtleColor>(
      "/turtle_color", 10, std::bind(&HMI::color_callback, this, std::placeholders::_1));
    
    sphere_subscription_ = this->create_subscription<custom_interfaces::msg::Sphere>(
      "/turtle_sphere", 10, std::bind(&HMI::sphere_callback, this, std::placeholders::_1));
    
    state_subscription_ = this->create_subscription<custom_interfaces::msg::TurtleState>(
      "/turtle_state", 10, std::bind(&HMI::state_callback, this, std::placeholders::_1));
    
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/turtle_marker", 10);
    
    timer_ = this->create_wall_timer(std::chrono::milliseconds(16), std::bind(&HMI::timer_callback, this));
  }

private:
  void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
  {
    current_pose_ = *msg;
  }

  void color_callback(const custom_interfaces::msg::TurtleColor::SharedPtr msg)
  {
    current_color_ = *msg;
  }

  void sphere_callback(const custom_interfaces::msg::Sphere::SharedPtr msg)
  {
    current_sphere_ = *msg;
  }

  void state_callback(const custom_interfaces::msg::TurtleState::SharedPtr msg)
  {
    current_state_ = *msg;
  }

  void timer_callback()
  {
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    /*marker.type = visualization_msgs::msg::Marker::SPHERE;*/
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    marker.pose.position.x = current_sphere_.center.x;
    marker.pose.position.y = current_sphere_.center.y;
    marker.pose.position.z = current_sphere_.center.z;
    
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    
    marker.scale.x = current_sphere_.radius * 2;
    marker.scale.y = current_sphere_.radius * 2;
    marker.scale.z = current_sphere_.radius * 2;
    
    marker.color.r = current_color_.red;
    marker.color.g = current_color_.green;
    marker.color.b = current_color_.blue;
    marker.color.a = 1.0;
    
    marker_publisher_->publish(marker);

    /*RCLCPP_INFO(this->get_logger(), "Turtle State - X Velocity: %f, Yaw Rate: %f",*/
    /*            current_state_.x_velocity.data, current_state_.yaw_rate.data);*/
  }

  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscription_;
  rclcpp::Subscription<custom_interfaces::msg::TurtleColor>::SharedPtr color_subscription_;
  rclcpp::Subscription<custom_interfaces::msg::Sphere>::SharedPtr sphere_subscription_;
  rclcpp::Subscription<custom_interfaces::msg::TurtleState>::SharedPtr state_subscription_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  turtlesim::msg::Pose current_pose_;
  custom_interfaces::msg::TurtleColor current_color_;
  custom_interfaces::msg::Sphere current_sphere_;
  custom_interfaces::msg::TurtleState current_state_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HMI>());
  rclcpp::shutdown();
  return 0;
}
