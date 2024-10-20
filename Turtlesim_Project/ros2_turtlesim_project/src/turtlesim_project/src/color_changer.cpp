#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>
#include <custom_interfaces/msg/turtle_color.hpp>
#include <custom_interfaces/msg/sphere.hpp>
#include <custom_interfaces/msg/turtle_state.hpp>
#include <geometry_msgs/msg/point.hpp>

class ColorChanger : public rclcpp::Node
{
public:
  ColorChanger() : Node("color_changer")
  {
    subscription_ = this->create_subscription<turtlesim::msg::Pose>(
      "/turtle1/pose", 10, std::bind(&ColorChanger::pose_callback, this, std::placeholders::_1));
    
    color_publisher_ = this->create_publisher<custom_interfaces::msg::TurtleColor>("/turtle_color", 10);
    sphere_publisher_ = this->create_publisher<custom_interfaces::msg::Sphere>("/turtle_sphere", 10);
    state_publisher_ = this->create_publisher<custom_interfaces::msg::TurtleState>("/turtle_state", 10);
    
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ColorChanger::timer_callback, this));

    param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "/turtlesim");
    
    while (!param_client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        rclcpp::shutdown();
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Parameter service not available, waiting again...");
    }
    RCLCPP_INFO(this->get_logger(), "Parameter service connected successfully.");
  }

private:
  void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
  {
    current_pose_ = *msg;
  }

  void timer_callback()
  {
    publish_turtle_color();
    publish_sphere();
    publish_turtle_state();

    /*RCLCPP_INFO(this->get_logger(), "Published custom messages");*/
  }

  void publish_turtle_color()
  {
    auto color_msg = custom_interfaces::msg::TurtleColor();

    if (current_pose_.theta >= 0.0 && current_pose_.theta < M_PI_2) {
      set_background_color(255, 255, 255);
      color_msg.red = 1.0; color_msg.green = 1.0; color_msg.blue = 1.0;
    } else if (current_pose_.theta >= M_PI_2 && current_pose_.theta < M_PI) {
      set_background_color(255, 0, 0);
      color_msg.red = 1.0; color_msg.green = 0.0; color_msg.blue = 0.0;
    } else if (current_pose_.theta >= -M_PI && current_pose_.theta < -M_PI_2) {
      set_background_color(0, 255, 0);
      color_msg.red = 0.0; color_msg.green = 1.0; color_msg.blue = 0.0;
    } else {
      set_background_color(0, 0, 255);
      color_msg.red = 0.0; color_msg.green = 0.0; color_msg.blue = 1.0;
    }

    color_publisher_->publish(color_msg);
  }

  void publish_sphere()
  {
    auto sphere_msg = custom_interfaces::msg::Sphere();
    sphere_msg.center.x = current_pose_.x;
    sphere_msg.center.y = current_pose_.y;
    sphere_msg.center.z = 0.0;  // Assuming 2D space
    sphere_msg.radius = 0.5;  // Arbitrary radius, adjust as needed

    sphere_publisher_->publish(sphere_msg);
  }

  void publish_turtle_state()
  {
    auto state_msg = custom_interfaces::msg::TurtleState();
    state_msg.x_velocity.data = current_pose_.linear_velocity;
    state_msg.yaw_rate.data = current_pose_.angular_velocity;

    state_publisher_->publish(state_msg);
  }

  void set_background_color(int r, int g, int b)
  {
    if (!param_client_->service_is_ready()) {
      RCLCPP_WARN(this->get_logger(), "Parameter service is not ready. Cannot set background color.");
      return;
    }

    auto callback = [this](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
      auto results = future.get();
      for (const auto& result : results) {
        if (!result.successful) {
          RCLCPP_ERROR(this->get_logger(), "Failed to set parameter: %s", result.reason.c_str());
        }
      }
    };

    param_client_->set_parameters({
      rclcpp::Parameter("background_r", r),
      rclcpp::Parameter("background_g", g),
      rclcpp::Parameter("background_b", b)
    }, callback);
  }

  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
  rclcpp::Publisher<custom_interfaces::msg::TurtleColor>::SharedPtr color_publisher_;
  rclcpp::Publisher<custom_interfaces::msg::Sphere>::SharedPtr sphere_publisher_;
  rclcpp::Publisher<custom_interfaces::msg::TurtleState>::SharedPtr state_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<rclcpp::AsyncParametersClient> param_client_;
  turtlesim::msg::Pose current_pose_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ColorChanger>());
  rclcpp::shutdown();
  return 0;
}
