#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

class JieTwistStamper : public rclcpp::Node
{
public:
  JieTwistStamper()
  : Node("jie_twist_stamper")
  {
    const auto input_topic = declare_parameter<std::string>("input_topic", "/cmd_vel_jie");
    const auto output_topic = declare_parameter<std::string>("output_topic", "/cmd_vel");
    frame_id_ = declare_parameter<std::string>("frame_id", "base_link");

    publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>(output_topic, 10);
    subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      input_topic, 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        geometry_msgs::msg::TwistStamped stamped;
        stamped.header.stamp = now();
        stamped.header.frame_id = frame_id_;
        stamped.twist = *msg;
        publisher_->publish(stamped);
      });

    RCLCPP_INFO(
      get_logger(), "JIE velocity adapter started: %s (Twist) -> %s (TwistStamped)",
      input_topic.c_str(), output_topic.c_str());
  }

private:
  std::string frame_id_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JieTwistStamper>());
  rclcpp::shutdown();
  return 0;
}
