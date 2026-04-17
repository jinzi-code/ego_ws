#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <algorithm>

class CmdVelBridge : public rclcpp::Node
{
public:
  CmdVelBridge()
  : Node("cmd_vel_bridge_node")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/cmd_vel");
    output_topic_ = declare_parameter<std::string>("output_topic", "/cmd_vel1");
    queue_size_ = declare_parameter<int>("queue_size", 50);
    max_abs_linear_x_ = declare_parameter<double>("max_abs_linear_x", 0.02);
    max_abs_angular_z_ = declare_parameter<double>("max_abs_angular_z", 0.02);

    pub_ = create_publisher<geometry_msgs::msg::Twist>(output_topic_, queue_size_);
    sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      input_topic_, queue_size_,
      std::bind(&CmdVelBridge::cmdCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "[cmd_vel_bridge] Subscribe: %s -> Publish: %s | |linear.x|<=%.3f, |angular.z|<=%.3f",
      input_topic_.c_str(), output_topic_.c_str(), max_abs_linear_x_, max_abs_angular_z_);
  }

private:
  double saturate(double value, double max_abs)
  {
    return std::max(-max_abs, std::min(value, max_abs));
  }

  void cmdCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
  {
    geometry_msgs::msg::Twist out;
    out = msg->twist;
    out.linear.x = saturate(out.linear.x, max_abs_linear_x_);
    out.angular.z = saturate(out.angular.z, max_abs_angular_z_);
    pub_->publish(out);
  }

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;

  std::string input_topic_;
  std::string output_topic_;
  int queue_size_;
  double max_abs_linear_x_;
  double max_abs_angular_z_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto bridge = std::make_shared<CmdVelBridge>();
  rclcpp::spin(bridge);
  rclcpp::shutdown();
  return 0;
}
