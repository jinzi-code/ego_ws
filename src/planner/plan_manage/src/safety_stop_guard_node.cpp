#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>

class SafetyStopGuard : public rclcpp::Node
{
public:
  SafetyStopGuard()
  : Node("safety_stop_guard_node")
  {
    watch_topic_ = declare_parameter<std::string>("watch_topic", "/cmd_vel");
    stop_cmd_topic_ = declare_parameter<std::string>("stop_cmd_topic", "/cmd_vel");
    stop_cmd_unstamped_topic_ = declare_parameter<std::string>("stop_cmd_unstamped_topic", "/cmd_vel1");
    manual_stop_topic_ = declare_parameter<std::string>("manual_stop_topic", "/safety_stop");
    emergency_topic_ = declare_parameter<std::string>("emergency_topic", "/emergency_stop");

    watchdog_timeout_ = declare_parameter<double>("watchdog_timeout", 0.4);
    publish_rate_ = declare_parameter<double>("publish_rate", 30.0);
    stop_on_start_ = declare_parameter<bool>("stop_on_start", true);
    arm_after_first_cmd_ = declare_parameter<bool>("arm_after_first_cmd", true);

    cmd_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      watch_topic_, 50, std::bind(&SafetyStopGuard::cmdCallback, this, std::placeholders::_1));
    manual_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
      manual_stop_topic_, 10, std::bind(&SafetyStopGuard::manualStopCallback, this, std::placeholders::_1));

    stop_cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(stop_cmd_topic_, 20);
    stop_cmd_unstamped_pub_ = create_publisher<geometry_msgs::msg::Twist>(stop_cmd_unstamped_topic_, 20);
    emergency_pub_ = create_publisher<std_msgs::msg::UInt8>(emergency_topic_, 20);

    last_cmd_time_ = now();
    manual_stop_ = stop_on_start_;

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_),
      std::bind(&SafetyStopGuard::timerCallback, this));

    RCLCPP_WARN(get_logger(), "[safety_stop_guard] started. watch=%s timeout=%.3fs manual_stop_topic=%s",
                watch_topic_.c_str(), watchdog_timeout_, manual_stop_topic_.c_str());
    RCLCPP_WARN(get_logger(), "[safety_stop_guard] stop outputs: %s and %s | emergency: %s",
                stop_cmd_topic_.c_str(), stop_cmd_unstamped_topic_.c_str(), emergency_topic_.c_str());

  }

private:
  void cmdCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr)
  {
    last_cmd_time_ = now();
    has_received_cmd_ = true;
    if (stop_active_ && !manual_stop_)
      clearStopState();
  }

  void manualStopCallback(const std_msgs::msg::Bool::ConstSharedPtr msg)
  {
    manual_stop_ = msg->data;
    RCLCPP_WARN(get_logger(), "[safety_stop_guard] manual stop: %s", manual_stop_ ? "ON" : "OFF");
    if (!manual_stop_)
    {
      // allow trajectory server to resume
      publishEmergency(0);
      emergency_stop_latched_ = false;
    }
  }

  void timerCallback()
  {
    const bool timeout = (now() - last_cmd_time_).seconds() > watchdog_timeout_;
    const bool watchdog_armed = !arm_after_first_cmd_ || has_received_cmd_;

    if (manual_stop_ || (watchdog_armed && timeout))
    {
      stop_active_ = true;
      publishStop(true);
      return;
    }

    if (stop_active_)
      clearStopState();
  }

  void clearStopState()
  {
    publishEmergency(0);
    stop_active_ = false;
    emergency_stop_latched_ = false;
  }

  void publishEmergency(uint8_t value)
  {
    std_msgs::msg::UInt8 msg;
    msg.data = value;
    emergency_pub_->publish(msg);
  }

  void publishStop(bool emergency)
  {
    geometry_msgs::msg::TwistStamped stop_stamped;
    stop_stamped.header.stamp = now();
    stop_stamped.twist.linear.x = 0.0;
    stop_stamped.twist.linear.y = 0.0;
    stop_stamped.twist.linear.z = 0.0;
    stop_stamped.twist.angular.x = 0.0;
    stop_stamped.twist.angular.y = 0.0;
    stop_stamped.twist.angular.z = 0.0;

    geometry_msgs::msg::Twist stop_unstamped;
    stop_unstamped.linear.x = 0.0;
    stop_unstamped.linear.y = 0.0;
    stop_unstamped.linear.z = 0.0;
    stop_unstamped.angular.x = 0.0;
    stop_unstamped.angular.y = 0.0;
    stop_unstamped.angular.z = 0.0;

    stop_cmd_pub_->publish(stop_stamped);
    stop_cmd_unstamped_pub_->publish(stop_unstamped);

    if (emergency)
    {
      publishEmergency(1);
      emergency_stop_latched_ = true;
    }
    else if (emergency_stop_latched_)
    {
      publishEmergency(0);
      emergency_stop_latched_ = false;
    }
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manual_stop_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr stop_cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr stop_cmd_unstamped_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr emergency_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string watch_topic_;
  std::string stop_cmd_topic_;
  std::string stop_cmd_unstamped_topic_;
  std::string manual_stop_topic_;
  std::string emergency_topic_;

  double watchdog_timeout_;
  double publish_rate_;
  bool stop_on_start_;
  bool arm_after_first_cmd_;

  bool manual_stop_{false};
  bool emergency_stop_latched_{false};
  bool has_received_cmd_{false};
  bool stop_active_{false};
  rclcpp::Time last_cmd_time_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto guard = std::make_shared<SafetyStopGuard>();
  rclcpp::spin(guard);
  rclcpp::shutdown();
  return 0;
}
