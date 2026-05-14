#include "MPC.hpp"
#include "bspline_opt/uniform_bspline.h"

#include <ego_planner/msg/bspline.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Eigen>
#include <ctime>

#define PI 3.1415926
#define yaw_error_max 90.0 / 180 * PI
#define N 20

using ego_planner::UniformBspline;

rclcpp::Node::SharedPtr g_node;
rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_cmd_pub;

geometry_msgs::msg::TwistStamped cmd;
bool receive_traj_ = false;
bool is_orientation_init = false;
std::vector<UniformBspline> traj_;
double traj_duration_;
rclcpp::Time start_time_;
int traj_id_;

Eigen::Vector3d odom_pos_, odom_vel_;
Eigen::Quaterniond odom_orient_;

MPC_controller mpc_controller;
double roll, pitch, yaw;
tf2::Quaternion quat;
std_msgs::msg::UInt8 is_adjust_pose;
std_msgs::msg::UInt8 dir;

enum DIRECTION { POSITIVE = 0, NEGATIVE = 1 };

// yaw control
double t_step;

std_msgs::msg::UInt8 stop_command;

// time record
clock_t start_clock, end_clock;
double duration;

void bsplineCallback(const ego_planner::msg::Bspline::ConstSharedPtr msg)
{
  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());

  Eigen::VectorXd knots(msg->knots.size());
  for (size_t i = 0; i < msg->knots.size(); ++i)
  {
    knots(i) = msg->knots[i];
  }

  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  start_time_ = rclcpp::Time(msg->start_time);
  traj_id_ = msg->traj_id;

  traj_.clear();
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());

  traj_duration_ = traj_[0].getTimeSum();
  receive_traj_ = true;
}

void poseCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  tf2::fromMsg(msg->pose.pose.orientation, quat);
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}

void adjust_yaw_Callback(const std_msgs::msg::UInt8::ConstSharedPtr msg)
{
  is_adjust_pose = *msg;
}

void dirCallback(const std_msgs::msg::UInt8::ConstSharedPtr msg)
{
  dir = *msg;
}

void MPC_calculate(double &t_cur)
{
  std::vector<Eigen::Vector3d> X_r;
  std::vector<Eigen::Vector2d> U_r;
  Eigen::MatrixXd u_k;
  Eigen::Vector3d pos_r, pos_r_1, pos_final, v_r_1, v_r_2, X_k;
  Eigen::Vector2d u_r;
  Eigen::Vector3d x_r;
  double v_linear_1, w;
  double t_k, t_k_1;

  Eigen::Vector3d vel_start = traj_[1].evaluateDeBoor(t_cur);
  double yaw_start = atan2(vel_start(1), vel_start(0));
  (void)yaw_start;
  bool is_orientation_adjust = false;
  double orientation_adjust = 0;
  pos_final = traj_[0].evaluateDeBoor(traj_duration_);

  is_orientation_init = true;
  for (int i = 0; i < N; i++)
  {
    t_k = t_cur + i * t_step;
    t_k_1 = t_cur + (i + 1) * t_step;

    pos_r = traj_[0].evaluateDeBoor(t_k);
    pos_r_1 = traj_[0].evaluateDeBoor(t_k_1);

    x_r(0) = pos_r(0);
    x_r(1) = pos_r(1);

    v_r_1 = traj_[1].evaluateDeBoor(t_k);
    v_r_2 = traj_[1].evaluateDeBoor(t_k_1);
    v_r_1(2) = 0;
    v_r_2(2) = 0;
    v_linear_1 = v_r_1.norm();
    if ((t_k - traj_duration_) >= 0)
    {
      x_r(2) = atan2((pos_r - pos_final)(1), (pos_r - pos_final)(0));
    }
    else
    {
      x_r(2) = atan2(v_r_1(1), v_r_1(0));
    }

    double yaw1 = atan2(v_r_1(1), v_r_1(0));
    double yaw2 = atan2(v_r_2(1), v_r_2(0));

    if (abs(yaw2 - yaw1) > PI)
    {
      is_orientation_adjust = true;
      if ((yaw2 - yaw1) < 0)
      {
        orientation_adjust = 2 * PI;
        w = (2 * PI + (yaw2 - yaw1)) / t_step;
      }
      else
      {
        w = ((yaw2 - yaw1) - 2 * PI) / t_step;
        orientation_adjust = -2 * PI;
      }
    }
    else
    {
      w = (yaw2 - yaw1) / t_step;
    }

    if (is_orientation_adjust)
    {
      x_r(2) += orientation_adjust;
    }

    u_r(0) = v_linear_1;
    u_r(1) = w;
    X_r.push_back(x_r);
    U_r.push_back(u_r);
  }

  X_k(0) = odom_pos_(0);
  X_k(1) = odom_pos_(1);
  if (yaw / X_r[0](2) < 0 && abs(yaw) > (PI * 5 / 6))
  {
    if (yaw < 0)
    {
      X_k(2) = yaw + 2 * PI;
    }
    else
    {
      X_k(2) = yaw - 2 * PI;
    }
  }
  else
  {
    X_k(2) = yaw;
  }

  u_k = mpc_controller.MPC_Solve_qp(X_k, X_r, U_r, N);

  if (dir.data == NEGATIVE)
  {
    cmd.twist.linear.x = -u_k.col(0)(0);
  }
  else
  {
    cmd.twist.linear.x = u_k.col(0)(0);
  }

  cmd.twist.angular.z = u_k.col(0)(1);
}

void stopCallback(const std_msgs::msg::UInt8::ConstSharedPtr msg)
{
  stop_command = *msg;
}

void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  odom_pos_(0) = msg->pose.pose.position.x;
  odom_pos_(1) = msg->pose.pose.position.y;
  odom_pos_(2) = msg->pose.pose.position.z;

  odom_vel_(0) = msg->twist.twist.linear.x;
  odom_vel_(1) = msg->twist.twist.linear.y;
  odom_vel_(2) = msg->twist.twist.linear.z;

  odom_orient_.w() = msg->pose.pose.orientation.w;
  odom_orient_.x() = msg->pose.pose.orientation.x;
  odom_orient_.y() = msg->pose.pose.orientation.y;
  odom_orient_.z() = msg->pose.pose.orientation.z;

  

  
  // tf2::fromMsg(msg->pose.pose.orientation, quat);
  // tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  // if (dir.data == NEGATIVE)
  // {
  //   if (yaw > 0)
  //   {
  //     yaw -= PI;
  //   }
  //   else if (yaw < 0)
  //   {
  //     yaw += PI;
  //   }
  // }
}

void cmdCallback()
{
  if (stop_command.data == 1)
  {
    cmd.twist.angular.z = 0;
    cmd.twist.linear.x = 0;
    vel_cmd_pub->publish(cmd);
    return;
  }

  // During yaw-adjust phase, ownership of /cmd_vel belongs to ego_planner_node.
  // traj_server must stay silent to avoid command contention.
  if (is_adjust_pose.data == 1)
  {
    return;
  }

  if (!receive_traj_)
  {
    return;
  }

  rclcpp::Time time_s = g_node->now();
  double t_cur = (time_s - start_time_).seconds();

  static rclcpp::Time time_last = g_node->now();
  (void)time_last;

  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    start_clock = clock();
    MPC_calculate(t_cur);
    end_clock = clock();
    duration = (double)(end_clock - start_clock) / CLOCKS_PER_SEC * 1000;
    (void)duration;
  }
  else if (t_cur >= traj_duration_)
  {
    cmd.twist.angular.z = 0;
    cmd.twist.linear.x = 0;
    vel_cmd_pub->publish(cmd);
    is_orientation_init = false;
  }
  else
  {
    std::cout << "[Traj server]: invalid time." << std::endl;
    return;
  }

  time_last = time_s;
  vel_cmd_pub->publish(cmd);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("traj_server");
  g_node = node;

  std::string pose_topic, odom_topic;
  pose_topic = node->declare_parameter<std::string>("traj_server/pose_topic", "/odom_adjust");
  odom_topic = node->declare_parameter<std::string>("traj_server/odom_topic", "/state_estimation");

  auto bspline_sub = node->create_subscription<ego_planner::msg::Bspline>(
    "/planning/bspline", 10, bsplineCallback);
  auto pose_sub = node->create_subscription<nav_msgs::msg::Odometry>(
    pose_topic, 10, poseCallback);
  auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, 10, odometryCallback);
  auto stop_sub = node->create_subscription<std_msgs::msg::UInt8>(
    "/emergency_stop", 10, stopCallback);
  auto adjust_yaw_sub = node->create_subscription<std_msgs::msg::UInt8>(
    "/is_adjust_yaw", 10, adjust_yaw_Callback);
  auto command_sub = node->create_subscription<std_msgs::msg::UInt8>(
    "/direction", 10, dirCallback);

  mpc_controller.MPC_init(node);
  vel_cmd_pub = node->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 50);

  stop_command.data = 0;
  is_adjust_pose.data = 0;
  dir.data = POSITIVE;
  t_step = 0.03;

  auto cmd_timer = node->create_wall_timer(
    std::chrono::duration<double>(0.03), cmdCallback);

  rclcpp::sleep_for(std::chrono::seconds(1));
  RCLCPP_WARN(node->get_logger(), "[Traj server]: ready.");

  rclcpp::spin(node);
  rclcpp::shutdown();

  (void)bspline_sub;
  (void)pose_sub;
  (void)odom_sub;
  (void)stop_sub;
  (void)adjust_yaw_sub;
  (void)command_sub;
  (void)cmd_timer;

  return 0;
}
