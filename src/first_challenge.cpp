#include "first_challenge/first_challenge.hpp"

#include <cmath>
#include <functional>
using std::placeholders::_1;

FirstChallenge::FirstChallenge() : rclcpp::Node("first_challenge")
{
  hz_ = 10;
  goal_dist_ = 1.0;   // 1m
  velocity_  = 0.3;   // 0.1m/s  0.3に修正してみた

  // Pub: /cmd_vel
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  // Sub: /odom
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10, std::bind(&FirstChallenge::odometry_callback, this, _1));

  cmd_vel_ = geometry_msgs::msg::Twist();
}

void FirstChallenge::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  odom_ = *msg;
}

bool FirstChallenge::can_move()
{
  // odomを受け取れているか
  return odom_.has_value();
}

double FirstChallenge::calc_distance()
{
  if (!odom_.has_value()) return 0.0;

  const double x = odom_.value().pose.pose.position.x;
  const double y = odom_.value().pose.pose.position.y;

  static bool start_initialized = false;
  static double start_x = 0.0;
  static double start_y = 0.0;

  if (!start_initialized) {
    start_x = x;
    start_y = y;
    start_initialized = true;
    return 0.0;
  }

  return std::hypot(x - start_x, y - start_y);
}

bool FirstChallenge::is_goal()
{
  return calc_distance() >= goal_dist_;
}

void FirstChallenge::run(float velocity, float omega)
{
  cmd_vel_.linear.x  = velocity;
  cmd_vel_.linear.y  = 0.0;
  cmd_vel_.linear.z  = 0.0;
  cmd_vel_.angular.x = 0.0;
  cmd_vel_.angular.y = 0.0;
  cmd_vel_.angular.z = omega;

  cmd_vel_pub_->publish(cmd_vel_);
}

void FirstChallenge::set_cmd_vel()
{
  if (!can_move()) return;

  if (is_goal()) {
    run(0.0f, 0.0f);  // 1m到達 → 停止
  } else {
    run(static_cast<float>(velocity_), 0.0f); // 直進
  }
}
