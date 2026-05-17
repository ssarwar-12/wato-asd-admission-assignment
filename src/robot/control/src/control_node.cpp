#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>

#include "control_node.hpp"

ControlNode::ControlNode()
: Node("control"),
  lookahead_distance_(0.8),
  goal_tolerance_(0.35),
  linear_speed_(0.25),
  angular_gain_(1.5),
  max_angular_speed_(1.0),
  has_path_(false),
  has_odom_(false),
  control_(robot::ControlCore(this->get_logger()))
{
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path",
    10,
    std::bind(&ControlNode::pathCallback, this, std::placeholders::_1)
  );

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered",
    10,
    std::bind(&ControlNode::odomCallback, this, std::placeholders::_1)
  );

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&ControlNode::controlLoop, this)
  );

  RCLCPP_INFO(this->get_logger(), "Control node initialized.");
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  current_path_ = *msg;
  has_path_ = true;

  if (current_path_.poses.empty()) {
    publishStop();
  }
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  current_odom_ = *msg;
  has_odom_ = true;
}

void ControlNode::controlLoop() {
  if (!has_path_ || !has_odom_) {
    return;
  }

  if (current_path_.poses.empty()) {
    publishStop();
    return;
  }

  if (reachedFinalWaypoint()) {
    publishStop();
    return;
  }

  const std::optional<geometry_msgs::msg::PoseStamped> lookahead_point =
    findLookaheadPoint();

  if (!lookahead_point.has_value()) {
    publishStop();
    return;
  }

  geometry_msgs::msg::Twist cmd = computeVelocityCommand(lookahead_point.value());
  cmd_vel_pub_->publish(cmd);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() const {
  if (current_path_.poses.empty()) {
    return std::nullopt;
  }

  const double robot_x = current_odom_.pose.pose.position.x;
  const double robot_y = current_odom_.pose.pose.position.y;

  for (const auto & pose : current_path_.poses) {
    const double px = pose.pose.position.x;
    const double py = pose.pose.position.y;

    if (distance(robot_x, robot_y, px, py) >= lookahead_distance_) {
      return pose;
    }
  }

  return current_path_.poses.back();
}

geometry_msgs::msg::Twist ControlNode::computeVelocityCommand(
  const geometry_msgs::msg::PoseStamped & target
) const {
  geometry_msgs::msg::Twist cmd;

  const double robot_x = current_odom_.pose.pose.position.x;
  const double robot_y = current_odom_.pose.pose.position.y;
  const double robot_yaw = yawFromQuaternion(current_odom_.pose.pose.orientation);

  const double target_x = target.pose.position.x;
  const double target_y = target.pose.position.y;

  const double target_angle = std::atan2(target_y - robot_y, target_x - robot_x);
  const double angle_error = normalizeAngle(target_angle - robot_yaw);

  double angular_velocity = angular_gain_ * angle_error;
  angular_velocity = std::clamp(
    angular_velocity,
    -max_angular_speed_,
    max_angular_speed_
  );

  double forward_speed = linear_speed_;

  if (std::abs(angle_error) > 1.0) {
    forward_speed = 0.05;
  }

  cmd.linear.x = forward_speed;
  cmd.angular.z = angular_velocity;

  return cmd;
}

void ControlNode::publishStop() {
  geometry_msgs::msg::Twist stop;
  cmd_vel_pub_->publish(stop);
}

bool ControlNode::reachedFinalWaypoint() const {
  if (current_path_.poses.empty()) {
    return true;
  }

  const auto & final_pose = current_path_.poses.back();

  const double robot_x = current_odom_.pose.pose.position.x;
  const double robot_y = current_odom_.pose.pose.position.y;

  const double goal_x = final_pose.pose.position.x;
  const double goal_y = final_pose.pose.position.y;

  return distance(robot_x, robot_y, goal_x, goal_y) <= goal_tolerance_;
}

double ControlNode::yawFromQuaternion(const geometry_msgs::msg::Quaternion & q) const {
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double ControlNode::normalizeAngle(double angle) const {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }

  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }

  return angle;
}

double ControlNode::distance(double x1, double y1, double x2, double y2) const {
  const double dx = x1 - x2;
  const double dy = y1 - y2;
  return std::sqrt(dx * dx + dy * dy);
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}