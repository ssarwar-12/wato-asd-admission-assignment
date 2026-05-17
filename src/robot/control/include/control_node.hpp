#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include <optional>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "control_core.hpp"

class ControlNode : public rclcpp::Node {
public:
  ControlNode();

private:
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void controlLoop();

  std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint() const;
  geometry_msgs::msg::Twist computeVelocityCommand(
    const geometry_msgs::msg::PoseStamped & target
  ) const;

  void publishStop();

  double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q) const;
  double normalizeAngle(double angle) const;
  double distance(double x1, double y1, double x2, double y2) const;

  bool reachedFinalWaypoint() const;

  double lookahead_distance_;
  double goal_tolerance_;
  double linear_speed_;
  double angular_gain_;
  double max_angular_speed_;

  bool has_path_;
  bool has_odom_;

  nav_msgs::msg::Path current_path_;
  nav_msgs::msg::Odometry current_odom_;

  robot::ControlCore control_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif