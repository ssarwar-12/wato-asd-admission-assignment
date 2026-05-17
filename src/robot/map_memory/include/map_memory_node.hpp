#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
public:
  MapMemoryNode();

private:
  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void timerCallback();

  void initializeGlobalMap();
  void integrateCostmap();
  void publishMap();

  double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q) const;
  double distance(double x1, double y1, double x2, double y2) const;

  int toIndex(int x, int y, int width) const;
  bool inBounds(int x, int y, int width, int height) const;

  int global_width_;
  int global_height_;
  double resolution_;
  double origin_x_;
  double origin_y_;
  double update_distance_;

  bool has_costmap_;
  bool has_odom_;
  bool has_last_update_pose_;

  double last_update_x_;
  double last_update_y_;

  nav_msgs::msg::OccupancyGrid global_map_;
  nav_msgs::msg::OccupancyGrid latest_costmap_;
  nav_msgs::msg::Odometry latest_odom_;

  robot::MapMemoryCore map_memory_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif