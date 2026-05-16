#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
public:
  CostmapNode();

private:
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
  void initializeCostmap();
  void markObstacle(int x, int y);
  void inflateObstacles();
  void publishCostmap();

  int width_;
  int height_;
  double resolution_;
  double inflation_radius_;

  std::vector<int8_t> costmap_data_;

  robot::CostmapCore costmap_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
};

#endif