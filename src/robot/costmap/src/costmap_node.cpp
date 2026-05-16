#include <algorithm>
#include <cmath>
#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode()
: Node("costmap"),
  width_(120),
  height_(120),
  resolution_(0.1),
  inflation_radius_(0.4),
  costmap_(robot::CostmapCore(this->get_logger()))
{
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar",
    10,
    std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1)
  );

  initializeCostmap();
}

void CostmapNode::initializeCostmap() {
  costmap_data_.assign(width_ * height_, 0);
}

void CostmapNode::markObstacle(int x, int y) {
  if (x < 0 || x >= width_ || y < 0 || y >= height_) {
    return;
  }

  costmap_data_[y * width_ + x] = 100;
}

void CostmapNode::inflateObstacles() {
  std::vector<int8_t> inflated = costmap_data_;
  int inflation_cells = static_cast<int>(inflation_radius_ / resolution_);

  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      if (costmap_data_[y * width_ + x] != 100) {
        continue;
      }

      for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
        for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
          int nx = x + dx;
          int ny = y + dy;

          if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) {
            continue;
          }

          double distance = std::sqrt(dx * dx + dy * dy) * resolution_;

          if (distance <= inflation_radius_) {
            int cost = static_cast<int>(100.0 * (1.0 - distance / inflation_radius_));
            int index = ny * width_ + nx;
            inflated[index] = std::max<int8_t>(
              inflated[index],
              static_cast<int8_t>(cost)
            );
          }
        }
      }
    }
  }

  costmap_data_ = inflated;
}

void CostmapNode::publishCostmap() {
  nav_msgs::msg::OccupancyGrid msg;

  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = "robot/chassis/lidar";

  msg.info.resolution = resolution_;
  msg.info.width = width_;
  msg.info.height = height_;

  msg.info.origin.position.x = -static_cast<double>(width_) * resolution_ / 2.0;
  msg.info.origin.position.y = -static_cast<double>(height_) * resolution_ / 2.0;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.w = 1.0;

  msg.data = costmap_data_;

  costmap_pub_->publish(msg);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  initializeCostmap();

  int center_x = width_ / 2;
  int center_y = height_ / 2;

  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double range = scan->ranges[i];

    if (!std::isfinite(range)) {
      continue;
    }

    if (range < scan->range_min || range > scan->range_max) {
      continue;
    }

    double angle = scan->angle_min + static_cast<double>(i) * scan->angle_increment;

    double x = range * std::cos(angle);
    double y = range * std::sin(angle);

    int grid_x = center_x + static_cast<int>(x / resolution_);
    int grid_y = center_y + static_cast<int>(y / resolution_);

    markObstacle(grid_x, grid_y);
  }

  inflateObstacles();
  publishCostmap();
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}