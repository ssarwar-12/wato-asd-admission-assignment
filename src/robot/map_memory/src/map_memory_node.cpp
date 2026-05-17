#include <algorithm>
#include <cmath>
#include <memory>

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode()
: Node("map_memory"),
  global_width_(400),
  global_height_(400),
  resolution_(0.1),
  origin_x_(-20.0),
  origin_y_(-20.0),
  update_distance_(0.5),
  has_costmap_(false),
  has_odom_(false),
  has_last_update_pose_(false),
  last_update_x_(0.0),
  last_update_y_(0.0),
  map_memory_(robot::MapMemoryCore(this->get_logger()))
{
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap",
    10,
    std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1)
  );

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered",
    10,
    std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1)
  );

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&MapMemoryNode::timerCallback, this)
  );

  initializeGlobalMap();
  publishMap();

  RCLCPP_INFO(this->get_logger(), "Map memory node initialized and published initial map.");
}

void MapMemoryNode::initializeGlobalMap() {
  global_map_.header.frame_id = "map";

  global_map_.info.resolution = resolution_;
  global_map_.info.width = global_width_;
  global_map_.info.height = global_height_;

  global_map_.info.origin.position.x = origin_x_;
  global_map_.info.origin.position.y = origin_y_;
  global_map_.info.origin.position.z = 0.0;
  global_map_.info.origin.orientation.w = 1.0;

  global_map_.data.assign(global_width_ * global_height_, -1);
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = *msg;
  has_costmap_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  latest_odom_ = *msg;
  has_odom_ = true;
}

void MapMemoryNode::timerCallback() {
  if (!has_costmap_ || !has_odom_) {
    publishMap();
    return;
  }

  const double robot_x = latest_odom_.pose.pose.position.x;
  const double robot_y = latest_odom_.pose.pose.position.y;

  if (!has_last_update_pose_) {
    last_update_x_ = robot_x;
    last_update_y_ = robot_y;
    has_last_update_pose_ = true;

    integrateCostmap();
    publishMap();
    return;
  }

  const double moved = distance(robot_x, robot_y, last_update_x_, last_update_y_);

  if (moved >= update_distance_) {
    integrateCostmap();

    last_update_x_ = robot_x;
    last_update_y_ = robot_y;
  }

  publishMap();
}

void MapMemoryNode::integrateCostmap() {
  if (latest_costmap_.data.empty()) {
    return;
  }

  const double robot_x = latest_odom_.pose.pose.position.x;
  const double robot_y = latest_odom_.pose.pose.position.y;
  const double robot_yaw = yawFromQuaternion(latest_odom_.pose.pose.orientation);

  const int local_width = static_cast<int>(latest_costmap_.info.width);
  const int local_height = static_cast<int>(latest_costmap_.info.height);
  const double local_resolution = latest_costmap_.info.resolution;

  const double local_origin_x = latest_costmap_.info.origin.position.x;
  const double local_origin_y = latest_costmap_.info.origin.position.y;

  for (int ly = 0; ly < local_height; ++ly) {
    for (int lx = 0; lx < local_width; ++lx) {
      const int local_index = toIndex(lx, ly, local_width);
      const int8_t local_value = latest_costmap_.data[local_index];

      if (local_value < 0) {
        continue;
      }

      const double local_x = local_origin_x + (static_cast<double>(lx) + 0.5) * local_resolution;
      const double local_y = local_origin_y + (static_cast<double>(ly) + 0.5) * local_resolution;

      const double world_x =
        robot_x + std::cos(robot_yaw) * local_x - std::sin(robot_yaw) * local_y;

      const double world_y =
        robot_y + std::sin(robot_yaw) * local_x + std::cos(robot_yaw) * local_y;

      const int gx = static_cast<int>((world_x - origin_x_) / resolution_);
      const int gy = static_cast<int>((world_y - origin_y_) / resolution_);

      if (!inBounds(gx, gy, global_width_, global_height_)) {
        continue;
      }

      const int global_index = toIndex(gx, gy, global_width_);

      if (global_map_.data[global_index] < 0) {
        global_map_.data[global_index] = local_value;
      } else {
        global_map_.data[global_index] = std::max<int8_t>(
          global_map_.data[global_index],
          local_value
        );
      }
    }
  }
}

void MapMemoryNode::publishMap() {
  global_map_.header.stamp = this->get_clock()->now();
  global_map_.header.frame_id = "map";
  map_pub_->publish(global_map_);
}

double MapMemoryNode::yawFromQuaternion(const geometry_msgs::msg::Quaternion & q) const {
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double MapMemoryNode::distance(double x1, double y1, double x2, double y2) const {
  const double dx = x1 - x2;
  const double dy = y1 - y2;
  return std::sqrt(dx * dx + dy * dy);
}

int MapMemoryNode::toIndex(int x, int y, int width) const {
  return y * width + x;
}

bool MapMemoryNode::inBounds(int x, int y, int width, int height) const {
  return x >= 0 && x < width && y >= 0 && y < height;
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}