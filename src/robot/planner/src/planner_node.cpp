#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

#include "planner_node.hpp"

PlannerNode::PlannerNode()
: Node("planner"),
  state_(State::WAITING_FOR_GOAL),
  has_map_(false),
  has_goal_(false),
  has_odom_(false),
  goal_tolerance_(0.5),
  planner_(robot::PlannerCore(this->get_logger()))
{
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map",
    10,
    std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1)
  );

  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point",
    10,
    std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1)
  );

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered",
    10,
    std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1)
  );

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000),
    std::bind(&PlannerNode::timerCallback, this)
  );

  RCLCPP_INFO(this->get_logger(), "Planner node initialized.");
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  current_map_ = *msg;
  has_map_ = true;

  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    planPath();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_ = *msg;
  has_goal_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;

  RCLCPP_INFO(
    this->get_logger(),
    "Received goal: x=%.2f, y=%.2f",
    goal_.point.x,
    goal_.point.y
  );

  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_pose_ = msg->pose.pose;
  has_odom_ = true;
}

void PlannerNode::timerCallback() {
  if (state_ != State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    return;
  }

  if (goalReached()) {
    RCLCPP_INFO(this->get_logger(), "Goal reached. Waiting for next goal.");
    state_ = State::WAITING_FOR_GOAL;
    has_goal_ = false;

    nav_msgs::msg::Path empty_path;
    empty_path.header.stamp = this->get_clock()->now();
    empty_path.header.frame_id = "sim_world";
    path_pub_->publish(empty_path);
    return;
  }

  planPath();
}

bool PlannerNode::goalReached() const {
  if (!has_goal_ || !has_odom_) {
    return false;
  }

  const double dx = goal_.point.x - robot_pose_.position.x;
  const double dy = goal_.point.y - robot_pose_.position.y;

  return std::sqrt(dx * dx + dy * dy) <= goal_tolerance_;
}

void PlannerNode::planPath() {
  if (!has_map_ || !has_goal_ || !has_odom_) {
    RCLCPP_WARN(
      this->get_logger(),
      "Cannot plan path yet. has_map=%d has_goal=%d has_odom=%d",
      has_map_,
      has_goal_,
      has_odom_
    );
    return;
  }

  if (current_map_.data.empty()) {
    RCLCPP_WARN(this->get_logger(), "Cannot plan path because map data is empty.");
    return;
  }

  const CellIndex start = worldToGrid(robot_pose_.position.x, robot_pose_.position.y);
  const CellIndex goal = worldToGrid(goal_.point.x, goal_.point.y);

  if (!inBounds(start)) {
    RCLCPP_WARN(this->get_logger(), "Start cell is outside map bounds.");
    return;
  }

  if (!inBounds(goal)) {
    RCLCPP_WARN(this->get_logger(), "Goal cell is outside map bounds.");
    return;
  }

  std::vector<CellIndex> grid_path = runAStar(start, goal);

  if (grid_path.empty()) {
    RCLCPP_WARN(this->get_logger(), "A* failed to find a path.");
    return;
  }

  nav_msgs::msg::Path path;
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = "sim_world";

  for (const CellIndex & cell : grid_path) {
    geometry_msgs::msg::PoseStamped pose = gridToPose(cell);
    pose.header.stamp = path.header.stamp;
    path.poses.push_back(pose);
  }

  path_pub_->publish(path);

  RCLCPP_INFO(this->get_logger(), "Published path with %zu poses.", path.poses.size());
}

std::vector<CellIndex> PlannerNode::runAStar(const CellIndex & start, const CellIndex & goal) {
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;

  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  std::unordered_set<CellIndex, CellIndexHash> closed_set;

  open_set.push(AStarNode(start, heuristic(start, goal)));
  g_score[start] = 0.0;

  const std::vector<CellIndex> directions = {
    CellIndex(1, 0),
    CellIndex(-1, 0),
    CellIndex(0, 1),
    CellIndex(0, -1),
    CellIndex(1, 1),
    CellIndex(1, -1),
    CellIndex(-1, 1),
    CellIndex(-1, -1)
  };

  while (!open_set.empty()) {
    const CellIndex current = open_set.top().index;
    open_set.pop();

    if (closed_set.find(current) != closed_set.end()) {
      continue;
    }

    if (current == goal) {
      return reconstructPath(came_from, current);
    }

    closed_set.insert(current);

    for (const CellIndex & direction : directions) {
      CellIndex neighbor(current.x + direction.x, current.y + direction.y);

      if (!inBounds(neighbor)) {
        continue;
      }

      if (!isCellTraversable(neighbor)) {
        continue;
      }

      if (closed_set.find(neighbor) != closed_set.end()) {
        continue;
      }

      const double tentative_g_score =
        g_score[current] + movementCost(current, neighbor);

      if (
        g_score.find(neighbor) == g_score.end() ||
        tentative_g_score < g_score[neighbor]
      ) {
        came_from[neighbor] = current;
        g_score[neighbor] = tentative_g_score;

        const double f_score = tentative_g_score + heuristic(neighbor, goal);
        open_set.push(AStarNode(neighbor, f_score));
      }
    }
  }

  return {};
}

std::vector<CellIndex> PlannerNode::reconstructPath(
  const std::unordered_map<CellIndex, CellIndex, CellIndexHash> & came_from,
  const CellIndex & current
) const {
  std::vector<CellIndex> path;
  CellIndex node = current;
  path.push_back(node);

  while (came_from.find(node) != came_from.end()) {
    node = came_from.at(node);
    path.push_back(node);
  }

  std::reverse(path.begin(), path.end());
  return path;
}

CellIndex PlannerNode::worldToGrid(double wx, double wy) const {
  const int gx = static_cast<int>(
    (wx - current_map_.info.origin.position.x) / current_map_.info.resolution
  );

  const int gy = static_cast<int>(
    (wy - current_map_.info.origin.position.y) / current_map_.info.resolution
  );

  return CellIndex(gx, gy);
}

geometry_msgs::msg::PoseStamped PlannerNode::gridToPose(const CellIndex & cell) const {
  geometry_msgs::msg::PoseStamped pose;

  pose.header.frame_id = "sim_world";

  pose.pose.position.x =
    current_map_.info.origin.position.x +
    (static_cast<double>(cell.x) + 0.5) * current_map_.info.resolution;

  pose.pose.position.y =
    current_map_.info.origin.position.y +
    (static_cast<double>(cell.y) + 0.5) * current_map_.info.resolution;

  pose.pose.position.z = 0.0;
  pose.pose.orientation.w = 1.0;

  return pose;
}

bool PlannerNode::inBounds(const CellIndex & cell) const {
  return (
    cell.x >= 0 &&
    cell.y >= 0 &&
    cell.x < static_cast<int>(current_map_.info.width) &&
    cell.y < static_cast<int>(current_map_.info.height)
  );
}

bool PlannerNode::isCellTraversable(const CellIndex & cell) const {
  const int index = toIndex(cell);

  if (index < 0 || index >= static_cast<int>(current_map_.data.size())) {
    return false;
  }

  const int8_t value = current_map_.data[index];

  if (value < 0) {
    return true;
  }

  return value <= 50;
}

int PlannerNode::toIndex(const CellIndex & cell) const {
  return cell.y * static_cast<int>(current_map_.info.width) + cell.x;
}

double PlannerNode::heuristic(const CellIndex & a, const CellIndex & b) const {
  const double dx = static_cast<double>(a.x - b.x);
  const double dy = static_cast<double>(a.y - b.y);
  return std::sqrt(dx * dx + dy * dy);
}

double PlannerNode::movementCost(const CellIndex & a, const CellIndex & b) const {
  const double dx = static_cast<double>(a.x - b.x);
  const double dy = static_cast<double>(a.y - b.y);
  return std::sqrt(dx * dx + dy * dy);
}

double PlannerNode::distance(double x1, double y1, double x2, double y2) const {
  const double dx = x1 - x2;
  const double dy = y1 - y2;
  return std::sqrt(dx * dx + dy * dy);
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}