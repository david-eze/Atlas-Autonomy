#include "robot_planners/astar_planner.hpp"
#include "robot_planners/grid_search.hpp"

#include <cmath>
#include <utility>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace robot_planners
{

void AstarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer>,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  name_ = std::move(name);
  costmap_ros_ = costmap_ros;
  node_ = parent.lock();

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".tolerance", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".allow_unknown", rclcpp::ParameterValue(true));

  tolerance_ = node_->get_parameter(name_ + ".tolerance").as_double();
  allow_unknown_ = node_->get_parameter(name_ + ".allow_unknown").as_bool();

  explored_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    name_ + "/explored_nodes", 1);
}

void AstarPlanner::cleanup()
{
  explored_pub_.reset();
}

void AstarPlanner::activate()
{
  explored_pub_->on_activate();
}

void AstarPlanner::deactivate()
{
  explored_pub_->on_deactivate();
}

nav_msgs::msg::Path AstarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header = goal.header;
  path.header.frame_id = costmap_ros_->getGlobalFrameID();

  costmap_ = costmap_ros_->getCostmap();
  if (!costmap_) {
    RCLCPP_WARN(node_->get_logger(), "No costmap available for A* planning");
    return path;
  }

  const double wx = costmap_->getOriginX();
  const double wy = costmap_->getOriginY();
  const double res = costmap_->getResolution();

  const int start_x = static_cast<int>(std::floor((start.pose.position.x - wx) / res));
  const int start_y = static_cast<int>(std::floor((start.pose.position.y - wy) / res));
  const int goal_x = static_cast<int>(std::floor((goal.pose.position.x - wx) / res));
  const int goal_y = static_cast<int>(std::floor((goal.pose.position.y - wy) / res));

  const int width = costmap_->getSizeInCellsX();
  const int height = costmap_->getSizeInCellsY();

  const auto cost_at = [this](int x, int y) -> double {
    const unsigned char cost = costmap_->getCost(x, y);
    if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
      return -1.0;
    }
    if (cost == nav2_costmap_2d::NO_INFORMATION && !allow_unknown_) {
      return -1.0;
    }
    return static_cast<double>(cost) / 252.0;
  };

  const double goal_scale = costmap_->getResolution();

  GridSearchResult result = grid_search(
    width, height, start_x, start_y, goal_x, goal_y, cost_at, true);

  // Publish the path cells as an approximation of the explored set;
  // the expensive full node set stays in the benchmark pipeline.
  publish_explored_nodes(result.path, path.header);

  if (!result.success) {
    RCLCPP_WARN(node_->get_logger(), "A* failed to find a path");
    return path;
  }

  for (const auto & [cx, cy] : result.path) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = wx + (static_cast<double>(cx) + 0.5) * res;
    pose.pose.position.y = wy + (static_cast<double>(cy) + 0.5) * res;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "A*: success, %zu nodes expanded, path length %.2f m",
    result.nodes_expanded, result.path_length_cells * goal_scale);

  return path;
}

void AstarPlanner::publish_explored_nodes(
  const std::vector<std::pair<int, int>> & nodes,
  const std_msgs::msg::Header & header)
{
  if (!nodes.empty() && explored_pub_->is_activated()) {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = "explored_nodes";
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.color.r = 0.2f;
    marker.color.g = 0.6f;
    marker.color.b = 0.9f;
    marker.color.a = 0.5f;
    marker.points.reserve(nodes.size());
    const double wx = costmap_->getOriginX();
    const double wy = costmap_->getOriginY();
    const double res = costmap_->getResolution();
    for (const auto & [cx, cy] : nodes) {
      geometry_msgs::msg::Point p;
      p.x = wx + (static_cast<double>(cx) + 0.5) * res;
      p.y = wy + (static_cast<double>(cy) + 0.5) * res;
      marker.points.push_back(p);
    }
    markers.markers.push_back(marker);
    explored_pub_->publish(markers);
  }
}

}  // namespace robot_planners

PLUGINLIB_EXPORT_CLASS(robot_planners::AstarPlanner, nav2_core::GlobalPlanner)
