#include "robot_planners/dijkstra_planner.hpp"
#include "robot_planners/grid_search.hpp"

#include <cmath>
#include <utility>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace robot_planners
{

void DijkstraPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer>,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  name_ = std::move(name);
  costmap_ros_ = costmap_ros;
  node_ = parent.lock();

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".allow_unknown", rclcpp::ParameterValue(true));
  allow_unknown_ = node_->get_parameter(name_ + ".allow_unknown").as_bool();

  explored_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    name_ + "/explored_nodes", 1);
}

void DijkstraPlanner::cleanup()
{
  explored_pub_.reset();
}

void DijkstraPlanner::activate()
{
  explored_pub_->on_activate();
}

void DijkstraPlanner::deactivate()
{
  explored_pub_->on_deactivate();
}

nav_msgs::msg::Path DijkstraPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header = goal.header;
  path.header.frame_id = costmap_ros_->getGlobalFrameID();

  auto * costmap = costmap_ros_->getCostmap();
  if (!costmap) {
    RCLCPP_WARN(node_->get_logger(), "No costmap available for Dijkstra planning");
    return path;
  }

  const double wx = costmap->getOriginX();
  const double wy = costmap->getOriginY();
  const double res = costmap->getResolution();

  const int start_x = static_cast<int>(std::floor((start.pose.position.x - wx) / res));
  const int start_y = static_cast<int>(std::floor((start.pose.position.y - wy) / res));
  const int goal_x = static_cast<int>(std::floor((goal.pose.position.x - wx) / res));
  const int goal_y = static_cast<int>(std::floor((goal.pose.position.y - wy) / res));

  const int width = costmap->getSizeInCellsX();
  const int height = costmap->getSizeInCellsY();

  const auto cost_at = [allow_unknown = allow_unknown_](
      nav2_costmap_2d::Costmap2D * cm, int x, int y) -> double {
    const unsigned char cost = cm->getCost(x, y);
    if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
      return -1.0;
    }
    if (cost == nav2_costmap_2d::NO_INFORMATION && !allow_unknown) {
      return -1.0;
    }
    return static_cast<double>(cost) / 252.0;
  };

  GridSearchResult result = grid_search(
    width, height, start_x, start_y, goal_x, goal_y,
    [&](int x, int y) { return cost_at(costmap, x, y); },
    false);

  if (!result.success) {
    RCLCPP_WARN(node_->get_logger(), "Dijkstra failed to find a path");
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
    "Dijkstra: success, %zu nodes expanded, path length %.2f m",
    result.nodes_expanded, result.path_length_cells * res);

  return path;
}

}  // namespace robot_planners

PLUGINLIB_EXPORT_CLASS(robot_planners::DijkstraPlanner, nav2_core::GlobalPlanner)
