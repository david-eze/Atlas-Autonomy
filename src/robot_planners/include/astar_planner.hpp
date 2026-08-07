// A* planner plugin. Wraps grid_search, octile heuristic.
// Publishes explored nodes for RViz.

#ifndef ROBOT_PLANNERS__ASTAR_PLANNER_HPP_
#define ROBOT_PLANNERS__ASTAR_PLANNER_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace robot_planners
{

class AstarPlanner : public nav2_core::GlobalPlanner
{
public:
  AstarPlanner() = default;
  ~AstarPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  void publish_explored_nodes(
    const std::vector<std::pair<int, int>> & nodes,
    const std_msgs::msg::Header & header);

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::string name_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    explored_pub_;
  double tolerance_{0.25};
  bool allow_unknown_{true};
};

}  // namespace robot_planners

#endif  // ROBOT_PLANNERS__ASTAR_PLANNER_HPP_
