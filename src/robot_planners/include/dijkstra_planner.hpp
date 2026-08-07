// Dijkstra planner plugin. Same as AstarPlanner but heuristic disabled
// (uniform expansion from start). Separate class so the plugin registry
// and benchmarks can tell them apart.

#ifndef ROBOT_PLANNERS__DIJKSTRA_PLANNER_HPP_
#define ROBOT_PLANNERS__DIJKSTRA_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace robot_planners
{

class DijkstraPlanner : public nav2_core::GlobalPlanner
{
public:
  DijkstraPlanner() = default;
  ~DijkstraPlanner() override = default;

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
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string name_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    explored_pub_;
  bool allow_unknown_{true};
};

}  // namespace robot_planners

#endif  // ROBOT_PLANNERS__DIJKSTRA_PLANNER_HPP_
