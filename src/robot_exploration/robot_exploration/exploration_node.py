"""Frontier exploration node.

Watches the occupancy map, clusters and scores frontiers, and sends
goal poses to Nav2's NavigateToPose action. Doesn't touch velocities
directly, just hands goals off to the planner/controller stack.
Stops once coverage passes the configured threshold or there's
nothing left to explore.
"""

from typing import Dict, Tuple

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .frontiers import find_frontier_clusters, score_frontiers, UNKNOWN

DEFAULT_PARAMS = {
    'map_topic': '/map',
    'goal_topic': '/exploration_goal',
    'stop_coverage': 0.95,
    'goal_tolerance': 0.5,
    'history_increment': 1.0,
    'history_window': 3,
    'score_weights': {'gain': 1.0, 'dist': 1.0, 'obs': 0.8, 'history': 1.2},
}


class ExplorationNode(Node):
    def __init__(self) -> None:
        super().__init__('exploration_node')

        for name, default in DEFAULT_PARAMS.items():
            self.declare_parameter(name, default)

        self._stop_coverage = self.get_parameter('stop_coverage').value
        self._goal_tolerance = self.get_parameter('goal_tolerance').value
        self._history_increment = self.get_parameter('history_increment').value
        self._history_window = int(self.get_parameter('history_window').value)
        self._weights = self.get_parameter('score_weights').value

        self._map = None
        self._map_info = None
        self._history: Dict[Tuple[int, int], float] = {}

        self._map_sub = self.create_subscription(
            OccupancyGrid, self.get_parameter('map_topic').value,
            self._on_map, 10)
        self._goal_pub = self.create_publisher(
            PoseStamped, self.get_parameter('goal_topic').value, 10)
        self._frontier_pub = self.create_publisher(
            MarkerArray, '/frontiers', 10)

        # Buffer has to be built before the listener, and both need to
        # live on the node so they survive the whole spin loop.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._goal_active = False
        self._last_coverage = 0.0
        self._sent_goals = 0
        self._stale_goals: Dict[Tuple[int, int], int] = {}

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)
        self._map_info = msg.info

    def _publish_frontier_markers(
        self, frontiers, best, robot_row: int, robot_col: int) -> None:
        markers = MarkerArray()
        info = self._map_info
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        res = info.resolution

        all_cells = Marker()
        all_cells.header.frame_id = 'map'
        all_cells.header.stamp = self.get_clock().now().to_msg()
        all_cells.ns = 'frontier_cells'
        all_cells.type = Marker.POINTS
        all_cells.action = Marker.ADD
        all_cells.scale.x = res
        all_cells.scale.y = res
        all_cells.color.r = 0.0
        all_cells.color.g = 1.0
        all_cells.color.b = 0.0
        all_cells.color.a = 0.5
        for f in frontiers:
            for r, c in f.cells[:2000]:
                p = all_cells.points.add()
                p.x = origin_x + (c + 0.5) * res
                p.y = origin_y + (r + 0.5) * res
        markers.markers.append(all_cells)

        if best is not None:
            target = Marker()
            target.header = all_cells.header
            target.ns = 'frontier_target'
            target.type = Marker.SPHERE
            target.action = Marker.ADD
            target.scale.x = 0.3
            target.scale.y = 0.3
            target.scale.z = 0.3
            target.color.r = 1.0
            target.color.g = 0.0
            target.color.b = 0.0
            target.color.a = 1.0
            target.pose.position.x = origin_x + (best.centroid[1] + 0.5) * res
            target.pose.position.y = origin_y + (best.centroid[0] + 0.5) * res
            markers.markers.append(target)

        self._frontier_pub.publish(markers)

    def _robot_cell(self) -> Tuple[int, int] | None:
        """Robot's current map cell, looked up from TF."""
        try:
            # rclpy.time.Time() with no args = "give me the latest you've got"
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.2))
        except Exception:
            return None
        info = self._map_info
        col = int((transform.transform.translation.x - info.origin.position.x)
                  / info.resolution)
        row = int((transform.transform.translation.y - info.origin.position.y)
                  / info.resolution)
        return row, col

    def tick(self) -> None:
        if self._map is None or self._map_info is None:
            return

        unknown_count = int(np.count_nonzero(self._map == UNKNOWN))
        total = self._map.size
        coverage = 1.0 - unknown_count / total
        self._last_coverage = coverage

        if coverage >= self._stop_coverage:
            self.get_logger().info('Exploration complete: coverage=%.3f' % coverage)
            return

        frontiers = find_frontier_clusters(self._map)
        if not frontiers:
            self.get_logger().info('No frontiers remaining; coverage=%.3f' % coverage)
            return

        robot = self._robot_cell()
        if robot is None:
            return

        scored = score_frontiers(frontiers, robot, self._map, self._history,
                                 self._weights)
        best = scored[0]
        # Target's been stale too long, bail and grab the next unstuck one.
        if best.centroid in self._stale_goals:
            self._stale_goals[best.centroid] = self._stale_goals.get(best.centroid, 0) + 1
            if self._stale_goals[best.centroid] > 3:
                for cand in scored[1:]:
                    if cand.centroid not in self._stale_goals:
                        best = cand
                        break

        self._publish_frontier_markers(scored, best, robot[0], robot[1])

        if not self._goal_active:
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            info = self._map_info
            goal.pose.pose.position.x = info.origin.position.x + (best.centroid[1] + 0.5) * info.resolution
            goal.pose.pose.position.y = info.origin.position.y + (best.centroid[0] + 0.5) * info.resolution
            goal.pose.pose.orientation.w = 1.0
            self._nav_client.send_goal_async(
                goal,
                feedback_callback=self._on_feedback,
            ).add_done_callback(self._on_goal_done)
            self._goal_active = True
            self._sent_goals += 1
            self._history[best.centroid] = self._history.get(best.centroid, 0) + self._history_increment
            # decay so old visits stop mattering
            for key in list(self._history):
                self._history[key] *= 0.95
                if self._history[key] < 0.01:
                    del self._history[key]
            self.get_logger().info(
                f'Exploring: coverage={coverage:.3f}, frontiers={len(frontiers)}, '
                f'goal=({goal.pose.pose.position.x:.2f}, {goal.pose.pose.position.y:.2f})')

    def _on_feedback(self, feedback_msg) -> None:
        pass

    def _on_goal_done(self, future):
        try:
            future.result()
        except Exception:
            self.get_logger().warn('Navigation goal failed')
        self._goal_active = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExplorationNode()
    timer = node.create_timer(2.0, node.tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
