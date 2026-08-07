"""Semantic navigation node.

Accepts a friendly command like "go to the workstation", resolves it to
map coordinates via the semantic world model, and sends it to Nav2 as a
NavigateToPose goal.

The node never computes velocities. It converts high-level intentions
into goals; the deterministic Nav2 stack executes them.
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String


class SemanticNavigationNode(Node):
    def __init__(self) -> None:
        super().__init__('semantic_navigation')

        self.declare_parameter('input_topic', '/mission/command')
        self.declare_parameter('map_frame', 'map')

        self._map_frame = self.get_parameter('map_frame').value

        # Predefined place names for the demonstration. Detected objects
        # are registered at runtime by the perception node via the shared
        # semantic map service
        self._locations = {
            'charging_station': (2.0, 1.5),
            'workstation': (6.0, 4.0),
            'storage_area': (1.0, 8.0),
            'office': (9.0, 2.0),
        }

        self._cmd_sub = self.create_subscription(
            String, self.get_parameter('input_topic').value,
            self._on_command, 10)

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._current_goal = None

    def _parse_target(self, command: str) -> str:
        """Strip a leading verb and article to extract the target name."""
        lowered = command.strip().lower()
        for verb in ('go to ', 'navigate to ', 'drive to ', 'take me to '):
            if lowered.startswith(verb):
                lowered = lowered[len(verb):]
                break
        if lowered.startswith('the '):
            lowered = lowered[4:]
        return lowered.strip()

    def _on_command(self, msg: String) -> None:
        target = self._parse_target(msg.data)

        if target not in self._locations:
            self.get_logger().warn(f'Unknown semantic target: {target}')
            self.get_logger().info(
                f'Known locations: {", ".join(sorted(self._locations))}')
            return

        x, y = self._locations[target]
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self._map_frame
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.w = 1.0

        self.get_logger().info(
            f'Semantic command "{msg.data}" -> goal ({x:.2f}, {y:.2f})')

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 navigate_to_pose action server unavailable')
            return

        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        self._current_goal = target
        send_future = self._nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Navigation goal rejected')
            return
        self.get_logger().info('Navigation goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future) -> None:
        result = future.result().result
        target = self._current_goal
        if result is not None:
            self.get_logger().info(f'Navigation to {target} succeeded')
        else:
            self.get_logger().warn(f'Navigation to {target} failed')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SemanticNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
