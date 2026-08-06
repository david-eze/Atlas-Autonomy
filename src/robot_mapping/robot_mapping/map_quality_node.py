"""Periodically computes map quality stats from the live occupancy grid."""

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float64

from .map_quality import map_statistics


class MapQualityNode(Node):
    def __init__(self) -> None:
        super().__init__('map_quality_monitor')

        self._map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._on_map, 10)
        self._pub = self.create_publisher(Float64, '/map_coverage', 10)
        self._last_map = None

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._last_map = msg

    def tick(self) -> None:
        if self._last_map is None:
            return
        grid = np.array(self._last_map.data, dtype=np.int8).reshape(
            self._last_map.info.height, self._last_map.info.width)
        stats = map_statistics(grid)
        self.get_logger().info(
            f'coverage={stats["coverage"]:.2f} '
            f'unknown={stats["unknown_fraction"]:.2f} '
            f'frontiers={int(stats["frontier_count"])}')
        self._pub.publish(Float64(data=float(stats['coverage'])))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapQualityNode()
    # Process the cached map on a timer rather than in the subscription
    # callback, so the coverage metric updates at a fixed rate even if
    # the map topic publishes faster than we want to log.
    timer = node.create_timer(1.0, node.tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
