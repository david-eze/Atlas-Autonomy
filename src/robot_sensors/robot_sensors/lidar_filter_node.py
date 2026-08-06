"""LiDAR preprocessing node.

The Gazebo ray sensor already applies a small Gaussian noise, but real
2D LiDARs additionally produce occasional missed returns (inf) and
require clamping to the valid range. This node models those effects so
downstream SLAM / costmap consumers see a realistic scan profile.

Publishes:
  /scan_filtered (sensor_msgs/LaserScan)
"""

import math
import random
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

DEFAULT_PARAMS = {
    'input_topic': '/scan',
    'output_topic': '/scan_filtered',
    'noise_stddev': 0.01,          # metres, additive Gaussian range noise
    'max_range': 15.0,
    'min_range': 0.12,
    'dropout_probability': 0.01,   # probability a single beam is a missed return
    'burst_dropout_probability': 0.003,  # probability of a short burst dropout
    'burst_length': 8,
    'update_rate': 20.0,
    'seed': 42,
}


class LidarFilterNode(Node):
    def __init__(self) -> None:
        super().__init__('lidar_filter')

        self.declare_parameter('input_topic', DEFAULT_PARAMS['input_topic'])
        self.declare_parameter('output_topic', DEFAULT_PARAMS['output_topic'])
        self.declare_parameter('noise_stddev', DEFAULT_PARAMS['noise_stddev'])
        self.declare_parameter('max_range', DEFAULT_PARAMS['max_range'])
        self.declare_parameter('min_range', DEFAULT_PARAMS['min_range'])
        self.declare_parameter('dropout_probability', DEFAULT_PARAMS['dropout_probability'])
        self.declare_parameter('burst_dropout_probability', DEFAULT_PARAMS['burst_dropout_probability'])
        self.declare_parameter('burst_length', DEFAULT_PARAMS['burst_length'])
        self.declare_parameter('seed', DEFAULT_PARAMS['seed'])

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self._noise_stddev = self.get_parameter('noise_stddev').value
        self._max_range = self.get_parameter('max_range').value
        self._min_range = self.get_parameter('min_range').value
        self._dropout_prob = self.get_parameter('dropout_probability').value
        self._burst_prob = self.get_parameter('burst_dropout_probability').value
        self._burst_length = self.get_parameter('burst_length').value
        seed = self.get_parameter('seed').value

        self._rng = random.Random(seed)

        self._scan_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self._sub = self.create_subscription(
            LaserScan, input_topic, self._on_scan, self._scan_qos)
        self._pub = self.create_publisher(
            LaserScan, output_topic, self._scan_qos)

        self.get_logger().info(
            f'Listening on {input_topic} -> {output_topic} '
            f'(noise={self._noise_stddev} m, dropout={self._dropout_prob:.3f})')

    def _on_scan(self, msg: LaserScan) -> None:
        filtered = self._apply_noise(msg.ranges)
        msg.ranges = filtered
        msg.range_min = self._min_range
        msg.range_max = self._max_range
        self._pub.publish(msg)

    def _apply_noise(self, ranges: List[float]) -> List[float]:
        """Returns a copy of ``ranges`` with additive noise, range clamping
        and occasional missed returns."""
        n = len(ranges)
        out = [0.0] * n
        i = 0
        while i < n:
            if self._rng.random() < self._burst_prob:
                for j in range(i, min(i + self._burst_length, n)):
                    out[j] = float('inf')
                i += self._burst_length
                continue

            if self._rng.random() < self._dropout_prob:
                out[i] = float('inf')
                i += 1
                continue

            value = ranges[i]
            if math.isinf(value) or math.isnan(value):
                out[i] = float('inf')
            else:
                value += self._rng.gauss(0.0, self._noise_stddev)
                out[i] = max(self._min_range, min(self._max_range, value))
            i += 1
        return out


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
