"""Sensor health monitor for the fusion pipeline.

Tracks message arrival rates on key topics (odometry, IMU, LiDAR-
filtered scan, velocity commands) and publishes a diagnostic array
consumed by the mission/recovery layer. This is deliberately separate
from runtime safety: health monitoring informs high-level decisions,
it never directly intervenes in low-level control.
"""

import math
from typing import Dict

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan

DEFAULT_PARAMS = {
    'timeout_seconds': 1.0,
    'publish_rate_hz': 2.0,
}


class SensorHealthMonitor(Node):
    def __init__(self) -> None:
        super().__init__('sensor_health_monitor')

        self.declare_parameter('timeout_seconds', DEFAULT_PARAMS['timeout_seconds'])
        self.declare_parameter('publish_rate_hz', DEFAULT_PARAMS['publish_rate_hz'])

        self._timeout = self.get_parameter('timeout_seconds').value
        self._last_seen: Dict[str, float] = {}

        self._monitored = {
            '/odom': Odometry,
            '/imu/data': Imu,
            '/scan_filtered': LaserScan,
            '/cmd_vel': Twist,
        }

        for topic in self._monitored:
            self._last_seen[topic] = -math.inf
            self.create_subscription(
                self._monitored[topic], topic,
                lambda msg, t=topic: self._mark(t),
                10)

        self._diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        timer_period = 1.0 / self.get_parameter('publish_rate_hz').value
        self.create_timer(timer_period, self._publish)

    def _mark(self, topic: str) -> None:
        self._last_seen[topic] = self.get_clock().now().nanoseconds * 1e-9

    def _publish(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()
        for topic, last in self._last_seen.items():
            status = DiagnosticStatus()
            status.name = f'sensor/{topic.lstrip("/")}'
            seconds_since = now - last
            if seconds_since > self._timeout:
                status.level = DiagnosticStatus.ERROR
                status.message = f'No data on {topic} for >{self._timeout:.1f} s'
            elif seconds_since > self._timeout * 0.5:
                status.level = DiagnosticStatus.WARN
                status.message = f'Data on {topic} arriving slowly'
            else:
                status.level = DiagnosticStatus.OK
                status.message = f'Data on {topic} arriving normally'
            status.values = [
                KeyValue(key='last_seen_s', value=f'{seconds_since:.2f}')
            ]
            diag.status.append(status)
        self._diag_pub.publish(diag)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SensorHealthMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
