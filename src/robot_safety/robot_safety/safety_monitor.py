"""Independent safety monitor node.

Subscribes to the commanded velocity and the raw LiDAR scan, evaluates
the command against the deterministic safety core, and republishes a
safe velocity on /cmd_vel_safe. diff_drive_controller listens to the
safe topic, so this sits between any velocity source (Nav2, teleop,
AI) and the motors.

No dependency on Nav2 or the AI stack on purpose, so a higher-level
component can't bypass it.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32

from .safety_core import SafetyConfig, evaluate_command


class SafetyMonitor(Node):
    def __init__(self) -> None:
        super().__init__('safety_monitor')

        self.declare_parameter('scan_topic', '/scan_filtered')
        self.declare_parameter('cmd_vel_in', '/cmd_vel')
        self.declare_parameter('cmd_vel_out', '/cmd_vel_safe')
        self.declare_parameter('min_obstacle_distance', 0.25)
        self.declare_parameter('hard_stop_distance', 0.20)
        self.declare_parameter('max_linear_velocity', 1.2)
        self.declare_parameter('max_angular_velocity', 1.5)
        self.declare_parameter('max_scan_age', 0.5)

        self._config = SafetyConfig(
            min_obstacle_distance=self.get_parameter('min_obstacle_distance').value,
            hard_stop_distance=self.get_parameter('hard_stop_distance').value,
            max_linear_velocity=self.get_parameter('max_linear_velocity').value,
            max_angular_velocity=self.get_parameter('max_angular_velocity').value,
            max_scan_age=self.get_parameter('max_scan_age').value,
        )

        self._scan = None
        self._scan_stamp = None

        self._scan_sub = self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value, self._on_scan, 10)
        self._cmd_sub = self.create_subscription(
            Twist, self.get_parameter('cmd_vel_in').value, self._on_cmd, 10)

        self._cmd_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_out').value, 10)
        self._status_pub = self.create_publisher(Bool, '/safety/active', 10)
        self._min_dist_pub = self.create_publisher(Float32, '/safety/min_distance', 10)

    def _on_scan(self, msg: LaserScan) -> None:
        self._scan = msg
        self._scan_stamp = self.get_clock().now()

    def _on_cmd(self, msg: Twist) -> None:
        if self._scan is None:
            self._publish_zero('no_scan')
            return

        age = (self.get_clock().now() - self._scan_stamp).nanoseconds * 1e-9
        if age > self._config.max_scan_age:
            self._publish_zero('stale_scan')
            return

        decision = evaluate_command(
            msg.linear.x,
            msg.angular.z,
            list(self._scan.ranges),
            self._scan.angle_min,
            self._scan.angle_increment,
            self._scan.range_max,
            self._config,
        )

        out = Twist()
        out.linear.x = decision.override_linear
        out.angular.z = decision.override_angular
        self._cmd_pub.publish(out)

        self._status_pub.publish(Bool(data=not decision.safe))
        self._min_dist_pub.publish(Float32(data=float(decision.min_distance)))

        if not decision.safe:
            self.get_logger().warn(
                f'Safety override ({decision.reason}): '
                f'min_dist={decision.min_distance:.3f} m')

    def _publish_zero(self, reason: str) -> None:
        out = Twist()
        self._cmd_pub.publish(out)
        self._status_pub.publish(Bool(data=True))
        self.get_logger().warn(f'Safety stop: {reason}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
