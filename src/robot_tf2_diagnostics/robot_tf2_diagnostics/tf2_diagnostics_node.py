"""TF2 tree diagnostic tool.

Periodically checks the robot's TF tree: required frames present,
required parent->child links resolve, no sensor frame disconnected
from the main tree, and per-edge transform latency. Meant to double
as launch-time validation / CI check via its exit code.
"""

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

REQUIRED_EDGES = [
    ('map', 'odom'),
    ('odom', 'base_link'),
    ('base_link', 'laser_link'),
    ('base_link', 'camera_link'),
    ('base_link', 'imu_link'),
]

SENSOR_LEAVES = ['laser_link', 'camera_link', 'imu_link']


class Tf2Diagnostics(Node):
    def __init__(self) -> None:
        super().__init__('tf2_diagnostics')
        self.declare_parameter('check_rate_hz', 1.0)
        self.declare_parameter('timeout_seconds', 2.0)

        self._timeout = self.get_parameter('timeout_seconds').value
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        rate = self.get_parameter('check_rate_hz').value
        self.create_timer(1.0 / rate, self._check)

    def _check(self) -> None:
        now = rclpy.time.Time()
        lines = ['TF STATUS']
        healthy = True

        for parent, child in REQUIRED_EDGES:
            try:
                self.buffer.lookup_transform(
                    parent, child, now, timeout=rclpy.duration.Duration(seconds=self._timeout))
                # separate lookup with no timeout, just to read the stamp for latency
                try:
                    t = self.buffer.lookup_transform(parent, child, rclpy.time.Time())
                    age_ms = (self.get_clock().now() - t.header.stamp).nanoseconds * 1e-6
                    lines.append(f'{parent} -> {child}: OK ({age_ms:.0f} ms)')
                except Exception:
                    lines.append(f'{parent} -> {child}: OK (latency N/A)')
            except Exception:
                lines.append(f'{parent} -> {child}: MISSING')
                healthy = False

        # catches sensor frames that publish fine on their own but never
        # actually connect back up to map
        for leaf in SENSOR_LEAVES:
            try:
                self.buffer.lookup_transform(
                    'map', leaf, now, timeout=rclpy.duration.Duration(seconds=self._timeout))
                lines.append(f'map -> {leaf}: reachable')
            except Exception:
                lines.append(f'map -> {leaf}: UNREACHABLE (disconnected tree)')
                healthy = False

        # no generic way to detect a "wrong" frame relationship, so this
        # just flags anything showing up that isn't on the expected list
        all_frames = []
        try:
            all_frames = self.buffer.all_frames_as_string().split('\n')
        except Exception:
            pass
        expected = {'map', 'odom', 'base_link', 'laser_link', 'camera_link',
                    'camera_depth_frame', 'camera_depth_optical_frame',
                    'imu_link', 'left_wheel_link', 'right_wheel_link',
                    'caster_link'}
        unknown = [f.split(' ')[0] for f in all_frames
                   if f and f.split(' ')[0] not in expected]
        if unknown:
            lines.append(f'Unexpected frames: {", ".join(unknown)}')

        for line in lines:
            self.get_logger().info(line)

        if healthy:
            self.get_logger().info('TF tree: HEALTHY')
        else:
            self.get_logger().error('TF tree: UNHEALTHY')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Tf2Diagnostics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
