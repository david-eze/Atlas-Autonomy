"""Recovery manager node.

Subscribes to navigation failure signals (from the mission layer and
sensor health diagnostics) plus the AMCL pose covariance, and publishes
the selected recovery action. The mission layer executes the action;
this node only decides *what* to do, keeping the decision logic
auditable and testable.
"""

import math

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String

from .recovery_core import (
    FailureContext,
    FailureType,
    RecoveryAction,
    decide_recovery,
)


class RecoveryManager(Node):
    def __init__(self) -> None:
        super().__init__('recovery_manager')

        self.declare_parameter('localization_covariance_threshold', 0.5)
        self._cov_threshold = self.get_parameter(
            'localization_covariance_threshold').value

        self._recovery_count = 0
        self._replan_count = 0
        self._obstacle_persistence = 0
        self._sensor_health = {}
        self._localization_covariance = 0.0

        self._diag_sub = self.create_subscription(
            DiagnosticArray, '/diagnostics', self._on_diagnostics, 10)
        self._failure_sub = self.create_subscription(
            String, '/mission/failure', self._on_failure, 10)
        self._amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl_pose, 10)

        self._action_pub = self.create_publisher(String, '/mission/recovery_action', 10)
        self._log_pub = self.create_publisher(String, '/mission/recovery_log', 10)

    def _on_diagnostics(self, msg: DiagnosticArray) -> None:
        for status in msg.status:
            if status.level == DiagnosticStatus.ERROR:
                self._sensor_health[status.name] = False
            elif status.level == DiagnosticStatus.OK:
                self._sensor_health[status.name] = True

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        # The position covariance block [0, 4] is the x/y uncertainty in
        # metres^2; the diagonal sum is a cheap scalar confidence metric.
        cov = msg.pose.covariance
        self._localization_covariance = math.sqrt(
            max(0.0, cov[0]) + max(0.0, cov[7]))

    def _on_failure(self, msg: String) -> None:
        failure_str = msg.data.strip().lower()
        try:
            failure_type = FailureType(failure_str)
        except ValueError:
            failure_type = FailureType.UNKNOWN

        ctx = FailureContext(
            failure_type=failure_type,
            obstacle_persistence_count=self._obstacle_persistence,
            localization_covariance=self._localization_covariance,
            sensor_health=dict(self._sensor_health),
            replan_count=self._replan_count,
            recovery_count=self._recovery_count,
        )

        decision = decide_recovery(ctx)

        if decision.action == RecoveryAction.CLEAR_COSTMAP:
            self._obstacle_persistence += 1
        elif decision.action == RecoveryAction.REPLAN:
            self._replan_count += 1
        elif decision.action == RecoveryAction.ABORT:
            self._obstacle_persistence = 0
            self._replan_count = 0
            self._recovery_count = 0

        self._recovery_count += 1
        self._action_pub.publish(String(data=decision.action.value))
        self._log_pub.publish(String(data=decision.log_entry))
        self.get_logger().warn(f'Recovery: {decision.log_entry}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RecoveryManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
