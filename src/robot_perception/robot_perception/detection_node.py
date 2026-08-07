"""YOLO-based semantic perception node.

Runs YOLO on the RGB stream, and for each detection projects the
bbox centre into the map frame using depth + TF. Publishes
vision_msgs/Detection2DArray and feeds the semantic world model.

This node only outputs labels and map coordinates, it never touches
velocity commands, so a bad detection can't directly move the robot.
"""

import time
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from .semantic_map import SemanticMap

try:
    from ultralytics import YOLO
    _HAS_YOLO = True
except ImportError:
    _HAS_YOLO = False


class DetectionNode(Node):
    def __init__(self) -> None:
        super().__init__('detection_node')

        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('camera_frame', 'camera_depth_optical_frame')

        self._confidence = self.get_parameter('confidence_threshold').value
        self._map_frame = self.get_parameter('map_frame').value
        self._camera_frame = self.get_parameter('camera_frame').value

        self.semantic_map = SemanticMap()

        # Image streams are high-rate, don't bother with reliable QoS here.
        sensor_qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)

        self._image_sub = self.create_subscription(
            Image, self.get_parameter('image_topic').value, self._on_image, sensor_qos)
        self._depth_sub = self.create_subscription(
            Image, self.get_parameter('depth_topic').value, self._on_depth, sensor_qos)
        self._info_sub = self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self._on_camera_info, sensor_qos)

        self._det_pub = self.create_publisher(
            Detection2DArray, '/detections', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._depth: Optional[np.ndarray] = None
        self._camera_info: Optional[CameraInfo] = None
        self._model = None
        if _HAS_YOLO:
            self._model = YOLO(self.get_parameter('model_path').value)
            self.get_logger().info('YOLO model loaded')
        else:
            self.get_logger().warn(
                'ultralytics not installed; detection node runs in '
                'simulation-stub mode (no real inference)')

        self._inference_times: List[float] = []

    def _on_depth(self, msg: Image) -> None:
        # Gazebo's depth camera sends 16-bit mm, hence the /1000 to metres.
        self._depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(
            msg.height, msg.width).astype(np.float32) / 1000.0

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._camera_info = msg

    def _on_image(self, msg: Image) -> None:
        if self._camera_info is None or self._depth is None:
            return

        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3)
        img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        detections = Detection2DArray()
        detections.header = msg.header

        if self._model is not None:
            start = time.perf_counter()
            results = self._model.predict(img_bgr, conf=self._confidence, verbose=False)
            elapsed = time.perf_counter() - start
            self._inference_times.append(elapsed)
            if len(self._inference_times) > 100:
                self._inference_times.pop(0)

            for result in results:
                for box in result.boxes:
                    cls_id = int(box.cls[0])
                    label = result.names[cls_id]
                    conf = float(box.conf[0])
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    cx = (x1 + x2) / 2.0
                    cy = (y1 + y2) / 2.0

                    map_xy = self._project_to_map(cx, cy, msg.header.stamp)
                    if map_xy is not None:
                        self.semantic_map.update_from_detection(
                            label, map_xy[0], map_xy[1], conf)

                    det = Detection2D()
                    det.header = msg.header
                    det.bbox.center.x = float(cx)
                    det.bbox.center.y = float(cy)
                    det.bbox.size_x = float(x2 - x1)
                    det.bbox.size_y = float(y2 - y1)
                    hyp = ObjectHypothesisWithPose()
                    hyp.hypothesis.class_id = label
                    hyp.hypothesis.score = conf
                    det.results.append(hyp)
                    detections.detections.append(det)
        else:
            # No model loaded, still publish so downstream isn't left hanging.
            pass

        self._det_pub.publish(detections)

    def _project_to_map(
        self,
        u: float,
        v: float,
        stamp,
    ) -> Optional[Tuple[float, float]]:
        """Back-project a pixel into map coordinates using depth + TF."""
        if self._depth is None or self._camera_info is None:
            return None
        u_int = int(round(u))
        v_int = int(round(v))
        if not (0 <= u_int < self._depth.shape[1] and 0 <= v_int < self._depth.shape[0]):
            return None
        z = float(self._depth[v_int, u_int])
        if z <= 0.0 or not np.isfinite(z):
            return None

        fx = self._camera_info.k[0]
        fy = self._camera_info.k[4]
        cx = self._camera_info.k[2]
        cy = self._camera_info.k[5]

        x_cam = (u - cx) * z / fx
        y_cam = (v - cy) * z / fy
        z_cam = z

        try:
            stamp_time = rclpy.time.Time.from_msg(stamp)
            transform = self.tf_buffer.lookup_transform(
                self._map_frame, self._camera_frame, stamp_time,
                timeout=rclpy.duration.Duration(seconds=0.1))
        except Exception:
            return None

        t = transform.transform.translation
        q = transform.transform.rotation
        # Using the full quaternion here instead of assuming yaw-only,
        # since the camera can be tilted relative to the base.
        px = x_cam
        py = y_cam
        pz = z_cam
        x, y, z, w = q.x, q.y, q.z, q.w
        rx = (1 - 2 * (y * y + z * z)) * px + 2 * (x * y - z * w) * py + 2 * (x * z + y * w) * pz
        ry = 2 * (x * y + z * w) * px + (1 - 2 * (x * x + z * z)) * py + 2 * (y * z - x * w) * pz
        return (t.x + rx, t.y + ry)

    def inference_latency_ms(self) -> float:
        if not self._inference_times:
            return 0.0
        return 1000.0 * sum(self._inference_times) / len(self._inference_times)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
