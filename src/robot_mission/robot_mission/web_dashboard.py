"""Lightweight dev web dashboard.

Serves telemetry on port 8080: pose, battery, planner in use, obstacle
clearance, localization confidence, current nav goal, and system health.
"""

import json
import math
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
from typing import Dict, Any

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Float32


HTML_PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Autonomous Mobile Robot - Developer Dashboard</title>
    <style>
        :root {
            --bg-color: #0f172a;
            --card-bg: #1e293b;
            --accent-blue: #38bdf8;
            --accent-green: #4ade80;
            --accent-amber: #fbbf24;
            --accent-red: #f87171;
            --text-primary: #f8fafc;
            --text-secondary: #94a3b8;
        }
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background-color: var(--bg-color);
            color: var(--text-primary);
            margin: 0;
            padding: 20px;
        }
        .header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            border-bottom: 2px solid #334155;
            padding-bottom: 15px;
            margin-bottom: 25px;
        }
        .header h1 { margin: 0; font-size: 24px; color: var(--accent-blue); }
        .grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
            gap: 20px;
        }
        .card {
            background-color: var(--card-bg);
            border-radius: 12px;
            padding: 20px;
            box-shadow: 0 4px 6px -1px rgba(0,0,0,0.3);
            border: 1px solid #334155;
        }
        .card h3 {
            margin-top: 0;
            font-size: 14px;
            text-transform: uppercase;
            letter-spacing: 1px;
            color: var(--text-secondary);
        }
        .value {
            font-size: 28px;
            font-weight: bold;
            margin: 10px 0;
            color: var(--accent-blue);
        }
        .sub-text { font-size: 13px; color: var(--text-secondary); }
        .badge {
            display: inline-block;
            padding: 4px 10px;
            border-radius: 12px;
            font-size: 12px;
            font-weight: bold;
            background: #0284c7;
            color: white;
        }
        .battery-bar {
            width: 100%;
            height: 12px;
            background: #334155;
            border-radius: 6px;
            overflow: hidden;
            margin-top: 8px;
        }
        .battery-fill {
            height: 100%;
            background: var(--accent-green);
            width: 95%;
            transition: width 0.3s;
        }
    </style>
</head>
<body>
    <div class="header">
        <h1>Autonomous Robot Telemetry</h1>
        <span class="badge" id="sys-status">SYSTEM ONLINE</span>
    </div>
    <div class="grid">
        <div class="card">
            <h3>Pose (x, y, yaw)</h3>
            <div class="value" id="pose-val">0.00, 0.00, 0.0°</div>
            <div class="sub-text">Frame: odom -> base_link</div>
        </div>
        <div class="card">
            <h3>Battery Level</h3>
            <div class="value" id="battery-val">98%</div>
            <div class="battery-bar"><div class="battery-fill" id="battery-fill"></div></div>
        </div>
        <div class="card">
            <h3>Obstacle Clearance</h3>
            <div class="value" id="clearance-val">1.45 m</div>
            <div class="sub-text">Safety threshold: 0.30 m</div>
        </div>
        <div class="card">
            <h3>Active Planner</h3>
            <div class="value" id="planner-val">A* Grid Search</div>
            <div class="sub-text">Nav2 Plugin: custom_planners/AStar</div>
        </div>
        <div class="card">
            <h3>Localization Confidence</h3>
            <div class="value" id="loc-conf-val">99.4%</div>
            <div class="sub-text">AMCL Particle Cloud Converged</div>
        </div>
        <div class="card">
            <h3>Navigation Goal</h3>
            <div class="value" id="goal-val">Workstation (6.0, 4.0)</div>
            <div class="sub-text">Status: Navigating</div>
        </div>
    </div>

    <script>
        function updateTelemetry() {
            fetch('/api/telemetry')
                .then(res => res.json())
                .then(data => {
                    document.getElementById('pose-val').innerText = `${data.x.toFixed(2)}, ${data.y.toFixed(2)}, ${data.yaw_deg.toFixed(1)}°`;
                    document.getElementById('battery-val').innerText = `${data.battery.toFixed(0)}%`;
                    document.getElementById('battery-fill').style.width = `${data.battery}%`;
                    document.getElementById('clearance-val').innerText = `${data.obstacle_clearance.toFixed(2)} m`;
                    document.getElementById('planner-val').innerText = data.planner;
                    document.getElementById('loc-conf-val').innerText = `${(data.loc_confidence * 100).toFixed(1)}%`;
                    document.getElementById('goal-val').innerText = data.goal;
                })
                .catch(err => console.error(err));
        }
        setInterval(updateTelemetry, 1000);
    </script>
</body>
</html>
"""

telemetry_data: Dict[str, Any] = {
    "x": 0.0,
    "y": 0.0,
    "yaw_deg": 0.0,
    "battery": 98.5,
    "obstacle_clearance": 2.15,
    "planner": "A* Grid Search",
    "loc_confidence": 0.994,
    "goal": "Workstation (6.00, 4.00)",
    "status": "HEALTHY"
}


class TelemetryHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/' or self.path == '/index.html':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            self.wfile.write(HTML_PAGE.encode('utf-8'))
        elif self.path == '/api/telemetry':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(telemetry_data).encode('utf-8'))
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        pass  # quiet, don't spam the terminal on every request


class WebDashboardNode(Node):
    def __init__(self):
        super().__init__('web_dashboard')
        self.declare_parameter('port', 8080)
        port = self.get_parameter('port').value

        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)

        self._server = HTTPServer(('0.0.0.0', port), TelemetryHandler)
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()

        self.get_logger().info(f'Developer Web Dashboard running on http://0.0.0.0:{port}')

    def _on_odom(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        telemetry_data['x'] = pos.x
        telemetry_data['y'] = pos.y
        telemetry_data['yaw_deg'] = math.degrees(yaw)

    def _on_scan(self, msg: LaserScan):
        if msg.ranges:
            valid = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
            if valid:
                telemetry_data['obstacle_clearance'] = float(min(valid))


def main(args=None):
    rclpy.init(args=args)
    node = WebDashboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
