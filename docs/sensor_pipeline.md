# Sensor Data Processing Pipeline

## Overview
Sensors feed filtered data to the state estimation, mapping, and navigation costmaps.

```
[2D LiDAR] ----> Range Filter ----> Scan Filtering ----> SLAM / Costmap
[Encoders] ----> Diff Drive Kinematics ---\
                                           +---> EKF Fusion -> odom->base_link
[IMU] --------> Gyro/Accel Filter --------/
[RGB-D Camera] -> Object Detector Model -> Semantic World Model -> Nav Goal
```

## LiDAR Filtering (`robot_sensors/lidar_filter_node`)

* **Range Clipping**: Drops measurements outside the sensor's usable range (`range_min`: 0.12 m, `range_max`: 12.0 m).
* **Noise Injection Simulation**: Adds configurable Gaussian noise ($\sigma = 0.01\text{ m}$) and random measurement dropouts to test how the system handles imperfect sensor data.

## Wheel Encoders & IMU Fusion (`robot_localization`)

* Wheel encoders track the rotation of the left and right wheels, which is used to calculate the robot's linear and angular motion using differential-drive kinematics.
* The IMU provides a higher-rate angular velocity measurement ($\omega_z$), helping reduce yaw errors caused by wheel slip.
* **Extended Kalman Filter (EKF)**: Fuses encoder velocity ($v_x$) with the IMU yaw rate ($\omega_z$) to produce a smoother $50\text{ Hz}$ odometry stream.

## RGB-D Camera Pipeline (`robot_perception`)

* Color frames ($640 \times 480 @ 30\text{ FPS}$) are passed through the object-detection model wrapper.
* The depth map is used to convert 2D detections into 3D coordinates $(x, y, z)$ in the robot/world frame.
